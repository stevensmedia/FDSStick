#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "device.h"
#include "fds.h"
#include "spi.h"
#include "os.h"

/*
Disk format in flash:
struct {
    uint16_t filename[120];  //null terminated unicode string.  filename[0]: 0xFFFF=empty, 0x0000=multi-disk image (continued from previous)
    uint8_t reserved[14];   //set to 0
    uint16_t lead_in;       //lead-in length (#bits), 0=default
    uint8_t data[0xff00];   //disk data, beginning with gap end mark (0x80) of first block
}
*/

enum {
    DEFAULT_LEAD_IN=28300,      //#bits
    GAP=976/8-1,
    MIN_GAP_SIZE=0x300,         //bits
    FDSSIZE=65500,              //size of .fds disk side, excluding header
    FLASHHEADERSIZE=0x100,
    FLASHRAWSIZE=SLOTSIZE-256,  //size of image in flash, excluding header
};


static void raw03_to_bin(uint8_t *raw, int rawSize, uint8_t **_bin, int *_binSize);


//don't include gap end
uint16_t calc_crc(uint8_t *buf, int size) {
    uint32_t crc=0x8000;
    int i;
    while(size--) {
        crc |= (*buf++)<<16;
        for(i=0;i<8;i++) {
            if(crc & 1) crc ^= 0x10810;
            crc>>=1;
        }
    }
    return crc;
}

void copy_block(uint8_t *dst, uint8_t *src, int size) {
    dst[0] = 0x80;
    memcpy(dst+1, src, size);
    uint32_t crc = calc_crc(dst+1, size+2);
    dst[size+1]=crc;
    dst[size+2]=crc>>8;
}

//Adds GAP + GAP end (0x80) + CRCs to .FDS image
//Returns size (0=error)
// * TODO - this is also used for disk writing, dst size shouldn't be constrained to one "slot"
int fds_to_bin(uint8_t *dst, uint8_t *src) {
    int i=0, o=0;

    //check *NINTENDO-HVC* header
    if(src[0]!=0x01 || src[1]!=0x2a || src[2]!=0x4e) {
        printf("Not an FDS file.\n");
        return 0;
    }
    memset(dst, 0, FLASHRAWSIZE);

    //block type 1
    copy_block(dst+o, src+i, 0x38);
    i+=0x38;
    o+=0x38+3+GAP;

    //block type 2
    copy_block(dst+o, src+i, 2);
    i+=2;
    o+=2+3+GAP;

    //block type 3+4...
    while(src[i]==3) {
        uint32_t size = (src[i+13] | (src[i+14]<<8))+1;
        if(o + 16+3 + GAP + size+3 > FLASHRAWSIZE) {
            printf("Out of space (%d bytes short), adjust GAP size?\n",(o + 16+3 + GAP + size+3)-FLASHRAWSIZE);
            return 0;
        }
        copy_block(dst+o, src+i, 16);
        i+=16;
        o+=16+3+GAP;

        copy_block(dst+o, src+i, size);
        i+=size;
        o+=size+3+GAP;
    }
    return o;
}


//look for pattern of bits matching block 1
static int findFirstBlock(uint8_t *raw) {
    static const uint8_t dat[]={1,0,1,0,0,0,0,0, 0,1,2,2,1,0,1,0, 0,1,1,2,1,1,1,1, 1,1,0,0,1,1,1,0};
    int i,len;
    for(i=0, len=0; i<0x2000*8; i++) {
        if(raw[i]==dat[len]) {
            if(len==sizeof(dat)-1)
                return i-len;
            len++;
        } else {
            i-=len;
            len=0;
        }
    }
    return -1;
}

bool block_decode(uint8_t *dst, uint8_t *src, int *inP, int *outP, int srcSize, int dstSize, int blockSize, char blockType) {
    if(*outP+blockSize+2 > dstSize) {
        printf("Out of space\n");
        return false;
    }

    int in=*inP;
    int outEnd=(*outP+blockSize+2)*8;
    int out=(*outP)*8;
    int start;

    //scan for gap end
    for(int zeros=0; src[in]!=1 || zeros<MIN_GAP_SIZE; in++) {
        if(src[in]==0) {
            zeros++;
        } else {
            zeros=0;
        }
        if(in>=srcSize-2)
            return false;
    }
    start=in;

    char bitval=1;
    in++;
    do {
        if(in>=srcSize) {   //not necessarily an error, probably garbage at end of disk
            //printf("Disk end\n"); 
            return false;
        }
        switch(src[in]|(bitval<<4)) {
            case 0x11:
                out++;
            case 0x00:
                out++;
                bitval=0;
                break;
            case 0x12:
                out++;
            case 0x01:
            case 0x10:
                dst[out/8] |= 1<<(out&7);
                out++;
                bitval=1;
                break;
            default: //Unexpected value.  Keep going, we'll probably get a CRC warning
                //printf("glitch(%d) @ %X(%X.%d)\n", src[in], in, out/8, out%8);
                out++;
                bitval=0;
                break;
        }
        in++;
    } while(out<outEnd);
    if(dst[*outP]!=blockType) {
        printf("Wrong block type %X(%X)-%X(%X)\n", start, *outP, in, out-1);
        return false;
    }
    out=out/8-2;

    //printf("Out%d %X(%X)-%X(%X)\n", blockType, start, *outP, in, out-1);

    if(calc_crc(dst+*outP,blockSize+2)) {
        uint16_t crc1=(dst[out+1]<<8)|dst[out];
        dst[out]=0;
        dst[out+1]=0;
        uint16_t crc2=calc_crc(dst+*outP,blockSize+2);
        printf("Bad CRC (%04X!=%04X)\n", crc1, crc2);
    }

    dst[out]=0;     //clear CRC
    dst[out+1]=0;
    dst[out+2]=0;   //+spare bit
    *inP=in;
    *outP=out;
    return true;
}

//Turn raw data from adapter to pulse widths (0..3)
//Input capture clock is 6MHz.  At 96.4kHz (FDS bitrate), 1 bit ~= 62 clocks
//We could be clever about this and account for drive speed fluctuations, etc. but this will do for now
static void raw_to_raw03(uint8_t *raw, int rawSize) {
    for(int i=0; i<rawSize; ++i) {
        if(raw[i]<0x30) raw[i]=3;
        else if(raw[i]<0x50) raw[i]=0;      //0=1 bit
        else if(raw[i]<0x70) raw[i]=1;      //1=1.5 bits
        else if(raw[i]<0xa0) raw[i]=2;      //2=2 bits
        else raw[i]=3;                      //3=out of range
    }
}

//Simplified disk decoding.  This assumes disk will follow standard FDS file structure
static bool raw03_to_fds(uint8_t *raw, uint8_t *fds, int rawsize) {
    int in,out;

    memset(fds,0,FDSSIZE);

    //lead-in can vary a lot depending on drive, scan for first block to get our bearings
    in=findFirstBlock(raw)-MIN_GAP_SIZE;
    if(in<0)
        return false;

    out=0;
    if(!block_decode(fds, raw, &in, &out, rawsize, FDSSIZE+2, 0x38, 1))
        return false;
    if(!block_decode(fds, raw, &in, &out, rawsize, FDSSIZE+2, 2, 2))
        return false;
    do {
        if(!block_decode(fds, raw, &in, &out, rawsize, FDSSIZE+2, 16, 3))
            return true;
        if(!block_decode(fds, raw, &in, &out, rawsize, FDSSIZE+2, 1+(fds[out-16+13] | (fds[out-16+14]<<8)), 4))
            return true;
    } while(in<rawsize);
    return true;
}

// TODO - only handles one side, files will need to be joined manually
bool FDS_readDisk(char *filename_raw, char *filename_bin, char *filename_fds) {
    enum { READBUFSIZE=0x90000 };

    FILE *f;
    uint8_t *readBuf=NULL;
    int result;
    int bytesIn=0;
    uint8_t sequence=1;

    //if(!(dev_readIO()&MEDIA_SET)) {
    //    printf("Warning - Disk not inserted?\n");
    //}
    if(!dev_readStart())
        return false;

    readBuf=(uint8_t*)malloc(READBUFSIZE);
    do {
        result=dev_readDisk(readBuf+bytesIn);
        bytesIn+=result;
        if(!(bytesIn%((DISK_READMAX)*32)))
            printf(".");
    } while(result==DISK_READMAX && bytesIn<READBUFSIZE-DISK_READMAX);
    printf("\n");
    if(result<0) {
        printf("Read error.\n");
        return false;
    }

    if(filename_raw) {
        if( (f=fopen(filename_raw,"wb")) ) {
            fwrite(readBuf, 1, bytesIn, f);
            fclose(f);
            printf("Wrote %s\n",filename_raw);
        }
    }

    raw_to_raw03(readBuf, bytesIn);

    //decode to .fds
    if(filename_fds) {
        uint8_t *fds=(uint8_t*)malloc(FDSSIZE+16);   //extra room for CRC junk
        raw03_to_fds(readBuf, fds, bytesIn);
        if( (f=fopen(filename_fds,"wb")) ) {
            fwrite(fds, 1, FDSSIZE, f);
            fclose(f);
            printf("Wrote %s\n",filename_fds);
        }
        free(fds);

    //decode to .bin
    } else if(filename_bin) {
        uint8_t *binBuf;
        int binSize;

        raw03_to_bin(readBuf, bytesIn, &binBuf, &binSize);
        if( (f=fopen(filename_bin, "wb")) ) {
            fwrite(binBuf, 1, binSize, f);
            fclose(f);
            printf("Wrote %s\n", filename_bin);
        }
        free(binBuf);
    }

    free(readBuf);
    return true;
}

static bool writeDisk(uint8_t *buf, int size) {
    int bytesOut;

    //if(!(readIO()&MEDIA_SET)) {
    //    printf("Disk not inserted.\n");
    //}
    if(!dev_writeStart())
        return false;

    for(bytesOut=0; bytesOut<size; bytesOut+=DISK_WRITEMAX) {
        if(!dev_writeDisk(buf+bytesOut, DISK_WRITEMAX)) {
            printf("Write error (disk full?)\n");
            return false;
        }
        if(!(bytesOut%(DISK_WRITEMAX*16)))
            printf("#");
    }

    //Fill remainder with empty space.   Keep writing until we can't, EP0 will stall at end of disk
    memset(buf, 0xaa, DISK_WRITEMAX);
    for(bytesOut=0; bytesOut<0x20000; bytesOut+=DISK_WRITEMAX) {
        if(!dev_writeDisk(buf, DISK_WRITEMAX))
            break;
        if(!(bytesOut%(DISK_WRITEMAX*16)))
            printf(".");
    }
    return true;
}

bool FDS_writeDisk(char *name) {
    static const uint8_t expand[]={ 0xaa, 0xa9, 0xa6, 0xa5, 0x9a, 0x99, 0x96, 0x95, 0x6a, 0x69, 0x66, 0x65, 0x5a, 0x59, 0x56, 0x55 };
    enum { WRITESIZE=(FLASHRAWSIZE+DEFAULT_LEAD_IN/8)*2, };
    uint8_t *inbuf=0;       //.FDS buffer
    uint8_t *bin=0;         //.FDS with gaps/CRC
    uint8_t *outbuf=0;      //bitstream
    FILE *f;

    f=fopen(name,"rb");
    if(!f) { printf("Can't open %s\n", name); return false; }

    fseek(f,0,SEEK_END);
    int filesize=ftell(f);
    fseek(f,0,SEEK_SET);
    filesize=16 + FDSSIZE*((filesize+FDSSIZE-17)/FDSSIZE);  //pad up to whole disk size

    inbuf=(uint8_t*)malloc(filesize);
    outbuf=(uint8_t*)malloc(WRITESIZE);
    bin=(uint8_t*)malloc(FLASHRAWSIZE);     //fix this, we shouldn't be limited by flash size

    memset(inbuf,0,filesize);
    fread(inbuf,1,filesize,f);
    fclose(f);

    int inpos=0, side=0;
    if(inbuf[0]=='F')
        inpos=16;      //skip fwNES header

    char prompt;
    do {
        printf("Side %d\n", side+1);

        int imgsize=fds_to_bin(bin, inbuf+inpos);
        if(!imgsize)
            break;

        //expand to bitpattern for writing
        memset(outbuf, 0xaa, WRITESIZE);
        for(int i=0;i<imgsize;i++) {
            outbuf[DEFAULT_LEAD_IN/8*2 + i*2 + 0]=expand[bin[i]&0x0f];
            outbuf[DEFAULT_LEAD_IN/8*2 + i*2 + 1]=expand[(bin[i]>>4)&0x0f];
        }

        if(!writeDisk(outbuf, (imgsize+DEFAULT_LEAD_IN/8)*2))
            break;
        inpos+=FDSSIZE;
        side++;

        //prompt for disk change
        prompt=0;
        if(inpos<filesize && inbuf[inpos]==0x01) {
            printf("Push ENTER for next disk side (or any other key to stop)\n");
            prompt=readKb();
        }
    } while(prompt==0x0d);

    free(bin);
    free(outbuf);
    free(inbuf);
    return true;
}

//slot 1..n
bool FDS_writeFlash(char *name, int slot) {
    enum { FILENAMELENGTH=120, };   //number of characters including null

    uint8_t *inbuf=0;
    uint8_t *outbuf=0;
    FILE *f;

    f=fopen(name,"rb");
    if(!f) return false;

    fseek(f,0,SEEK_END);
    int filesize=ftell(f);
    fseek(f,0,SEEK_SET);
    filesize=16 + FDSSIZE*((filesize+FDSSIZE-17)/FDSSIZE);  //pad up to whole disk size

    inbuf=(uint8_t*)malloc(filesize);
    outbuf=(uint8_t*)malloc(SLOTSIZE);
    memset(inbuf,0,filesize);
    fread(inbuf,1,filesize,f);
    fclose(f);

    int pos=0, side=0;
    if(inbuf[0]=='F')
        pos=16;      //skip fwNES header

    do {
        printf("Side %d\n", side+1);
        if(fds_to_bin(outbuf+FLASHHEADERSIZE, inbuf+pos)) {
            memset(outbuf,0,FLASHHEADERSIZE);
            outbuf[0xfe]=DEFAULT_LEAD_IN & 0xff;
            outbuf[0xff]=DEFAULT_LEAD_IN / 256;
            if(side==0) {
                //strip path from filename
                char *shortName=strrchr(name,'/');      // ...dir/file.fds
#ifdef _WIN32
                if(!shortName)
                    shortName=strrchr(name,'\\');        // ...dir\file.fds
                if(!shortName)
                    shortName=strchr(name,':');         // C:file.fds
#endif
                if(!shortName)
                    shortName=name;
                else
                    shortName++;
                utf8_to_utf16((uint16_t*)outbuf, shortName, FILENAMELENGTH*2);
                ((uint16_t*)outbuf)[FILENAMELENGTH-1]=0;
            }
            spi_writeFlash(outbuf, (slot+side-1)*SLOTSIZE, SLOTSIZE);
        }
        pos+=FDSSIZE;
        side++;
    } while(pos<filesize && inbuf[pos]==0x01);
    free(inbuf);
    free(outbuf);
    return true;
}

bool FDS_list() {
    uint8_t buf[256];
    int side=0;
    for(int slot=1;slot<=dev_slots;slot++) {
        if(!spi_readFlash((slot-1)*SLOTSIZE,buf,256))
            return false;

        if(buf[0]==0xff) {          //empty
            printf("%d:\n",slot);
            side=0;
        } else if(buf[0]!=0) {      //filename present
            wprintf(L"%d: %s\n", slot, buf);
            side=1;
        } else if(!side) {          //first side is missing
            printf("%d: ?\n", slot);
        } else {                    //next side
            printf("%d:    Side %d\n", slot, ++side);
        }
    }
    return true;
}

//===============================

//check for gap at EOF
bool looks_like_file_end(uint8_t *raw, int start, int rawSize) {
    enum {
        MIN_GAP=976-100,
        MAX_GAP=976+100,
    };
    int zeros=0;
    int in=start;
    for(; in<start+MAX_GAP && in<rawSize; in++) {
        if(raw[in]==1 && zeros>MIN_GAP) {
            return true;
        } else if(raw[in]==0) {
            zeros++;
        }
        if(raw[in]!=0)
            zeros=0;
    }
    return in>=rawSize;  //end of disk = end of file!
}

//detect EOF by looking for good CRC.  in=start of file
//returns 0 if nothing found
int crc_detect(uint8_t *raw, int in, int rawSize) {
    static uint32_t crc;
    static uint8_t bitval;
    static int out;
    static bool match;

    //local function ;)
    struct { void shift(uint8_t bit) {
        crc|=bit<<16;
        if(crc & 1) crc ^= 0x10810;
        crc>>=1;
        bitval=bit;
        out++;
        if(crc==0 && !(out&7))  //on a byte bounary and CRC is valid
            match=true;
    }} f;

    crc=0x8000;
    bitval=1;
    out=0;
    do {
        match=false;
        switch(raw[in]|(bitval<<4)) {
            case 0x11:
                f.shift(0);
            case 0x00:
                f.shift(0);
                break;
            case 0x12:
                f.shift(0);
            case 0x01:
            case 0x10:
                f.shift(1);
                break;
            default:    //garbage / bad encoding
                return 0;
        }
        in++;
    } while(in<rawSize && !(match && looks_like_file_end(raw,in,rawSize)));
    return match? in: 0;
}

//gap end is known, backtrack and mark the start.  !! this assumes junk data exists between EOF and gap start
static void mark_gap_start(uint8_t *raw, int gapEnd) {
    int i;
    for(i=gapEnd-1; i>=0 && raw[i]==0; --i)
        { }
    raw[i+1]=3;
    printf("mark gap %X-%X\n", i+1, gapEnd);
}

//For information only for now.  This checks for standard file format
static void verify_block(uint8_t *bin, int start, int *reverse) {
    enum { MAX_GAP=(976+100)/8, MIN_GAP=(976-100)/8 };
    static const uint8_t next[]={0,2,3,4,3};
    static int last = 0;
    static int lastLen = 0;
    static int blockCount = 0;

    int len=0;
    uint8_t type=bin[start];

    printf("%d:%X", ++blockCount, type);

    switch(type) {
        case 1:
            len=0x38;
            break;
        case 2:
            len=2;
            break;
        case 3:
            len=16;
            break;
        case 4:
            len=1+(bin[last+13] | (bin[last+14]<<8));
            break;
        default:
            printf(" bad block (%X)\n",start);
            return;
    }
    printf(" %X-%X / %X-%X(%X)", reverse[start], reverse[start+len], start, start+len, len);

    if((!last && type!=1) || (last && type!=next[bin[last]]))
        printf(", wrong filetype");
    if(calc_crc(bin+start, len+2)!=0)
        printf(", bad CRC");
    if(last && (last+lastLen+MAX_GAP)<start)
        printf(", lost block?");
    if(last+lastLen+MIN_GAP>start)
        printf(", block overlap?");
    //if(type==3 && ...)    //check other fields in file header?

    printf("\n");
    last=start;
    lastLen=len;
}

//find gap + gap end.  returns bit following gap end, >=rawSize if not found.
int nextGapEnd(uint8_t *raw, int in, int rawSize) {
    enum { MIN_GAP=976-100, };
    int zeros=0;
    for(; (raw[in]!=1 || zeros<MIN_GAP) && in<rawSize; in++) {
        if(raw[in]==0) {
            zeros++;
        } else {
            zeros=0;
        }
    }
    return in+1;
}

/*
Try to create byte-for-byte, unadulterated representation of disk.  Use hints from the disk structure, given
that it's probably a standard FDS game image but this should still make a best attempt regardless of the disk content.  

_bin and _binSize are updated on exit.  alloc'd buffer is returned in _bin, caller is responsible for freeing it.
*/
static void raw03_to_bin(uint8_t *raw, int rawSize, uint8_t **_bin, int *_binSize) {
    enum {
        BINSIZE=0xa0000,
        POST_GLITCH_GARBAGE=16,
        LONG_POST_GLITCH_GARBAGE=64,
        LONG_GAP=900,   //976 typ.
        SHORT_GAP=16,
    };
    int in, out;
    uint8_t *bin;
    int *reverse;
    int glitch;
    int zeros;

    bin=(uint8_t*)malloc(BINSIZE);
    reverse=(int*)malloc(BINSIZE*sizeof(int));
    memset(bin,0,BINSIZE);

    //--- assume any glitch is OOB, mark a run of zeros near a glitch as a gap start.

    int junk=0;
    glitch=0;
    zeros=0;
    junk=0;
    for(in=0; in<rawSize; in++) {
        if(raw[in]==3) {
            glitch=in;
            junk=0;
        } else if(raw[in]==1 || raw[in]==2) {
            junk=in;
        } else if(raw[in]==0) {
            zeros++;
            if(glitch && junk && zeros>SHORT_GAP && (junk-glitch)<POST_GLITCH_GARBAGE) {
                mark_gap_start(raw,in);
                glitch=0;
            }
        }
        if(raw[in]!=0)
            zeros=0;
    }

    //--- Walk filesystem, mark a block start/end when something looks like a valid file
/*
    in=findFirstBlock(raw)-MIN_GAP_SIZE;
    do {
        if(block_decode(..)) {
            raw[head]=0xff;
            raw[tail]=3;
        }
        next_gap(..);
    } while(..);
*/
    //--- Identify files by CRC. If data looks like it's surrounded by gaps and it has a valid CRC where we
    //    expect one to be, assume it's a file and mark its start/end.

    in=findFirstBlock(raw)+1;
    if(in>0) do {
        out=crc_detect(raw,in,rawSize);
        if(out) {
            printf("crc found %X-%X\n",in,out);
            raw[out]=3;     //mark glitch (gap start)
            //raw[in-1]=0xff;   //mark gap end 
        }
        in=nextGapEnd(raw, out? out: in, rawSize);
    } while(in<rawSize);

    //--- mark gap start/end using glitches to find gap start

    for(glitch=0, zeros=0, in=0; in<rawSize; in++) {
        if(raw[in]==3) {
            glitch=in;
        } else if(raw[in]==1) {
            if(zeros>LONG_GAP && (in-zeros-LONG_POST_GLITCH_GARBAGE)<glitch) {
                mark_gap_start(raw,in);
                raw[in]=0xff;
            }
        } else if(raw[in]==0) {
            zeros++;
        }
        if(raw[in]!=0)
            zeros=0;
    }

    //--- output

    /*
    FILE *f=fopen("raw0-3.bin","wb");
    fwrite(raw,1,rawSize,f);
    fclose(f);
    */

    char bitval=0;
    int lastBlockStart=0;
    for(in=0, out=0; in<rawSize; in++) {
        switch(raw[in]|(bitval<<4)) {
            case 0x11:
                out++;
            case 0x00:
                out++;
                bitval=0;
                break;
            case 0x12:
                out++;
            case 0x01:
            case 0x10:
                bin[out/8] |= 1<<(out&7);
                out++;
                bitval=1;
                break;
            case 0xff:  //block end
                if(lastBlockStart)
                    verify_block(bin, lastBlockStart, reverse);
                bin[out/8] = 0x80;
                out=(out|7)+1;      //byte-align for readability
                lastBlockStart=out/8;
                bitval=1;
                break;
            case 0x02:
                //printf("Encoding error @ %X(%X)\n",in,out/8);
            default: //anything else (glitch)
                out++;
                bitval=0;
                break;
        }
        reverse[out/8]=in;
    }
    //last block
    verify_block(bin, lastBlockStart, reverse);

    *_bin=bin;
    *_binSize=out/8+1;
    free(reverse);
}

bool FDS_rawToBin(char *filename_raw, char *filename_bin) {
    FILE *f;

    uint8_t *rawBuf=NULL;
    int rawSize=0;
    uint8_t *binBuf=NULL;
    int binSize=0;

    f=fopen(filename_raw, "rb");
    if(!f)
        return false;
    fseek(f,0,SEEK_END);
    rawSize=ftell(f);
    fseek(f,0,SEEK_SET);
    rawBuf=(uint8_t*)malloc(rawSize);
    fread(rawBuf,1,rawSize,f);
    fclose(f);

    raw_to_raw03(rawBuf, rawSize);
    raw03_to_bin(rawBuf, rawSize, &binBuf, &binSize);

    if(binSize && binBuf && filename_bin) {
        f=fopen(filename_bin, "wb");
        fwrite(binBuf, 1, binSize, f);
        fclose(f);
        printf("Wrote %s\n", filename_bin);
    }

    if(binBuf)
        free(binBuf);
    free(rawBuf);
    return true;
}


// =========================================

//make raw0-3 from flash image (sans header)
static void bin_to_raw03(uint8_t *bin, uint8_t *raw, int binSize, int rawSize) {
    int in, out;
    uint8_t bit;

    memset(raw,0xff,rawSize);
    for(bit=1, out=0, in=0; in<binSize*8; in++) {
        bit = (bit<<7) | (1 & (bin[in/8]>>(in%8)));   //LSB first
        switch(bit) {
            case 0x00:  //10 10
                raw[++out]++;
                break;
            case 0x01:  //10 01
            case 0x81:  //01 01
                ++raw[out++];
                break;
            case 0x80:  //01 10
                raw[out]+=2;
                break;
        }
    }
    memset(raw+out,3,rawSize-out);  //fill remainder with (undefined)
}

//Going directly to .FDS is messy, flash image isn't byte aligned and has gaps+CRCs.
//Just convert to raw and use disk dumping functions.
bool FDS_readFlashToFDS(char *filename_fds, int slot) {  //slot 1..N
    enum {
        RAWSIZE = SLOTSIZE*8,
    };

    static uint8_t fwnesHdr[16]={0x46, 0x44, 0x53, 0x1a, };

    FILE *f;
    uint8_t *bin, *raw, *fds;
    bool result=true;

    f=fopen(filename_fds, "wb");
    if(!f) {
        printf("Can't create %s\n",filename_fds);
        return false;
    }

    printf("Writing %s\n",filename_fds);
    fwnesHdr[4]=0;
    fwrite(fwnesHdr,1,sizeof(fwnesHdr),f);

    bin=(uint8_t*)malloc(SLOTSIZE);     //single side from flash
    raw=(uint8_t*)malloc(RAWSIZE);      //..to raw03
    fds=(uint8_t*)malloc(FDSSIZE);      //..to FDS

    int side=0;
    for(; side+slot<=dev_slots; side++) {
        if(!spi_readFlash((slot+side-1)*SLOTSIZE, bin, SLOTSIZE)) {
            result=false;
            break;
        }

        if(bin[0]==0xff || (bin[0]!=0 && side!=0)) {    //stop on empty slot or next game
            break;
        } else if(bin[0]==0 && side==0) {
            printf("Warning! Not first side of game\n");
        }

        printf("Side %d\n",side+1);
        memset(bin,0,FLASHHEADERSIZE);  //clear header, use it as lead-in
        bin_to_raw03(bin, raw, SLOTSIZE, RAWSIZE);
        if(!raw03_to_fds(raw, fds, RAWSIZE)) {
            result=false;
            break;
        }
        fwrite(fds,1,FDSSIZE,f);
        fwnesHdr[4]++;  //count sides written
    }

    fseek(f,0,SEEK_SET);
    fwrite(fwnesHdr,1,sizeof(fwnesHdr),f);      //update disk side count

    free(fds);
    free(raw);
    free(bin);
    fclose(f);
    return result;
}