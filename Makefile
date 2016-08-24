
UNAME := $(shell uname)

CC       ?= gcc
CXX      ?= g++
CFLAGS   ?= -Wall -g -c

TARGET    = fds
CPPOBJS   = main.o spi.o fds.o device.o os.o
ifeq ($(UNAME),Darwin)
 COBJS    = hidapi/hid-mac.o
 LIBS     = -framework IOKit -framework CoreFoundation -liconv
endif
ifeq ($(UNAME),Linux)
 COBJS    = hidapi/hid-linux.o
 LIBS      = `pkg-config libusb-1.0 --libs`
 INCLUDES ?= `pkg-config libusb-1.0 --cflags`
endif
OBJS      = $(COBJS) $(CPPOBJS)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -Wall -g $^ $(LIBS) -o $(TARGET)

$(COBJS): %.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@

$(CPPOBJS): %.o: %.cpp
	$(CXX) $(CFLAGS) $(INCLUDES) $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: clean
