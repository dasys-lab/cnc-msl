GCC = gcc

CFLAGS = -Wall -fPIC -I../include
DEFINES = -Dfreenect_EXPORTS 

SOURCES = core.o tilt.o cameras.o usb_libusb10.o
OBJS = $(SOURCES:.cpp=.o)
LIBS = -lusb-1.0

all: ../../lib/libfreenect.so ../../lib/libfreenect_sync.so

%.o: %.c
	$(GCC) $(CFLAGS) $(DEFINES) -c -o $@ $<

../../lib/libfreenect.so.0.0.1: $(OBJS)
	@mkdir -p ../../lib
	$(GCC) $(CFLAGS) $(LIBS) -shared -Wl,-soname,libfreenect.so.0.0 -o $@ $(OBJS)

../../lib/libfreenect_sync.so.0.0.1: libfreenect_sync.o
	@mkdir -p ../../lib
	$(GCC) $(CFLAGS) $(LIBS) -shared -Wl,-soname,libfreenect_sync.so.0.0 -o $@ $<

../../lib/libfreenect.so: ../../lib/libfreenect.so.0.0
	ln -sf libfreenect.so.0.0 $@

../../lib/libfreenect.so.0.0: ../../lib/libfreenect.so.0.0.1
	ln -sf libfreenect.so.0.0.1 $@

../../lib/libfreenect_sync.so: ../../lib/libfreenect_sync.so.0.0
	ln -sf libfreenect_sync.so.0.0 $@

../../lib/libfreenect_sync.so.0.0: ../../lib/libfreenect_sync.so.0.0.1
	ln -sf libfreenect_sync.so.0.0.1 $@

clean:
	rm -f *.o

distclean: clean

