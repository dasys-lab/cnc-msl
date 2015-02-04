.SUFFIXES: .o .c

EXEC = cansrvcn

CPC_INCLUDE_PATH = $(ROOTDIR)/linux-2.4.x/drivers/can

OBJS = utils.o udp.o can.o main.o

CFLAGS += -I${CPC_INCLUDE_PATH} -I/home/ucdist2/uClinux-dist/linux-2.4.x/include -ggdb -Wall
LDLIBS += -lz

FLTFLAGS = -s 16384

all: $(EXEC)

$(EXEC): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

efence: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS) -lefence

.c.o:
	$(CC) $(CFLAGS) -c $<

clean:
	-rm -f $(EXEC) *.elf *.gdb *.o

romfs:

test:
	arm-elf-gcc -Os  -Dlinux -D__linux__ -Dunix -D__uClinux__ -DEMBED -I/home/ucdist2/uClinux-dist/lib/uClibc/include -I/home/ucdist2/uClinux-dist/lib/libm -I/home/ucdist2/uClinux-dist/lib/zlib -I/home/ucdist2/uClinux-dist -fno-builtin -nostartfiles -I/home/ucdist2/uClinux-dist/linux-2.4.x/include -DPLATFORM_ETHERCAN -I/home/ucdist2/uClinux-dist/linux-2.4.x/drivers/can -I/home/ucdist2/uClinux-dist/linux-2.4.x/include -ggdb -Wall -c udp.c
