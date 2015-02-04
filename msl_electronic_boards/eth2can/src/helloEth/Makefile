GCC = gcc
CFLAGS = -O3 -funroll-loops -fomit-frame-pointer -g -Wall -Wunused 
LIBS =


all: udp-client udp-server


udp-client:
	$(GCC) $(CFLAGS) $@.c -o $@

udp-server:
	$(GCC) $(CFLAGS) $@.c -o $@


clean:
	rm -f *.o *~ udp-client udp-server
                        
