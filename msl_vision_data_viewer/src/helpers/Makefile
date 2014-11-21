include ../../../config/Makefile.inc

GCC = g++
CFLAGS = -O3 -Wall -fPIC -I$(ES_ROOT)/autogen/cpp/ -I$(ES_ROOT)/Spica/Castor/ -I$(ES_ROOT)/Spica/Castor/geminga -I$(ES_ROOT)/Spica/Castor/geminga/autogen/cpp
LIBS = -lpthread -lm -L$(ES_ROOT)/lib -lcastor -lgeminga -lmessages  -lutil -lboost_iostreams


EXECUTABLE = remoteLocPlot
#PCanGonzales \

EXECUTABLE_SRC =  $(EXECUTABLE:=.c)

EXECUTABLE_OBJS = $(EXECUTABLE_SRC:.c=.o)

OBJS = $(SOURCES:.c=.o)
OBJSCPP  = $(SOURCESCPP:.cpp=.o)

SOURCESCPP = SpicaHelper.cpp \
		FootballField.cpp \
		KeyHelper.cpp \
		LocalizeDebug.cpp

all: $(EXECUTABLE) 



$(EXECUTABLE): $(OBJSCPP) $(OBJS) $(EXECUTABLE_OBJS)
	$(GCC) $(LIBS) $(OBJS) $(OBJSCPP) $@.o -o $@
	@cp $@ ../../../bin/
.cpp.o:
	$(GCC) $(CFLAGS) $(DEFINES) -c $< -o $@

.c.o:
	$(GCC) $(CFLAGS) $(DEFINES) -c $< -o $@

clean:
	rm -f *.o *.so



distclean: clean
	rm -f $(EXECUTABLE)

