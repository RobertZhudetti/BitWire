CFLAGS = -Os -DF_CPU=8000000UL -c
SOURCES = $(wildcard *.cpp)
HEADERS = $(wildcard *.h)
LIBDIR = ../..
INCDIR = ../../include
OBJS = $(foreach mcutype, $(MCUTYPES), $(subst .cpp,.$(mcutype).o, $(SOURCES)))

%.atmega32u2.o:%.cpp
	$(CC) $(CFLAGS) -mmcu=atmega32u2 -I$(INCDIR) $< -o $@

%.attiny85.o:%.cpp
	$(CC) $(CFLAGS) -mmcu=attiny85 -I$(INCDIR) $< -o $@

all:$(OBJS)

clean:
	rm -rf *.o *.hex *.bin
