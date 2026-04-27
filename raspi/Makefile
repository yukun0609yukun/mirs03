# Makefile for Raspberry Pi

CC = gcc
CFLAGS = -Wall -O2
LDFLAGS = -lwiringPi -lpthread
SRCDIR = .
BINDIR = bin

SOURCES = arduino.c request.c direction.c io.c uss.c person_mirs.c person_org.c pilot.c pilot_vlt_mirs.c pilot_vlt_org.c test_request.c test_uss.c test_uss_thread.c test_position.c test_person.c test_io.c test_direction.c test_decode.c
OBJECTS = $(patsubst %.c, $(BINDIR)/%.o, $(SOURCES))

all: dirs $(BINDIR)/pilot_vlt_mirs $(BINDIR)/test_request

dirs:
	mkdir -p $(BINDIR)

$(BINDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/pilot_vlt_mirs: $(patsubst %.c, $(BINDIR)/%.o, pilot_vlt_mirs.c arduino.c request.c uss.c io.c person_mirs.c)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(BINDIR)/test_request: $(patsubst %.c, $(BINDIR)/%.o, test_request.c arduino.c request.c uss.c io.c)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf $(BINDIR)

.PHONY: all clean dirs
