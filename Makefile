# SPDX-License-Identifier: GPL-2.0-only

DESTDIR ?=
PREFIX ?= /usr/local

MAKEFLAGS := -k

CFLAGS := -O2 -Wall -Wno-parentheses

CPPFLAGS += \
	-Iinclude \
	-D_FILE_OFFSET_BITS=64 \
	-D_GNU_SOURCE

PROGRAMS := \
	canxlrcv \
	ccfd2xl \
	xl2ccfd

all: $(PROGRAMS)

clean:
	rm -f $(PROGRAMS) *.o

install:
	mkdir -p $(DESTDIR)$(PREFIX)/bin
	cp -f $(PROGRAMS) $(DESTDIR)$(PREFIX)/bin

distclean: clean
	rm -f $(PROGRAMS) *~
