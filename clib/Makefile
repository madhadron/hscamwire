# Makefile for Camwire library archive.
# $Id: Makefile,v 8.3 2008/05/19 00:50:18 johanns Exp $

# Camwire source and build base directory:

OBJECTS = camwirebus_1394.lo camwire_1394.lo

SRCS = camwirebus_1394.c \
	camwire_1394.c \
	camwirebus-test.c

BUILDLIBS = -lm -lraw1394 -ldc1394_control

LIBDIR = /usr/local/lib/camwire
INCLUDEDIR = /usr/local/include/camwire

INSTALLFILES = $(INSTALLDIR)/$(INC)/camwirebus.h \
		$(INSTALLDIR)/$(INC)/camwire.h \
		$(INSTALLDIR)/$(LIB)/libcamwire_1394.la

LIBNAME = libcamwire_1394.la

%.lo: %.c
	libtool --mode=compile $(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@

libcamwire_1394.la: $(OBJECTS)
	libtool --mode=link gcc -g -O -o $@ $(OBJECTS) -rpath $(LIBDIR) $(BUILDLIBS)

library: $(LIBNAME)

install: library
	libtool --mode=install install -c $(LIBNAME) $(LIBDIR)/$(LIBNAME)
	install camwire/camwire.h $(INCLUDEDIR)
	install camwire/camwirebus.h $(INCLUDEDIR)

clean:
	libtool --mode=uninstall rm *.lo 
	rm *~
