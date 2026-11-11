# Apple USB Modem (05ac:1401) driver for FreeBSD
#
#   make                 build uamodem.ko against /usr/src/sys
#   make SYSDIR=/path    build against another kernel source tree
#   sudo make load       kldload ./uamodem.ko
#   sudo make install    install into /boot/modules

.PATH: ${.CURDIR}

KMOD=	uamodem
SRCS=	uamodem.c
SRCS+=	opt_usb.h opt_bus.h device_if.h bus_if.h usb_if.h usbdevs.h

.include <bsd.kmod.mk>
