# Userland probe tools for the Apple USB Modem (05ac:1401).
#
#   make            build all tools
#   make clean      remove binaries and objects
#   make install    install to ${BINDIR} (as root)

PROGS=		modemprobe isotest answerplay
SCRIPTS=	linelog.sh tones.py
SCRIPTSNAME_linelog.sh=	linelog.sh
SCRIPTSNAME_tones.py=	tones.py

BINDIR?=	/usr/local/bin
MK_MAN=		no
MK_DEBUG_FILES=	no

WARNS?=		3
LDADD=		-lusb -lm -lpthread

.include <bsd.progs.mk>
