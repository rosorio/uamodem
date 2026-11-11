/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * uamodem - FreeBSD driver for the Apple USB Modem (05ac:1401, bcdDevice 2.02)
 *
 * WHAT THIS HARDWARE ACTUALLY IS
 * ------------------------------
 * Despite being sold as a "USB modem", this device is a *controllerless*
 * (soft) modem: a Motorola SM56 USB bridge in front of a Silicon Labs
 * Si3050/Si3018 DAA.  It contains no data pump and no AT command
 * interpreter.  It is NOT a CDC-ACM device and has no bulk endpoints, so
 * umodem(4) does not (and cannot) attach to it.  What it exposes is:
 *
 *   if 0  CDC Telephone Control Model   SET_HOOK_STATE, notifications on the
 *                                       interrupt IN endpoint (0x81)
 *   if 1  vendor specific               DAA register / LED command channel,
 *                                       carried in CDC SEND_ENCAPSULATED_COMMAND
 *                                       / GET_ENCAPSULATED_RESPONSE;
 *                                       SET/GET_COMM_FEATURE(2) = country
 *   if 2  audio control                 (nothing useful)
 *   if 3  audio stream, alt 1           iso IN  0x82, 8 kHz 16-bit mono (line->host)
 *   if 4  audio stream, alt 1           iso OUT 0x03, 8 kHz 16-bit mono (host->line)
 *   if 5  DFU                           never touched by this driver
 *
 * (Protocol reverse-engineered from the Mac OS X SM56KUSBAudioFamily.kext,
 * see ../NOTES.md.)
 *
 * WHAT THIS DRIVER PROVIDES
 * -------------------------
 * A standard ucom(4) serial port, /dev/cuaU<n> and /dev/ttyU<n>, so that
 * terminal programs (cu, tip, minicom) and dialers can talk Hayes AT
 * commands to it.  Since the hardware has no AT interpreter, the driver
 * implements one in the kernel:
 *
 *   - hook control (ATH, ATA, DTR drop => hang up)
 *   - tone dialling (ATDT) by synthesising DTMF and streaming it on the iso
 *     OUT endpoint, pulse dialling (ATDP) by flashing the hook switch
 *   - dial-tone detection on the iso IN endpoint (ATX4 behaviour)
 *   - RING reporting and auto-answer (ATS0) from CDC RING_DETECT
 *   - country selection (AT+GCI, ITU-T T.35 codes) which programs both the
 *     firmware country feature and the Si3050 line-interface registers
 *   - identification / diagnostics (ATI0..ATI4)
 *
 * What it does NOT provide is V.21..V.92 modulation: that is a full DSP
 * software modem and has no place in a kernel driver.  A dial therefore
 * produces a *voice* connection: the line is seized, the number dialled and
 * the call stays up until ATH / DTR drop / close, but no CONNECT is ever
 * reported.
 *
 * About CDC-ACM SET_LINE_CODING / SET_CONTROL_LINE_STATE: the device does
 * not implement them (there is no UART behind the USB link).  Termios
 * settings are therefore accepted and ignored, and DTR is mapped onto the
 * hook switch the way a real modem with AT&D2 behaves.
 *
 * LOCKING
 * -------
 * One mutex, sc_mtx, protects the whole softc.  It is also the ucom/tty
 * mutex and the mutex of all USB transfers, so USB callbacks and ucom
 * callbacks run with it held.  All blocking work (control requests, dial
 * timing) is done by a per-device kernel thread, the "worker", which drops
 * the mutex only inside msleep() and usbd_do_request_flags().
 */

#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/priv.h>
#include <sys/kthread.h>
#include <sys/proc.h>
#include <sys/serial.h>
#include <sys/ctype.h>
#include <machine/stdarg.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usb_cdc.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>

#define	USB_DEBUG_VAR uamodem_debug
#include <dev/usb/usb_debug.h>

#include <dev/usb/serial/usb_serial.h>

/* Q15 sine table (defined below), also used by the Caller ID decoder. */
static const int16_t uam_sine[256];
#include "uam_cid.h"

#define	UAMODEM_MODVER		1
#define	UAMODEM_VERSION		"1.0"

/* ------------------------------------------------------------------ */
/* Device constants                                                   */
/* ------------------------------------------------------------------ */

#define	UAM_VENDOR		0x05ac	/* Apple */
#define	UAM_PRODUCT		0x1401	/* Apple USB Modem */

/* CDC requests (class, interface recipient) */
#define	UAM_SEND_ENCAPSULATED	0x00
#define	UAM_GET_ENCAPSULATED	0x01
#define	UAM_SET_COMM_FEATURE	0x02
#define	UAM_GET_COMM_FEATURE	0x03
#define	UAM_SET_HOOK_STATE	0x11
#define	UAM_FEATURE_COUNTRY	2

/* CDC notifications seen on the interrupt endpoint */
#define	UAM_NOTIFY_RESPONSE	0x01	/* standard RESPONSE_AVAILABLE */
#define	UAM_NOTIFY_RESPONSE2	0x02	/* what this firmware actually sends */
#define	UAM_NOTIFY_JACK_HOOK	0x08
#define	UAM_NOTIFY_RING		0x09
#define	UAM_NOTIFY_LINE_STATE	0x29

/* LINE_STATE_CHANGE values, decoded from the Apple kext */
#define	UAM_LINE_UNPLUGGED	4
#define	UAM_LINE_PLUGGED	5

/* Encapsulated command opcodes ([op, a, b], 3 bytes) */
#define	UAM_OP_WRITE_REG	0x00	/* a = reg, b = value */
#define	UAM_OP_OR_REG		0x01	/* a = reg, b = bits to set */
#define	UAM_OP_AND_REG		0x02	/* a = reg, b = mask to keep */
#define	UAM_OP_READ_REG		0x80	/* a = reg, 1-byte reply */
#define	UAM_OP_FLUSH		0x30
#define	UAM_OP_FW_VERSION	0xd0	/* 2-byte reply */

/* Si3050 DAA registers */
#define	DAA_DCTL1		5	/* control 1 */
#define	DAA_DCTL1_OH		0x01	/* off-hook */
#define	DAA_DCTL1_ONHM		0x08	/* on-hook line monitor */
#define	DAA_LCS			12	/* loop current status (low 5 bits) */
#define	DAA_ICTL1		16	/* OHS, RZ, RT */
#define	DAA_DCTL		26	/* DCV, MINI, ILIM */
#define	DAA_LCS2		28	/* loop current, 1.1 mA/bit */
#define	DAA_LVS			29	/* line voltage, signed, 1 V/bit */
#define	DAA_ACIM		30
#define	DAA_ICTL4		31	/* OHS2 et al. */

/* Audio */
#define	UAM_RATE		8000	/* samples per second */
#define	UAM_FRAME_SAMPLES	(UAM_RATE / 1000)	/* per 1 ms USB frame */
#define	UAM_FRAME_BYTES		(UAM_FRAME_SAMPLES * 2)
#define	UAM_ISO_FRAMES		8	/* frames per transfer: 8 ms */
#define	UAM_UAC_SET_CUR		0x01
#define	UAM_UAC_SAMPLING_FREQ	0x0100	/* wValue: control selector << 8 */

#define	UAM_DTMF_AMPLITUDE	7000	/* per tone, ~ -10 dBm0 */
#define	UAM_MIN_OFFHOOK_MS	500	/* see uam_hook_raw() */

/* ------------------------------------------------------------------ */
/* Country table (AT+GCI)                                             */
/* ------------------------------------------------------------------ */

/*
 * t35:  ITU-T T.35 country code (what AT+GCI takes)
 * iso:  ISO 3166 numeric written as hex digits, e.g. France 250 -> 0x0250
 *       (format of the firmware's COUNTRY_SETTING comm feature)
 * Remaining fields are the Si3050 line-interface parameters, taken from
 * the DAHDI fxo_modes table, and are folded into registers 16/26/30/31
 * exactly the way the DAHDI wctdm driver does it.
 */
struct uam_country {
	uint8_t		t35;
	uint16_t	iso;
	const char	*name;
	uint8_t		ohs, rz, rt, ilim, dcv, mini, acim, ohs2;
};

static const struct uam_country uam_countries[] = {
	/* t35   iso     name           ohs rz rt ilim dcv mini acim ohs2 */
	{ 0x00, 0x0392, "Japan",	 0, 0, 0, 0,  0,  3,   0,   0 },
	{ 0x04, 0x0276, "Germany",	 0, 0, 0, 1,  3,  0,   3,   1 },
	{ 0x09, 0x0036, "Australia",	 1, 0, 0, 0,  0,  3,   3,   0 },
	{ 0x0a, 0x0040, "Austria",	 0, 0, 0, 1,  3,  0,   3,   1 },
	{ 0x0f, 0x0056, "Belgium",	 0, 0, 0, 1,  3,  0,   2,   1 },
	{ 0x20, 0x0124, "Canada",	 0, 0, 0, 0,  3,  0,   0,   0 },
	{ 0x3d, 0x0250, "France",	 0, 0, 0, 1,  3,  0,   2,   1 },
	{ 0x59, 0x0380, "Italy",	 0, 0, 0, 1,  3,  0,   2,   1 },
	{ 0x7b, 0x0528, "Netherlands",	 0, 0, 0, 1,  3,  0,   2,   1 },
	{ 0x8b, 0x0620, "Portugal",	 0, 0, 0, 1,  3,  0,   2,   1 },
	{ 0xa0, 0x0724, "Spain",	 0, 0, 0, 1,  3,  0,   2,   1 },
	{ 0xa5, 0x0756, "Switzerland",	 0, 0, 0, 1,  3,  0,   2,   1 },
	{ 0xb4, 0x0826, "United Kingdom", 0, 0, 0, 1, 3,  0,   5,   1 },
	{ 0xb5, 0x0840, "United States", 0, 0, 1, 0,  3,  0,   0,   0 },
};

#define	UAM_NCOUNTRIES	nitems(uam_countries)
#define	UAM_DEFAULT_COUNTRY_T35	0xb5

/* ------------------------------------------------------------------ */
/* Tunables / debug                                                   */
/* ------------------------------------------------------------------ */

static SYSCTL_NODE(_hw_usb, OID_AUTO, uamodem, CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
    "Apple USB Modem");

#ifdef USB_DEBUG
static int uamodem_debug = 0;
SYSCTL_INT(_hw_usb_uamodem, OID_AUTO, debug, CTLFLAG_RWTUN,
    &uamodem_debug, 0, "Debug level");
#endif

/* Claim the audio interfaces (needed for tone dialling + dial tone). */
static int uamodem_claim_audio = 1;
SYSCTL_INT(_hw_usb_uamodem, OID_AUTO, audio, CTLFLAG_RDTUN,
    &uamodem_claim_audio, 0,
    "Take the modem's audio interfaces away from uaudio(4)");

/* T.35 country applied at attach; -1 keeps what the firmware has. */
static int uamodem_country = -1;
SYSCTL_INT(_hw_usb_uamodem, OID_AUTO, country, CTLFLAG_RDTUN,
    &uamodem_country, 0, "T.35 country code applied at attach (-1 = keep)");

/* ------------------------------------------------------------------ */
/* Softc                                                              */
/* ------------------------------------------------------------------ */

enum {
	UAM_XFER_INTR,
	UAM_XFER_PLAY0,
	UAM_XFER_PLAY1,
	UAM_XFER_REC0,
	UAM_XFER_REC1,
	UAM_N_XFER,
};

/* indexes into sc_iface_index[] used by the transfer config below */
#define	UAM_IFX_COMM	0
#define	UAM_IFX_PLAY	1
#define	UAM_IFX_REC	2

/* sc_flags */
#define	UAM_F_GONE	0x0001	/* detaching: worker must exit */
#define	UAM_F_THREAD	0x0002	/* worker thread is running */
#define	UAM_F_OPEN	0x0004	/* tty is open */
#define	UAM_F_TXPEND	0x0008	/* tty has output for us (AT input) */
#define	UAM_F_HANGUP	0x0010	/* worker must go on-hook */
#define	UAM_F_AUDIO	0x0020	/* iso audio transfers are set up */
#define	UAM_F_RESP	0x0040	/* encapsulated response available */
#define	UAM_F_DTR	0x0080	/* DTR currently asserted */

/* result codes, index = numeric (ATV0) form */
enum {
	UAM_R_OK = 0, UAM_R_CONNECT = 1, UAM_R_RING = 2, UAM_R_NOCARRIER = 3,
	UAM_R_ERROR = 4, UAM_R_NODIALTONE = 6, UAM_R_BUSY = 7,
	UAM_R_NOANSWER = 8, UAM_R_NONE = 255,
};

static const char *const uam_result_text[] = {
	[UAM_R_OK] = "OK",
	[UAM_R_CONNECT] = "CONNECT",
	[UAM_R_RING] = "RING",
	[UAM_R_NOCARRIER] = "NO CARRIER",
	[UAM_R_ERROR] = "ERROR",
	[5] = "",
	[UAM_R_NODIALTONE] = "NO DIALTONE",
	[UAM_R_BUSY] = "BUSY",
	[UAM_R_NOANSWER] = "NO ANSWER",
};

#define	UAM_NSREG	32
#define	UAM_LINE_MAX	128

struct uamodem_softc {
	struct ucom_super_softc	sc_super_ucom;
	struct ucom_softc	sc_ucom;

	device_t		sc_dev;
	struct usb_device	*sc_udev;
	struct usb_xfer		*sc_xfer[UAM_N_XFER];
	struct mtx		sc_mtx;
	struct thread		*sc_td;		/* worker */

	uint8_t			sc_iface_index[3];
	uint8_t			sc_comm_iface_no;	/* CDC TCM, hook */
	uint8_t			sc_cmd_iface_no;	/* vendor, DAA channel */
	uint8_t			sc_play_ep;
	uint8_t			sc_rec_ep;

	int			sc_flags;
	int			sc_wchan;	/* worker sleeps on this */

	/* line / hook state */
	bool			sc_offhook;
	int			sc_offhook_ticks;	/* when we went off-hook */
	int			sc_line_state;	/* last LINE_STATE_CHANGE */
	int			sc_ring_events;	/* RINGs not yet reported */
	int			sc_last_ring;	/* ticks of last RING */
	int			sc_last_ring_notify;	/* ticks of last RING_DETECT */
	uint8_t			sc_resp[8];	/* encapsulated response */
	uint16_t		sc_fw_version;
	const struct uam_country *sc_country;

	/* DTMF generator (consumed by the play callback) */
	uint32_t		sc_tone_ph[2];
	uint32_t		sc_tone_step[2];
	uint32_t		sc_tone_left;	/* samples still to play */
	uint64_t		sc_tone_total;	/* DTMF samples generated */

	/* Caller ID (AT+VCID) */
	uint8_t			sc_vcid;	/* 0 off, 1 formatted, 2 raw */
	bool			sc_cid_ready;	/* sc_cid.msg holds a message */
	struct uam_cid		sc_cid;		/* decoder (record callback) */
	uint64_t		sc_rec_seen;	/* sc_rec_total at last check */
	int			sc_rec_restarts;
	int			sc_rec_check;	/* ticks of last watchdog check */
	int			sc_cid_msgs;
	char			sc_cid_last[2 * UAM_CID_MAXMSG + 1];

	/* capture statistics (filled by the record callback) */
	int64_t			sc_rec_sum;
	uint64_t		sc_rec_sumsq;
	uint32_t		sc_rec_n;
	int			sc_dialtone_rms;	/* threshold, sysctl */
	int			sc_last_rms;
	int			sc_last_dc;
	uint64_t		sc_rec_total;

	/* AT interpreter state */
	char			sc_line[UAM_LINE_MAX];
	int			sc_linelen;
	char			sc_lastcmd[UAM_LINE_MAX];
	bool			sc_echo;	/* E */
	bool			sc_quiet;	/* Q */
	bool			sc_verbose;	/* V */
	bool			sc_info_started;	/* info text sent for this line */
	uint8_t			sc_xlevel;	/* X */
	uint8_t			sc_dtr_mode;	/* &D */
	uint8_t			sc_sreg[UAM_NSREG];
	char			sc_prevchar;
};

/* ------------------------------------------------------------------ */
/* Prototypes                                                         */
/* ------------------------------------------------------------------ */

static device_probe_t uamodem_probe;
static device_attach_t uamodem_attach;
static device_detach_t uamodem_detach;
static void uamodem_free_softc(struct uamodem_softc *);

static usb_callback_t uamodem_intr_callback;
static usb_callback_t uamodem_play_callback;
static usb_callback_t uamodem_rec_callback;

static void uamodem_cfg_get_status(struct ucom_softc *, uint8_t *, uint8_t *);
static void uamodem_cfg_set_dtr(struct ucom_softc *, uint8_t);
static void uamodem_cfg_set_rts(struct ucom_softc *, uint8_t);
static void uamodem_cfg_open(struct ucom_softc *);
static void uamodem_cfg_close(struct ucom_softc *);
static int uamodem_pre_param(struct ucom_softc *, struct termios *);
static void uamodem_start_read(struct ucom_softc *);
static void uamodem_stop_read(struct ucom_softc *);
static void uamodem_start_write(struct ucom_softc *);
static void uamodem_stop_write(struct ucom_softc *);
static void uamodem_poll(struct ucom_softc *);
static void uamodem_free(struct ucom_softc *);

static void uamodem_worker(void *);

/* ------------------------------------------------------------------ */
/* Transfer configuration                                             */
/* ------------------------------------------------------------------ */

static const struct usb_config uamodem_config[UAM_N_XFER] = {
	[UAM_XFER_INTR] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.if_index = UAM_IFX_COMM,
		.flags = {.pipe_bof = 1, .short_xfer_ok = 1, },
		.bufsize = 0,	/* use wMaxPacketSize */
		.callback = &uamodem_intr_callback,
	},
	[UAM_XFER_PLAY0] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.if_index = UAM_IFX_PLAY,
		.frames = UAM_ISO_FRAMES,
		.bufsize = 0,	/* frames * wMaxPacketSize */
		.flags = {.short_xfer_ok = 1, },
		.callback = &uamodem_play_callback,
	},
	[UAM_XFER_PLAY1] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.if_index = UAM_IFX_PLAY,
		.frames = UAM_ISO_FRAMES,
		.bufsize = 0,
		.flags = {.short_xfer_ok = 1, },
		.callback = &uamodem_play_callback,
	},
	[UAM_XFER_REC0] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.if_index = UAM_IFX_REC,
		.frames = UAM_ISO_FRAMES,
		.bufsize = 0,
		.flags = {.short_xfer_ok = 1, },
		.callback = &uamodem_rec_callback,
	},
	[UAM_XFER_REC1] = {
		.type = UE_ISOCHRONOUS,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.if_index = UAM_IFX_REC,
		.frames = UAM_ISO_FRAMES,
		.bufsize = 0,
		.flags = {.short_xfer_ok = 1, },
		.callback = &uamodem_rec_callback,
	},
};

static const struct ucom_callback uamodem_callback = {
	.ucom_cfg_get_status = &uamodem_cfg_get_status,
	.ucom_cfg_set_dtr = &uamodem_cfg_set_dtr,
	.ucom_cfg_set_rts = &uamodem_cfg_set_rts,
	.ucom_cfg_open = &uamodem_cfg_open,
	.ucom_cfg_close = &uamodem_cfg_close,
	.ucom_pre_param = &uamodem_pre_param,
	.ucom_start_read = &uamodem_start_read,
	.ucom_stop_read = &uamodem_stop_read,
	.ucom_start_write = &uamodem_start_write,
	.ucom_stop_write = &uamodem_stop_write,
	.ucom_poll = &uamodem_poll,
	.ucom_free = &uamodem_free,
};

static const STRUCT_USB_HOST_ID uamodem_devs[] = {
	{USB_VPI(UAM_VENDOR, UAM_PRODUCT, 0)},
};

/* ------------------------------------------------------------------ */
/* Small helpers                                                      */
/* ------------------------------------------------------------------ */

/* 256-entry Q15 sine table for the DTMF oscillators. */
static const int16_t uam_sine[256] = {
	     0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
	  6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
	 12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
	 18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
	 23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
	 27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
	 30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
	 32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
	 32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
	 32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
	 30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
	 27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
	 23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
	 18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
	 12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
	  6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
	     0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
	 -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
	-12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
	-18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
	-23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
	-27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
	-30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
	-32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
	-32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
	-32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
	-30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
	-27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
	-23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
	-18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
	-12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
	 -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804,
};

static int
uam_ms_to_ticks(int ms)
{
	int t;

	t = (int)(((int64_t)ms * hz + 999) / 1000);
	return (t > 0 ? t : 1);
}

static uint32_t
uam_isqrt(uint64_t v)
{
	uint64_t r = 0, bit = (uint64_t)1 << 62;

	while (bit > v)
		bit >>= 2;
	while (bit != 0) {
		if (v >= r + bit) {
			v -= r + bit;
			r = (r >> 1) + bit;
		} else
			r >>= 1;
		bit >>= 2;
	}
	return ((uint32_t)r);
}

static const struct uam_country *
uam_find_country(int t35)
{
	u_int i;

	for (i = 0; i < UAM_NCOUNTRIES; i++)
		if (uam_countries[i].t35 == t35)
			return (&uam_countries[i]);
	return (NULL);
}

static const struct uam_country *
uam_find_country_iso(uint16_t iso)
{
	u_int i;

	for (i = 0; i < UAM_NCOUNTRIES; i++)
		if (uam_countries[i].iso == iso)
			return (&uam_countries[i]);
	return (NULL);
}

/* ------------------------------------------------------------------ */
/* Probe / attach / detach                                            */
/* ------------------------------------------------------------------ */

static int
uamodem_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);
	/* Bind to the CDC (telephone control) interface only. */
	if (uaa->info.bInterfaceClass != UICLASS_CDC)
		return (ENXIO);
	return (usbd_lookup_id_by_uaa(uamodem_devs, sizeof(uamodem_devs), uaa));
}

/*
 * Find, in the configuration descriptor, the first alternate setting of
 * interface "ifno" that has an isochronous endpoint.  Returns the
 * endpoint address (0 if none) and the alternate *index* to select.
 */
static uint8_t
uam_find_iso_alt(struct usb_device *udev, uint8_t ifno, uint8_t *alt_index)
{
	struct usb_config_descriptor *cd;
	struct usb_descriptor *desc = NULL;
	struct usb_interface_descriptor *id = NULL;
	struct usb_endpoint_descriptor *ed;
	int alt = -1;

	cd = usbd_get_config_descriptor(udev);
	while ((desc = usb_desc_foreach(cd, desc)) != NULL) {
		if (desc->bDescriptorType == UDESC_INTERFACE &&
		    desc->bLength >= sizeof(*id)) {
			id = (struct usb_interface_descriptor *)desc;
			if (id->bInterfaceNumber == ifno)
				alt++;
			continue;
		}
		if (desc->bDescriptorType != UDESC_ENDPOINT ||
		    desc->bLength < sizeof(*ed) || id == NULL ||
		    id->bInterfaceNumber != ifno)
			continue;
		ed = (struct usb_endpoint_descriptor *)desc;
		if (UE_GET_XFERTYPE(ed->bmAttributes) == UE_ISOCHRONOUS) {
			*alt_index = (uint8_t)alt;
			return (ed->bEndpointAddress);
		}
	}
	return (0);
}

/* Is some other driver (normally uaudio) attached to this interface? */
static bool
uam_iface_busy(struct usb_device *udev, uint8_t iface_index)
{
	struct usb_interface *iface = usbd_get_iface(udev, iface_index);

	return (iface != NULL && iface->subdev != NULL &&
	    device_is_attached(iface->subdev));
}

/* UAC1 SET_CUR(SAMPLING_FREQ) on an iso endpoint. */
static usb_error_t
uam_set_rate(struct uamodem_softc *sc, uint8_t ep)
{
	struct usb_device_request req;
	uint8_t data[3];

	req.bmRequestType = UT_WRITE_CLASS_ENDPOINT;
	req.bRequest = UAM_UAC_SET_CUR;
	USETW(req.wValue, UAM_UAC_SAMPLING_FREQ);
	USETW(req.wIndex, ep);
	USETW(req.wLength, 3);
	data[0] = UAM_RATE & 0xff;
	data[1] = (UAM_RATE >> 8) & 0xff;
	data[2] = (UAM_RATE >> 16) & 0xff;
	return (usbd_do_request_flags(sc->sc_udev, NULL, &req, data, 0, NULL,
	    1000));
}

/*
 * Locate the vendor command interface and the two audio streaming
 * interfaces, claim them so that no other driver attaches, select their
 * streaming alternate setting and program the sample rate.
 * Returns true if audio streaming is usable.
 */
static bool
uam_setup_interfaces(struct uamodem_softc *sc, struct usb_attach_arg *uaa)
{
	struct usb_interface *iface;
	struct usb_interface_descriptor *id;
	int ac_index = -1, play_index = -1, rec_index = -1;
	uint8_t play_alt = 0, rec_alt = 0, ep, alt, i;

	sc->sc_cmd_iface_no = 0xff;
	for (i = 0; (iface = usbd_get_iface(sc->sc_udev, i)) != NULL; i++) {
		id = usbd_get_interface_descriptor(iface);
		if (id == NULL || i == uaa->info.bIfaceIndex)
			continue;
		if (id->bInterfaceClass == UICLASS_VENDOR &&
		    sc->sc_cmd_iface_no == 0xff) {
			/* DAA command channel: claim it, no endpoints used */
			sc->sc_cmd_iface_no = id->bInterfaceNumber;
			usbd_set_parent_iface(sc->sc_udev, i,
			    uaa->info.bIfaceIndex);
		} else if (id->bInterfaceClass == UICLASS_AUDIO &&
		    id->bInterfaceSubClass == UISUBCLASS_AUDIOCONTROL) {
			ac_index = i;
		} else if (id->bInterfaceClass == UICLASS_AUDIO &&
		    id->bInterfaceSubClass == UISUBCLASS_AUDIOSTREAM) {
			ep = uam_find_iso_alt(sc->sc_udev,
			    id->bInterfaceNumber, &alt);
			if (ep == 0)
				continue;
			if (UE_GET_DIR(ep) == UE_DIR_IN && rec_index < 0) {
				rec_index = i;
				rec_alt = alt;
				sc->sc_rec_ep = ep;
			} else if (UE_GET_DIR(ep) == UE_DIR_OUT &&
			    play_index < 0) {
				play_index = i;
				play_alt = alt;
				sc->sc_play_ep = ep;
			}
		}
		/* Anything else (the DFU interface) is left alone. */
	}

	if (sc->sc_cmd_iface_no == 0xff) {
		/* Not fatal: we only lose DAA tuning and diagnostics. */
		device_printf(sc->sc_dev, "no vendor command interface\n");
	}

	if (!uamodem_claim_audio) {
		device_printf(sc->sc_dev, "audio disabled by "
		    "hw.usb.uamodem.audio=0: pulse dialling only\n");
		return (false);
	}
	if (play_index < 0 || rec_index < 0) {
		device_printf(sc->sc_dev, "audio streaming interfaces "
		    "not found: pulse dialling only\n");
		return (false);
	}
	if ((ac_index >= 0 && uam_iface_busy(sc->sc_udev, ac_index)) ||
	    uam_iface_busy(sc->sc_udev, play_index) ||
	    uam_iface_busy(sc->sc_udev, rec_index)) {
		device_printf(sc->sc_dev, "audio interfaces already owned by "
		    "another driver (uaudio?); replug the modem to let "
		    "uamodem claim them. Pulse dialling only.\n");
		return (false);
	}

	/* Claim the audio function as a whole so uaudio stays away. */
	if (ac_index >= 0)
		usbd_set_parent_iface(sc->sc_udev, ac_index,
		    uaa->info.bIfaceIndex);
	usbd_set_parent_iface(sc->sc_udev, play_index, uaa->info.bIfaceIndex);
	usbd_set_parent_iface(sc->sc_udev, rec_index, uaa->info.bIfaceIndex);

	if (usbd_set_alt_interface_index(sc->sc_udev, play_index, play_alt) ||
	    usbd_set_alt_interface_index(sc->sc_udev, rec_index, rec_alt)) {
		device_printf(sc->sc_dev, "cannot select audio alternate "
		    "settings: pulse dialling only\n");
		return (false);
	}
	/* Failure here is not fatal; 8 kHz is the power-on default. */
	(void)uam_set_rate(sc, sc->sc_play_ep);
	(void)uam_set_rate(sc, sc->sc_rec_ep);

	sc->sc_iface_index[UAM_IFX_PLAY] = play_index;
	sc->sc_iface_index[UAM_IFX_REC] = rec_index;
	return (true);
}

static int
uamodem_sysctl_cid_bytes(SYSCTL_HANDLER_ARGS)
{
	struct uamodem_softc *sc = arg1;
	u_int val;

	mtx_lock(&sc->sc_mtx);
	val = sc->sc_cid.d[0].bytes + sc->sc_cid.d[1].bytes;
	mtx_unlock(&sc->sc_mtx);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

static int
uamodem_sysctl_country(SYSCTL_HANDLER_ARGS)
{
	struct uamodem_softc *sc = arg1;
	int val;

	mtx_lock(&sc->sc_mtx);
	val = sc->sc_country != NULL ? sc->sc_country->t35 : -1;
	mtx_unlock(&sc->sc_mtx);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

static void
uam_add_sysctls(struct uamodem_softc *sc)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->sc_dev);
	struct sysctl_oid_list *kids =
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->sc_dev));

	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "dialtone_threshold",
	    CTLFLAG_RW, &sc->sc_dialtone_rms, 0,
	    "AC RMS level (16-bit PCM units) that counts as dial tone");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "last_rms", CTLFLAG_RD,
	    &sc->sc_last_rms, 0, "AC RMS of the last 100 ms of line audio");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "last_dc", CTLFLAG_RD,
	    &sc->sc_last_dc, 0, "DC level of the last 100 ms of line audio");
	SYSCTL_ADD_U64(ctx, kids, OID_AUTO, "rec_samples", CTLFLAG_RD,
	    &sc->sc_rec_total, 0, "Line audio samples received");
	SYSCTL_ADD_U64(ctx, kids, OID_AUTO, "tone_samples", CTLFLAG_RD,
	    &sc->sc_tone_total, 0, "DTMF samples sent to the line");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "rec_restarts", CTLFLAG_RD,
	    &sc->sc_rec_restarts, 0, "Capture stream restarts by the watchdog");
	SYSCTL_ADD_U8(ctx, kids, OID_AUTO, "vcid", CTLFLAG_RD,
	    &sc->sc_vcid, 0, "Caller ID mode (AT+VCID)");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "cid_msgs", CTLFLAG_RD,
	    &sc->sc_cid_msgs, 0, "Caller ID messages decoded");
	SYSCTL_ADD_STRING(ctx, kids, OID_AUTO, "cid_last", CTLFLAG_RD,
	    sc->sc_cid_last, 0, "Last Caller ID message (hex)");
	SYSCTL_ADD_PROC(ctx, kids, OID_AUTO, "cid_bytes",
	    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    uamodem_sysctl_cid_bytes, "IU", "FSK bytes received (V.23, Bell 202)");
	SYSCTL_ADD_INT(ctx, kids, OID_AUTO, "line_state", CTLFLAG_RD,
	    &sc->sc_line_state, 0,
	    "Last LINE_STATE_CHANGE value (4 = RJ11 unplugged, 5 = plugged)");
	SYSCTL_ADD_BOOL(ctx, kids, OID_AUTO, "offhook", CTLFLAG_RD,
	    &sc->sc_offhook, 0, "Line is seized");
	SYSCTL_ADD_U16(ctx, kids, OID_AUTO, "firmware", CTLFLAG_RD,
	    &sc->sc_fw_version, 0, "Firmware version");
	SYSCTL_ADD_PROC(ctx, kids, OID_AUTO, "country",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    uamodem_sysctl_country, "I", "Current T.35 country code");
}

static int
uamodem_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uamodem_softc *sc = device_get_softc(dev);
	uint8_t nxfer;
	int error;

	device_set_usb_desc(dev);
	mtx_init(&sc->sc_mtx, "uamodem", NULL, MTX_DEF);
	ucom_ref(&sc->sc_super_ucom);

	sc->sc_dev = dev;
	sc->sc_udev = uaa->device;
	sc->sc_comm_iface_no = uaa->info.bIfaceNum;
	sc->sc_iface_index[UAM_IFX_COMM] = uaa->info.bIfaceIndex;
	sc->sc_dialtone_rms = 150;
	sc->sc_line_state = -1;

	/* Step 1: descriptor parsing / interface claiming. */
	if (uam_setup_interfaces(sc, uaa))
		nxfer = UAM_N_XFER;
	else
		nxfer = UAM_XFER_INTR + 1;	/* interrupt endpoint only */

	/* Step 2: transfers. */
	error = usbd_transfer_setup(uaa->device, sc->sc_iface_index,
	    sc->sc_xfer, uamodem_config, nxfer, sc, &sc->sc_mtx);
	if (error != 0 && nxfer != UAM_XFER_INTR + 1) {
		device_printf(dev, "cannot set up audio transfers (%s): "
		    "pulse dialling only\n", usbd_errstr(error));
		nxfer = UAM_XFER_INTR + 1;
		error = usbd_transfer_setup(uaa->device, sc->sc_iface_index,
		    sc->sc_xfer, uamodem_config, nxfer, sc, &sc->sc_mtx);
	}
	if (error != 0) {
		device_printf(dev, "cannot set up transfers: %s\n",
		    usbd_errstr(error));
		goto detach;
	}
	if (nxfer == UAM_N_XFER)
		sc->sc_flags |= UAM_F_AUDIO;

	/* Step 3: worker thread (owns the AT interpreter and the line). */
	sc->sc_flags |= UAM_F_THREAD;
	error = kthread_add(uamodem_worker, sc, NULL, &sc->sc_td, 0, 0,
	    "uamodem%d", device_get_unit(dev));
	if (error != 0) {
		sc->sc_flags &= ~UAM_F_THREAD;
		device_printf(dev, "cannot create worker thread: %d\n", error);
		goto detach;
	}

	/* Step 4: the serial port. */
	error = ucom_attach(&sc->sc_super_ucom, &sc->sc_ucom, 1, sc,
	    &uamodem_callback, &sc->sc_mtx);
	if (error != 0) {
		device_printf(dev, "cannot attach ucom: %d\n", error);
		goto detach;
	}
	ucom_set_pnpinfo_usb(&sc->sc_super_ucom, dev);

	uam_add_sysctls(sc);

	/* Start listening for RING / line state notifications. */
	mtx_lock(&sc->sc_mtx);
	usbd_transfer_start(sc->sc_xfer[UAM_XFER_INTR]);
	mtx_unlock(&sc->sc_mtx);

	return (0);

detach:
	uamodem_detach(dev);
	return (ENXIO);
}

static int
uamodem_detach(device_t dev)
{
	struct uamodem_softc *sc = device_get_softc(dev);

	/*
	 * Stop the worker first: it calls into ucom (tty input/output),
	 * so it must be gone before ucom_detach() tears the tty down.  It
	 * puts the line on-hook on its way out.
	 */
	mtx_lock(&sc->sc_mtx);
	sc->sc_flags |= UAM_F_GONE;
	wakeup(&sc->sc_wchan);
	wakeup(&sc->sc_tone_left);
	wakeup(&sc->sc_resp);
	while (sc->sc_flags & UAM_F_THREAD) {
		/* kthread_exit() does wakeup(td) after the flag is cleared */
		msleep(sc->sc_td, &sc->sc_mtx, 0, "uamdet", 0);
	}
	mtx_unlock(&sc->sc_mtx);

	ucom_detach(&sc->sc_super_ucom, &sc->sc_ucom);
	usbd_transfer_unsetup(sc->sc_xfer, UAM_N_XFER);

	device_claim_softc(dev);
	uamodem_free_softc(sc);
	return (0);
}

/*
 * The softc is shared with ucom; it is freed when the last reference
 * (ours, or the tty's if a process still has the port open) goes away.
 */
static void
uamodem_free_softc(struct uamodem_softc *sc)
{
	if (ucom_unref(&sc->sc_super_ucom)) {
		mtx_destroy(&sc->sc_mtx);
		device_free_softc(sc);
	}
}

static void
uamodem_free(struct ucom_softc *ucom)
{
	uamodem_free_softc(ucom->sc_parent);
}

/* ------------------------------------------------------------------ */
/* USB transfer callbacks (called with sc_mtx held)                   */
/* ------------------------------------------------------------------ */

/*
 * CDC notifications: 8-byte header (bmRequestType 0xa1, bNotification,
 * wValue, wIndex, wLength) + optional data.
 */
static void
uamodem_intr_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct uamodem_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;
	uint8_t buf[16];
	int actlen;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		if (actlen < 2)
			goto tr_setup;
		if (actlen > (int)sizeof(buf))
			actlen = sizeof(buf);
		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, buf, actlen);
		DPRINTF("notify %02x %02x %02x\n", buf[0], buf[1],
		    actlen > 2 ? buf[2] : 0);

		switch (buf[1]) {
		case UAM_NOTIFY_RESPONSE:
		case UAM_NOTIFY_RESPONSE2:
			sc->sc_flags |= UAM_F_RESP;
			wakeup(&sc->sc_resp);
			break;
		case UAM_NOTIFY_RING:
			/*
			 * The firmware sends RING_DETECT for every cycle of
			 * the ringing voltage (and for mains hum on a bad
			 * line), tens per second.  Report one RING per ring
			 * burst: only after 1 s of silence.
			 */
			if ((u_int)(ticks - sc->sc_last_ring_notify) > (u_int)hz) {
				sc->sc_ring_events++;
				wakeup(&sc->sc_wchan);
			}
			sc->sc_last_ring_notify = ticks;
			break;
		case UAM_NOTIFY_LINE_STATE:
			if (actlen > 2) {
				sc->sc_line_state = buf[2];
				wakeup(&sc->sc_wchan);
			}
			break;
		default:
			break;
		}
		/* FALLTHROUGH */
	case USB_ST_SETUP:
tr_setup:
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);
		break;

	default:
		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		break;
	}
}

/* Host -> line: DTMF generator, silence when idle. */
static void
uamodem_play_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct uamodem_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;
	int16_t buf[UAM_ISO_FRAMES * UAM_FRAME_SAMPLES];
	int i, n, s;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
	case USB_ST_SETUP:
tr_setup:
		n = UAM_ISO_FRAMES * UAM_FRAME_SAMPLES;
		for (i = 0; i < n; i++) {
			if (sc->sc_tone_left == 0) {
				buf[i] = 0;
				continue;
			}
			s = uam_sine[sc->sc_tone_ph[0] >> 24] +
			    uam_sine[sc->sc_tone_ph[1] >> 24];
			buf[i] = htole16((int16_t)((s * UAM_DTMF_AMPLITUDE) >>
			    15));
			sc->sc_tone_ph[0] += sc->sc_tone_step[0];
			sc->sc_tone_ph[1] += sc->sc_tone_step[1];
			sc->sc_tone_total++;
			if (--sc->sc_tone_left == 0)
				wakeup(&sc->sc_tone_left);
		}
		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_in(pc, 0, buf, sizeof(buf));
		for (i = 0; i < UAM_ISO_FRAMES; i++)
			usbd_xfer_set_frame_len(xfer, i, UAM_FRAME_BYTES);
		usbd_xfer_set_frames(xfer, UAM_ISO_FRAMES);
		usbd_transfer_submit(xfer);
		break;

	default:
		if (error != USB_ERR_CANCELLED)
			goto tr_setup;
		break;
	}
}

/* Line -> host: accumulate level statistics for dial tone detection. */
static void
uamodem_rec_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct uamodem_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;
	int16_t buf[64];
	uint32_t mfl, len, off, j;
	int i, nframes, v;

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		usbd_xfer_status(xfer, NULL, NULL, NULL, &nframes);
		mfl = usbd_xfer_max_framelen(xfer);
		pc = usbd_xfer_get_frame(xfer, 0);
		/* IN frames are laid out at max-frame-length strides. */
		for (i = 0, off = 0; i < nframes; i++, off += mfl) {
			len = usbd_xfer_frame_len(xfer, i);
			if (len > sizeof(buf))
				len = sizeof(buf);
			usbd_copy_out(pc, off, buf, len);
			for (j = 0; j < len / 2; j++) {
				v = (int16_t)le16toh(buf[j]);
				if (!sc->sc_offhook && sc->sc_vcid != 0 &&
				    !sc->sc_cid_ready &&
				    uam_cid_sample(&sc->sc_cid, v)) {
					sc->sc_cid_ready = true;
					wakeup(&sc->sc_wchan);
				}
				sc->sc_rec_sum += v;
				sc->sc_rec_sumsq += (uint64_t)((int64_t)v * v);
			}
			sc->sc_rec_n += len / 2;
			sc->sc_rec_total += len / 2;
		}
		/* FALLTHROUGH */
	case USB_ST_SETUP:
tr_setup:
		mfl = usbd_xfer_max_framelen(xfer);
		for (i = 0; i < UAM_ISO_FRAMES; i++)
			usbd_xfer_set_frame_len(xfer, i, mfl);
		usbd_xfer_set_frames(xfer, UAM_ISO_FRAMES);
		usbd_transfer_submit(xfer);
		break;

	default:
		if (error != USB_ERR_CANCELLED)
			goto tr_setup;
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Device operations (worker thread, sc_mtx held, may sleep)          */
/* ------------------------------------------------------------------ */

static usb_error_t
uam_ctrl(struct uamodem_softc *sc, uint8_t rt, uint8_t req, uint16_t val,
    uint16_t idx, void *data, uint16_t len, uint16_t *actlen)
{
	struct usb_device_request r;

	mtx_assert(&sc->sc_mtx, MA_OWNED);
	r.bmRequestType = rt;
	r.bRequest = req;
	USETW(r.wValue, val);
	USETW(r.wIndex, idx);
	USETW(r.wLength, len);
	/* drops sc_mtx while the request is in flight */
	return (usbd_do_request_flags(sc->sc_udev, &sc->sc_mtx, &r, data,
	    USB_SHORT_XFER_OK, actlen, 1000));
}

/*
 * Interruptible delay.  Returns non-zero if the worker must abandon what
 * it is doing (detach, hang-up request).
 */
static int
uam_delay(struct uamodem_softc *sc, int ms)
{
	int end = ticks + uam_ms_to_ticks(ms);

	/* Other events also wake sc_wchan: sleep until the deadline. */
	while ((sc->sc_flags & (UAM_F_GONE | UAM_F_HANGUP)) == 0) {
		if (end - ticks <= 0)
			return (0);
		(void)msleep(&sc->sc_wchan, &sc->sc_mtx, 0, "uamdly",
		    end - ticks);
	}
	return (1);
}

/*
 * Send one 3-byte command to the DAA channel; optionally fetch the reply.
 */
static int
uam_daa_cmd(struct uamodem_softc *sc, uint8_t op, uint8_t a, uint8_t b,
    uint8_t *reply, int replylen)
{
	uint8_t cmd[3] = { op, a, b };
	uint8_t resp[8];
	uint16_t actlen;
	int i;

	if (sc->sc_cmd_iface_no == 0xff)
		return (ENXIO);
	sc->sc_flags &= ~UAM_F_RESP;
	if (uam_ctrl(sc, UT_WRITE_CLASS_INTERFACE, UAM_SEND_ENCAPSULATED, 0,
	    sc->sc_cmd_iface_no, cmd, sizeof(cmd), NULL) != 0)
		return (EIO);
	if (reply == NULL)
		return (0);

	/*
	 * Wait for the "response available" notification, but poll anyway
	 * every 20 ms in case it was lost.
	 */
	for (i = 0; i < 10; i++) {
		if (!(sc->sc_flags & UAM_F_RESP))
			(void)msleep(&sc->sc_resp, &sc->sc_mtx, 0, "uamrsp",
			    uam_ms_to_ticks(20));
		if (sc->sc_flags & UAM_F_GONE)
			return (ENXIO);
		if (uam_ctrl(sc, UT_READ_CLASS_INTERFACE, UAM_GET_ENCAPSULATED,
		    0, sc->sc_cmd_iface_no, resp, sizeof(resp), &actlen) != 0)
			return (EIO);
		if (actlen > 0) {
			memcpy(reply, resp, MIN(actlen, replylen));
			sc->sc_flags &= ~UAM_F_RESP;
			return (0);
		}
	}
	return (ETIMEDOUT);
}

static int
uam_daa_read(struct uamodem_softc *sc, uint8_t reg, uint8_t *val)
{
	return (uam_daa_cmd(sc, UAM_OP_READ_REG, reg, 0, val, 1));
}

static int
uam_daa_write(struct uamodem_softc *sc, uint8_t reg, uint8_t val)
{
	return (uam_daa_cmd(sc, UAM_OP_WRITE_REG, reg, val, NULL, 0));
}

/*
 * Toggle the Si3050 OH bit (register 5 bit 0) directly, leaving the
 * firmware hook state alone.  Used for dial pulses and hook flash,
 * which SET_HOOK_STATE cannot do (see uam_hook_raw()).
 */
static int
uam_daa_hook(struct uamodem_softc *sc, bool off)
{
	return (off ?
	    uam_daa_cmd(sc, UAM_OP_OR_REG, DAA_DCTL1, DAA_DCTL1_OH, NULL, 0) :
	    uam_daa_cmd(sc, UAM_OP_AND_REG, DAA_DCTL1, ~DAA_DCTL1_OH & 0xff,
	    NULL, 0));
}

/*
 * CDC SET_HOOK_STATE.  Firmware behaviour, measured:
 *  - a request for the state the firmware is already in is STALLed
 *    (that is success as far as we are concerned);
 *  - after going off-hook the firmware runs a line check (LINE_STATE 7
 *    then 4/5, ~430 ms).  An on-hook request during roughly the first
 *    400 ms is ACKed but IGNORED, leaving the line seized.  Callers must
 *    go through uam_set_hook(), which enforces a minimum off-hook time.
 */
static int
uam_hook_raw(struct uamodem_softc *sc, bool off)
{
	usb_error_t err;

	err = uam_ctrl(sc, UT_WRITE_CLASS_INTERFACE, UAM_SET_HOOK_STATE,
	    off ? 1 : 0, sc->sc_comm_iface_no, NULL, 0, NULL);
	if (err == USB_ERR_STALLED)
		return (0);	/* already in the requested state */
	if (err != 0) {
		device_printf(sc->sc_dev, "SET_HOOK_STATE(%d): %s\n", off,
		    usbd_errstr(err));
		return (EIO);
	}
	return (0);
}

/*
 * The firmware sometimes acknowledges an on-hook request without acting
 * on it.  Check the DAA OH bit, which is the real hook switch, and
 * insist; as a last resort clear the bit ourselves.
 */
static int
uam_verify_onhook(struct uamodem_softc *sc)
{
	uint8_t v;
	int i;

	for (i = 0; i < 10; i++) {
		if (uam_daa_read(sc, DAA_DCTL1, &v) != 0)
			return (0);	/* no DAA channel: trust the firmware */
		if ((v & DAA_DCTL1_OH) == 0)
			return (0);
		DPRINTF("on-hook not applied (reg5=%02x), retry %d\n", v, i);
		(void)msleep(&sc->sc_offhook_ticks, &sc->sc_mtx, 0, "uamonh",
		    uam_ms_to_ticks(100));
		if (!usbd_device_attached(sc->sc_udev))
			return (EIO);
		(void)uam_hook_raw(sc, false);
	}
	device_printf(sc->sc_dev, "firmware ignores on-hook, forcing DAA\n");
	return (uam_daa_hook(sc, false) ? EIO : 0);
}

/*
 * On-hook line monitoring for Caller ID.  The Si3050 only passes line
 * audio while on-hook if ONHM (register 5 bit 3) is set; without it the
 * ADC sits pinned at full scale.  The firmware rewrites register 5 on
 * every hook change, so this is redone after each on-hook.
 */
static void
uam_cid_monitor(struct uamodem_softc *sc, bool on)
{
	if (!(sc->sc_flags & UAM_F_AUDIO) || sc->sc_offhook)
		return;
	if (on) {
		(void)uam_daa_write(sc, DAA_DCTL1, DAA_DCTL1_ONHM);
		uam_cid_reset(&sc->sc_cid);
		sc->sc_cid_ready = false;
		sc->sc_rec_seen = sc->sc_rec_total;
		sc->sc_rec_check = ticks;
		usbd_transfer_start(sc->sc_xfer[UAM_XFER_REC0]);
		usbd_transfer_start(sc->sc_xfer[UAM_XFER_REC1]);
	} else {
		usbd_transfer_stop(sc->sc_xfer[UAM_XFER_REC0]);
		usbd_transfer_stop(sc->sc_xfer[UAM_XFER_REC1]);
	}
}

/* AT+VCID=n */
static void
uam_vcid_set(struct uamodem_softc *sc, uint8_t mode)
{
	sc->sc_vcid = mode;
	uam_cid_monitor(sc, mode != 0);
}

/*
 * Go off-hook (seize the line) or on-hook (release it), starting or
 * stopping the audio streams with it.
 */
static int
uam_set_hook(struct uamodem_softc *sc, bool off)
{
	int left;

	if (!off && sc->sc_offhook) {
		/* see uam_hook_raw(): respect the minimum off-hook time */
		left = sc->sc_offhook_ticks + uam_ms_to_ticks(UAM_MIN_OFFHOOK_MS)
		    - ticks;
		if (left > 0)
			(void)msleep(&sc->sc_offhook_ticks, &sc->sc_mtx, 0,
			    "uamhook", left);
	}
	if (uam_hook_raw(sc, off) != 0)
		return (EIO);
	if (!off && uam_verify_onhook(sc) != 0)
		return (EIO);
	if (off == sc->sc_offhook)
		return (0);
	sc->sc_offhook = off;
	if (off)
		sc->sc_offhook_ticks = ticks;

	if (sc->sc_flags & UAM_F_AUDIO) {
		if (off) {
			sc->sc_tone_left = 0;
			sc->sc_rec_sum = sc->sc_rec_sumsq = 0;
			sc->sc_rec_n = 0;
			usbd_transfer_start(sc->sc_xfer[UAM_XFER_PLAY0]);
			usbd_transfer_start(sc->sc_xfer[UAM_XFER_PLAY1]);
			usbd_transfer_start(sc->sc_xfer[UAM_XFER_REC0]);
			usbd_transfer_start(sc->sc_xfer[UAM_XFER_REC1]);
		} else {
			usbd_transfer_stop(sc->sc_xfer[UAM_XFER_PLAY0]);
			usbd_transfer_stop(sc->sc_xfer[UAM_XFER_PLAY1]);
			sc->sc_tone_left = 0;
			wakeup(&sc->sc_tone_left);
			/* keep listening on-hook if Caller ID is enabled */
			uam_cid_monitor(sc, sc->sc_vcid != 0);
		}
	}
	return (0);
}

/* Program the firmware country feature and the DAA line interface. */
static int
uam_set_country(struct uamodem_softc *sc, const struct uam_country *c)
{
	uint8_t feat[2];
	int error = 0;

	if (sc->sc_cmd_iface_no == 0xff)
		return (ENXIO);
	USETW(feat, c->iso);
	if (uam_ctrl(sc, UT_WRITE_CLASS_INTERFACE, UAM_SET_COMM_FEATURE,
	    UAM_FEATURE_COUNTRY, sc->sc_cmd_iface_no, feat, 2, NULL) != 0)
		error = EIO;

	/* Register layout as programmed by DAHDI's wctdm driver. */
	error |= uam_daa_write(sc, DAA_ICTL1,
	    (c->ohs << 6) | (c->rz << 1) | c->rt);
	error |= uam_daa_write(sc, DAA_DCTL,
	    (c->dcv << 6) | (c->mini << 4) | (c->ilim << 1));
	error |= uam_daa_write(sc, DAA_ACIM, c->acim);
	error |= uam_daa_write(sc, DAA_ICTL4, 0xa3 | (c->ohs2 << 3));
	if (error == 0)
		sc->sc_country = c;
	return (error ? EIO : 0);
}

/* Read the firmware version and current country; apply the tunable. */
static void
uam_init_device(struct uamodem_softc *sc)
{
	const struct uam_country *c;
	uint8_t v[2], feat[2];
	uint16_t actlen;

	(void)uam_set_hook(sc, false);

	if (uam_daa_cmd(sc, UAM_OP_FW_VERSION, 0, 0, v, 2) == 0)
		sc->sc_fw_version = (v[0] << 8) | v[1];

	if (sc->sc_cmd_iface_no != 0xff &&
	    uam_ctrl(sc, UT_READ_CLASS_INTERFACE, UAM_GET_COMM_FEATURE,
	    UAM_FEATURE_COUNTRY, sc->sc_cmd_iface_no, feat, 2, &actlen) == 0 &&
	    actlen == 2)
		sc->sc_country = uam_find_country_iso(UGETW(feat));

	if (uamodem_country >= 0) {
		c = uam_find_country(uamodem_country);
		if (c == NULL)
			device_printf(sc->sc_dev, "unknown T.35 country %#x\n",
			    uamodem_country);
		else if (uam_set_country(sc, c) != 0)
			device_printf(sc->sc_dev, "cannot set country %s\n",
			    c->name);
	}
	device_printf(sc->sc_dev, "firmware %x.%02x, country %s, %s\n",
	    sc->sc_fw_version >> 8, sc->sc_fw_version & 0xff,
	    sc->sc_country ? sc->sc_country->name : "unknown",
	    (sc->sc_flags & UAM_F_AUDIO) ? "tone+pulse dialling" :
	    "pulse dialling only");
}

/* ------------------------------------------------------------------ */
/* Talking to the tty (sc_mtx held)                                   */
/* ------------------------------------------------------------------ */

static void
uam_tty_write(struct uamodem_softc *sc, const char *s, size_t len)
{
	struct usb_page_cache pc;

	if (!(sc->sc_flags & UAM_F_OPEN) || len == 0)
		return;
	/* A DMA-less page cache is just a plain kernel buffer. */
	memset(&pc, 0, sizeof(pc));
	pc.buffer = __DECONST(char *, s);
	ucom_put_data(&sc->sc_ucom, &pc, 0, len);
}

static void
uam_puts(struct uamodem_softc *sc, const char *s)
{
	uam_tty_write(sc, s, strlen(s));
}

/* Hayes information text: <CR><LF>text<CR><LF> (ATV1) or text<CR><LF>. */
static void
uam_info(struct uamodem_softc *sc, const char *fmt, ...)
{
	char buf[128], eol[3];
	va_list ap;

	eol[0] = sc->sc_sreg[3];
	eol[1] = sc->sc_sreg[4];
	eol[2] = '\0';
	if (sc->sc_verbose && !sc->sc_info_started)
		uam_puts(sc, eol);
	sc->sc_info_started = true;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	uam_puts(sc, buf);
	uam_puts(sc, eol);
}

static void
uam_result(struct uamodem_softc *sc, int code)
{
	char buf[32];

	if (code == UAM_R_NONE || sc->sc_quiet)
		return;
	if (sc->sc_verbose)
		snprintf(buf, sizeof(buf), "%c%c%s%c%c", sc->sc_sreg[3],
		    sc->sc_sreg[4], uam_result_text[code], sc->sc_sreg[3],
		    sc->sc_sreg[4]);
	else
		snprintf(buf, sizeof(buf), "%d%c", code, sc->sc_sreg[3]);
	uam_puts(sc, buf);
}

/* ------------------------------------------------------------------ */
/* Telephony                                                          */
/* ------------------------------------------------------------------ */

/* Snapshot and reset the capture statistics; returns AC RMS or -1. */
static int
uam_take_rms(struct uamodem_softc *sc)
{
	int64_t mean;
	uint64_t var;
	uint32_t n = sc->sc_rec_n;

	if (n == 0)
		return (-1);
	mean = sc->sc_rec_sum / (int64_t)n;
	var = sc->sc_rec_sumsq / n;
	/* remove DC: the ADC sits at a large offset on an open line */
	var = var > (uint64_t)(mean * mean) ? var - (uint64_t)(mean * mean) : 0;
	sc->sc_rec_sum = sc->sc_rec_sumsq = 0;
	sc->sc_rec_n = 0;
	sc->sc_last_rms = (int)uam_isqrt(var);
	sc->sc_last_dc = (int)mean;
	return (sc->sc_last_rms);
}

/*
 * Wait up to S6 seconds for dial tone: at least 500 ms of continuous
 * audio energy above the threshold.  Also fails at once if the firmware
 * reports the line as unplugged.
 */
static int
uam_wait_dialtone(struct uamodem_softc *sc)
{
	int ms, good = 0, rms, limit;

	limit = MAX(sc->sc_sreg[6], 2) * 1000;
	(void)uam_take_rms(sc);
	for (ms = 0; ms < limit; ms += 100) {
		if (uam_delay(sc, 100))
			return (UAM_R_NOCARRIER);
		if (sc->sc_line_state == UAM_LINE_UNPLUGGED)
			return (UAM_R_NODIALTONE);
		rms = uam_take_rms(sc);
		if (rms >= sc->sc_dialtone_rms) {
			if (++good >= 5)
				return (UAM_R_OK);
		} else
			good = 0;
	}
	return (UAM_R_NODIALTONE);
}

/* Play one DTMF digit for S11 ms followed by S11 ms of silence. */
static int
uam_dial_dtmf(struct uamodem_softc *sc, char d)
{
	static const char keys[] = "123A456B789C*0#D";
	static const uint16_t lo[] = { 697, 770, 852, 941 };
	static const uint16_t hi[] = { 1209, 1336, 1477, 1633 };
	const char *p;
	int k, ms;

	p = strchr(keys, d);
	if (p == NULL || d == '\0')
		return (0);
	k = p - keys;
	ms = MAX(sc->sc_sreg[11], 50);

	sc->sc_tone_ph[0] = sc->sc_tone_ph[1] = 0;
	sc->sc_tone_step[0] = (uint32_t)(((uint64_t)lo[k / 4] << 32) /
	    UAM_RATE);
	sc->sc_tone_step[1] = (uint32_t)(((uint64_t)hi[k % 4] << 32) /
	    UAM_RATE);
	sc->sc_tone_left = UAM_RATE * ms / 1000;
	while (sc->sc_tone_left > 0) {
		if (sc->sc_flags & (UAM_F_GONE | UAM_F_HANGUP)) {
			sc->sc_tone_left = 0;
			return (1);
		}
		(void)msleep(&sc->sc_tone_left, &sc->sc_mtx, 0, "uamdtmf",
		    uam_ms_to_ticks(100));
	}
	return (uam_delay(sc, ms));
}

/*
 * Decadic (pulse) dialling at 10 pulses/s, 60 ms break / 40 ms make,
 * 750 ms inter-digit pause.  '0' is 10 pulses.
 */
static int
uam_dial_pulse(struct uamodem_softc *sc, char d)
{
	int n;

	if (d < '0' || d > '9')
		return (0);	/* * # A-D cannot be pulse dialled */
	n = (d == '0') ? 10 : d - '0';
	while (n-- > 0) {
		if (uam_daa_hook(sc, false) || uam_delay(sc, 60))
			return (1);
		if (uam_daa_hook(sc, true) || uam_delay(sc, 40))
			return (1);
	}
	return (uam_delay(sc, 750));
}

/* Hook flash: a 500 ms break, done on the DAA like a dial pulse. */
static int
uam_flash(struct uamodem_softc *sc)
{
	return (uam_daa_hook(sc, false) || uam_delay(sc, 500) ||
	    uam_daa_hook(sc, true) || uam_delay(sc, 500));
}

/* ATD<string>: returns the result code to print. */
static int
uam_dial(struct uamodem_softc *sc, const char *s)
{
	bool tone = true;
	int r;

	if (uam_set_hook(sc, true) != 0)
		return (UAM_R_ERROR);

	/* ATX0, X1, X3: blind dial; X2, X4: wait for dial tone. */
	if ((sc->sc_flags & UAM_F_AUDIO) &&
	    (sc->sc_xlevel == 2 || sc->sc_xlevel == 4)) {
		r = uam_wait_dialtone(sc);
		if (r != UAM_R_OK)
			goto fail;
	} else {
		/* blind dial: wait S6 seconds, check nothing */
		if (uam_delay(sc, MAX(sc->sc_sreg[6], 2) * 1000)) {
			r = UAM_R_NOCARRIER;
			goto fail;
		}
	}

	for (; *s != '\0'; s++) {
		char c = toupper(*s);

		switch (c) {
		case 'T':
			tone = true;
			continue;
		case 'P':
			tone = false;
			continue;
		case ',':
			if (uam_delay(sc, sc->sc_sreg[8] * 1000))
				goto aborted;
			continue;
		case 'W':
			if ((sc->sc_flags & UAM_F_AUDIO) &&
			    (r = uam_wait_dialtone(sc)) != UAM_R_OK)
				goto fail;
			continue;
		case '!':	/* hook flash */
			if (uam_flash(sc))
				goto aborted;
			continue;
		case ';':
			return (UAM_R_OK);	/* voice dial: stay off-hook */
		default:
			break;
		}
		if (!(isdigit(c) || c == '*' || c == '#' ||
		    (c >= 'A' && c <= 'D')))
			continue;	/* '-', '(', ')', spaces... */
		if (tone && !(sc->sc_flags & UAM_F_AUDIO)) {
			/* no audio path: fall back to pulse for digits */
			tone = false;
		}
		if (tone ? uam_dial_dtmf(sc, c) : uam_dial_pulse(sc, c))
			goto aborted;
	}
	/*
	 * There is no data pump, so a "data" call cannot CONNECT.  Keep the
	 * line up as a voice call and report OK; ATH or DTR drop hangs up.
	 */
	return (UAM_R_OK);

aborted:
	r = UAM_R_NOCARRIER;
fail:
	(void)uam_set_hook(sc, false);
	return (r);
}

/* ATI4: line diagnostics straight from the DAA. */
static void
uam_line_diag(struct uamodem_softc *sc)
{
	uint8_t lvs = 0, lcs = 0, lcs2 = 0, dctl1 = 0;

	if (uam_daa_read(sc, DAA_LVS, &lvs) != 0 ||
	    uam_daa_read(sc, DAA_LCS, &lcs) != 0 ||
	    uam_daa_read(sc, DAA_LCS2, &lcs2) != 0) {
		uam_info(sc, "DAA not responding");
		return;
	}
	(void)uam_daa_read(sc, DAA_DCTL1, &dctl1);
	uam_info(sc, "Hook: %s (DAA OH bit %d)", sc->sc_offhook ? "off" : "on",
	    dctl1 & DAA_DCTL1_OH);
	uam_info(sc, "Jack: %s (line state %d)",
	    sc->sc_line_state == UAM_LINE_UNPLUGGED ? "no line" :
	    sc->sc_line_state == UAM_LINE_PLUGGED ? "line present" : "unknown",
	    sc->sc_line_state);
	uam_info(sc, "Line voltage: %d V", (int)(int8_t)lvs);
	uam_info(sc, "Loop current: %d.%d mA (LCS %d)", lcs2 * 11 / 10,
	    lcs2 * 11 % 10, lcs & 0x1f);
	if (sc->sc_flags & UAM_F_AUDIO)
		uam_info(sc, "Line audio RMS: %d", sc->sc_last_rms);
}

/* ------------------------------------------------------------------ */
/* AT command interpreter                                             */
/* ------------------------------------------------------------------ */

static void
uam_reset_settings(struct uamodem_softc *sc)
{
	memset(sc->sc_sreg, 0, sizeof(sc->sc_sreg));
	sc->sc_sreg[2] = '+';	/* escape character */
	sc->sc_sreg[3] = '\r';	/* line terminator */
	sc->sc_sreg[4] = '\n';	/* response formatting */
	sc->sc_sreg[5] = '\b';	/* backspace */
	sc->sc_sreg[6] = 2;	/* dial tone wait, s */
	sc->sc_sreg[7] = 50;	/* carrier wait, s (unused) */
	sc->sc_sreg[8] = 2;	/* comma pause, s */
	sc->sc_sreg[10] = 14;	/* carrier loss, 1/10 s (unused) */
	sc->sc_sreg[11] = 95;	/* DTMF duration / spacing, ms */
	sc->sc_echo = true;
	sc->sc_quiet = false;
	sc->sc_verbose = true;
	sc->sc_xlevel = 4;
	sc->sc_dtr_mode = 2;
}

/* Parse a decimal number at *pp (default "def" if none). */
static int
uam_num(const char **pp, int def)
{
	const char *p = *pp;
	int v = 0;

	if (!isdigit(*p))
		return (def);
	while (isdigit(*p))
		v = v * 10 + (*p++ - '0');
	*pp = p;
	return (v);
}

static int
uam_hex(const char **pp)
{
	const char *p = *pp;
	int v = 0, n = 0;

	for (; n < 2 && isxdigit(*p); n++, p++)
		v = v * 16 + (isdigit(*p) ? *p - '0' : (toupper(*p) - 'A' + 10));
	*pp = p;
	return (n ? v : -1);
}

/*
 * Caller ID control, after "+VCID" or "#CID":  =0 off, =1 formatted,
 * =2 unformatted (hex), ? query, =? range.
 */
static int
uam_vcid_cmd(struct uamodem_softc *sc, const char **pp)
{
	const char *p = *pp;
	int v;

	if (*p == '?') {
		p++;
		uam_info(sc, "%d", sc->sc_vcid);
	} else if (strncmp(p, "=?", 2) == 0) {
		p += 2;
		uam_info(sc, "(0-2)");
	} else if (*p == '=') {
		p++;
		v = uam_num(&p, -1);
		if (v < 0 || v > 2)
			return (UAM_R_ERROR);
		if (v != 0 && !(sc->sc_flags & UAM_F_AUDIO))
			return (UAM_R_ERROR);	/* needs the line audio */
		uam_vcid_set(sc, v);
	} else
		return (UAM_R_ERROR);
	*pp = p;
	return (UAM_R_OK);
}

/* Extended (+) commands. */
static int
uam_exec_plus(struct uamodem_softc *sc, const char **pp)
{
	const struct uam_country *c;
	const char *p = *pp;
	char list[UAM_NCOUNTRIES * 3 + 1];
	u_int i;
	int v, r;

	if (strncasecmp(p, "GCI", 3) == 0) {
		p += 3;
		if (*p == '?') {
			p++;
			uam_info(sc, "+GCI: %02X",
			    sc->sc_country ? sc->sc_country->t35 : 0xff);
		} else if (strncmp(p, "=?", 2) == 0) {
			p += 2;
			list[0] = '\0';
			for (i = 0; i < UAM_NCOUNTRIES; i++)
				snprintf(list + strlen(list),
				    sizeof(list) - strlen(list), "%s%02X",
				    i ? "," : "", uam_countries[i].t35);
			uam_info(sc, "+GCI: (%s)", list);
		} else if (*p == '=') {
			p++;
			v = uam_hex(&p);
			c = v < 0 ? NULL : uam_find_country(v);
			if (c == NULL || uam_set_country(sc, c) != 0)
				return (UAM_R_ERROR);
		} else
			return (UAM_R_ERROR);
	} else if (strncasecmp(p, "VCID", 4) == 0) {
		p += 4;
		r = uam_vcid_cmd(sc, &p);
		if (r != UAM_R_OK)
			return (r);
	} else if (strncasecmp(p, "FCLASS", 6) == 0) {
		/* Only class 0 ("data") exists, and only in name. */
		p += 6;
		if (*p == '?') {
			p++;
			uam_info(sc, "0");
		} else if (strncmp(p, "=?", 2) == 0) {
			p += 2;
			uam_info(sc, "0");
		} else if (*p == '=') {
			p++;
			if (uam_num(&p, -1) != 0)
				return (UAM_R_ERROR);
		} else
			return (UAM_R_ERROR);
	} else if (strncasecmp(p, "MS", 2) == 0) {
		/*
		 * Modulation selection (sent by Apple's CCL scripts and many
		 * dialers): accepted and ignored, there is no data pump.
		 */
		p += 2;
		while (*p != '\0' && *p != ';')
			p++;
	} else
		return (UAM_R_ERROR);
	if (*p == ';')
		p++;
	*pp = p;
	return (UAM_R_OK);
}

/*
 * Execute one command line (text after "AT", spaces stripped except in a
 * dial string).  Returns the final result code.
 */
static int
uam_exec(struct uamodem_softc *sc, const char *p)
{
	int r, n, v;
	char c;

	while ((c = toupper(*p)) != '\0') {
		p++;
		switch (c) {
		case 'A':	/* answer: seize the line (voice) */
			if (uam_set_hook(sc, true) != 0)
				return (UAM_R_ERROR);
			sc->sc_sreg[1] = 0;
			return (UAM_R_OK);
		case 'D':	/* dial: rest of the line is the number */
			return (uam_dial(sc, p));
		case 'E':
			sc->sc_echo = uam_num(&p, 0) != 0;
			break;
		case 'H':
			if (uam_set_hook(sc, uam_num(&p, 0) == 1) != 0)
				return (UAM_R_ERROR);
			break;
		case 'I':
			switch (uam_num(&p, 0)) {
			case 0:
				uam_info(sc, "Apple USB Modem");
				break;
			case 1:
				uam_info(sc, "uamodem " UAMODEM_VERSION
				    " (FreeBSD), SM56 + Si3050/Si3018");
				break;
			case 2:
				uam_info(sc, "OK");
				break;
			case 3:
				uam_info(sc, "Firmware %x.%02x",
				    sc->sc_fw_version >> 8,
				    sc->sc_fw_version & 0xff);
				break;
			case 4:
				uam_line_diag(sc);
				break;
			default:
				return (UAM_R_ERROR);
			}
			break;
		case 'L':	/* speaker volume: no speaker */
		case 'M':	/* speaker mode */
		case 'B':	/* CCITT/Bell */
		case 'N':	/* auto-mode */
		case 'W':	/* connect message format */
			(void)uam_num(&p, 0);
			break;
		case 'O':	/* online: no data mode exists */
			return (UAM_R_NOCARRIER);
		case 'Q':
			sc->sc_quiet = uam_num(&p, 0) != 0;
			break;
		case 'S':
			n = uam_num(&p, -1);
			if (n < 0 || n >= UAM_NSREG)
				return (UAM_R_ERROR);
			if (*p == '?') {
				p++;
				uam_info(sc, "%03d", sc->sc_sreg[n]);
			} else if (*p == '=') {
				p++;
				v = uam_num(&p, 0);
				if (v > 255 || n == 1)
					return (UAM_R_ERROR);
				sc->sc_sreg[n] = v;
			} else
				return (UAM_R_ERROR);
			break;
		case 'V':
			sc->sc_verbose = uam_num(&p, 0) != 0;
			break;
		case 'X':
			v = uam_num(&p, 0);
			if (v > 4)
				return (UAM_R_ERROR);
			sc->sc_xlevel = v;
			break;
		case 'Z':
			if (uam_set_hook(sc, false) != 0)
				return (UAM_R_ERROR);
			uam_reset_settings(sc);
			uam_vcid_set(sc, 0);
			break;
		case '&':
			c = toupper(*p);
			if (c == '\0')
				return (UAM_R_ERROR);
			p++;
			v = uam_num(&p, 0);
			switch (c) {
			case 'F':
				uam_reset_settings(sc);
				uam_vcid_set(sc, 0);
				break;
			case 'D':
				if (v > 3)
					return (UAM_R_ERROR);
				sc->sc_dtr_mode = v;
				break;
			case 'C': case 'K': case 'Q': case 'S': case 'W':
			case 'V': case 'Y':
				break;	/* accepted, meaningless here */
			default:
				return (UAM_R_ERROR);
			}
			break;
		case '#':
			/* AT#CID=n: the Rockwell/Conexant spelling of +VCID */
			if (strncasecmp(p, "CID", 3) != 0)
				return (UAM_R_ERROR);
			p += 3;
			r = uam_vcid_cmd(sc, &p);
			if (r != UAM_R_OK)
				return (r);
			break;
		case '\\':
		case '%':
			/* \N3, %C1 ... (error correction/compression) */
			if (*p != '\0')
				p++;
			(void)uam_num(&p, 0);
			break;
		case '+':
			r = uam_exec_plus(sc, &p);
			if (r != UAM_R_OK)
				return (r);
			break;
		default:
			return (UAM_R_ERROR);
		}
	}
	return (UAM_R_OK);
}

/* A complete line was entered; find "AT", strip blanks and execute. */
static void
uam_command_line(struct uamodem_softc *sc, const char *line)
{
	char cmd[UAM_LINE_MAX];
	const char *p;
	int i = 0;
	bool in_dial = false;

	/* Skip anything before "AT" (line noise, stray characters). */
	for (p = line; p[0] != '\0'; p++)
		if (toupper(p[0]) == 'A' && toupper(p[1]) == 'T')
			break;
	if (p[0] == '\0')
		return;
	strlcpy(sc->sc_lastcmd, line, sizeof(sc->sc_lastcmd));
	for (p += 2; *p != '\0' && i < (int)sizeof(cmd) - 1; p++) {
		if (toupper(*p) == 'D')
			in_dial = true;
		if (*p == ' ' && !in_dial)
			continue;
		cmd[i++] = *p;
	}
	cmd[i] = '\0';
	sc->sc_info_started = false;
	uam_result(sc, uam_exec(sc, cmd));
}

/* Feed one character typed by the user into the line editor. */
static void
uam_input_char(struct uamodem_softc *sc, char ch)
{
	char prev = sc->sc_prevchar;

	sc->sc_prevchar = ch;

	if (ch == (char)sc->sc_sreg[5] || ch == 0x7f) {
		if (sc->sc_linelen > 0) {
			sc->sc_linelen--;
			if (sc->sc_echo)
				uam_puts(sc, "\b \b");
		}
		return;
	}
	if (sc->sc_echo)
		uam_tty_write(sc, &ch, 1);

	/* "A/" repeats the last command line immediately. */
	if (ch == '/' && toupper(prev) == 'A' && sc->sc_linelen == 1) {
		sc->sc_linelen = 0;
		if (sc->sc_echo)
			uam_puts(sc, "\r\n");
		uam_command_line(sc, sc->sc_lastcmd);
		return;
	}
	if (ch == (char)sc->sc_sreg[3]) {
		sc->sc_line[sc->sc_linelen] = '\0';
		sc->sc_linelen = 0;
		uam_command_line(sc, sc->sc_line);
		return;
	}
	if (ch == '\n' || ch == '\0')
		return;
	if (sc->sc_linelen < UAM_LINE_MAX - 1)
		sc->sc_line[sc->sc_linelen++] = ch;
}

/* Drain everything the tty has queued for "transmission". */
static void
uam_read_tty(struct uamodem_softc *sc)
{
	struct usb_page_cache pc;
	char buf[64];
	uint32_t actlen, i;

	memset(&pc, 0, sizeof(pc));
	pc.buffer = buf;
	while (!(sc->sc_flags & UAM_F_GONE) &&
	    ucom_get_data(&sc->sc_ucom, &pc, 0, sizeof(buf), &actlen) &&
	    actlen > 0) {
		for (i = 0; i < actlen; i++) {
			uam_input_char(sc, buf[i]);
			if (sc->sc_flags & UAM_F_GONE)
				return;
		}
	}
}

/* ------------------------------------------------------------------ */
/* Worker thread                                                      */
/* ------------------------------------------------------------------ */

/* Print a decoded Caller ID message (AT+VCID=1 or 2). */
static void
uam_cid_field(void *arg, const char *tag, const char *val)
{
	uam_info(arg, "%s = %s", tag, val);
}

static void
uam_cid_report(struct uamodem_softc *sc)
{
	uint8_t msg[UAM_CID_MAXMSG];
	char hex[2 * UAM_CID_MAXMSG + 1];
	int i, len;

	len = sc->sc_cid.len;
	memcpy(msg, sc->sc_cid.msg, len);
	sc->sc_cid_ready = false;	/* decoder may run again */
	sc->sc_cid_msgs++;
	for (i = 0; i < len && i < (int)sizeof(sc->sc_cid_last) / 2 - 1; i++)
		snprintf(sc->sc_cid_last + 2 * i, 3, "%02X", msg[i]);
	DPRINTF("caller ID (%s), %d bytes\n", sc->sc_cid.standard, len);
	if (sc->sc_vcid == 0 || sc->sc_offhook)
		return;
	sc->sc_info_started = false;
	if (sc->sc_vcid == 2) {
		for (i = 0; i < len; i++)
			snprintf(hex + 2 * i, 3, "%02X", msg[i]);
		uam_info(sc, "MESG = %s", hex);
	} else
		uam_cid_parse(msg, len, uam_cid_field, sc);
}

/*
 * The capture stream occasionally starts delivering only empty packets
 * (seen once on hardware).  If it should be running but no samples
 * arrived for over a second, restart it.
 */
static void
uam_rec_watchdog(struct uamodem_softc *sc)
{
	if (!(sc->sc_flags & UAM_F_AUDIO) ||
	    !(sc->sc_offhook || sc->sc_vcid != 0))
		return;
	if ((u_int)(ticks - sc->sc_rec_check) < (u_int)hz)
		return;
	sc->sc_rec_check = ticks;
	if (sc->sc_rec_total != sc->sc_rec_seen) {
		sc->sc_rec_seen = sc->sc_rec_total;
		return;
	}
	sc->sc_rec_restarts++;
	DPRINTF("capture stalled, restarting\n");
	usbd_transfer_stop(sc->sc_xfer[UAM_XFER_REC0]);
	usbd_transfer_stop(sc->sc_xfer[UAM_XFER_REC1]);
	usbd_transfer_start(sc->sc_xfer[UAM_XFER_REC0]);
	usbd_transfer_start(sc->sc_xfer[UAM_XFER_REC1]);
}

static void
uamodem_worker(void *arg)
{
	struct uamodem_softc *sc = arg;

	mtx_lock(&sc->sc_mtx);
	uam_reset_settings(sc);
	uam_init_device(sc);

	while (!(sc->sc_flags & UAM_F_GONE)) {
		if (sc->sc_flags & UAM_F_HANGUP) {
			sc->sc_flags &= ~UAM_F_HANGUP;
			if (sc->sc_offhook)
				(void)uam_set_hook(sc, false);
			continue;
		}
		if (sc->sc_ring_events > 0) {
			sc->sc_ring_events = 0;
			sc->sc_last_ring = ticks;
			if (sc->sc_sreg[1] < 255)
				sc->sc_sreg[1]++;
			if (!sc->sc_offhook) {
				uam_result(sc, UAM_R_RING);
				if (sc->sc_sreg[0] != 0 &&
				    sc->sc_sreg[1] >= sc->sc_sreg[0] &&
				    (sc->sc_flags & UAM_F_OPEN)) {
					sc->sc_sreg[1] = 0;
					uam_result(sc,
					    uam_set_hook(sc, true) == 0 ?
					    UAM_R_OK : UAM_R_ERROR);
				}
			}
			continue;
		}
		if (sc->sc_cid_ready) {
			uam_cid_report(sc);
			continue;
		}
		if (sc->sc_flags & UAM_F_TXPEND) {
			sc->sc_flags &= ~UAM_F_TXPEND;
			uam_read_tty(sc);
			continue;
		}
		/* Ring counter resets 8 s after the last ring. */
		if (sc->sc_sreg[1] != 0 &&
		    (u_int)(ticks - sc->sc_last_ring) > (u_int)(8 * hz))
			sc->sc_sreg[1] = 0;
		/* Capture watchdog: see uam_rec_watchdog(). */
		uam_rec_watchdog(sc);
		/* Keep the last_rms sysctl fresh while off-hook. */
		if (sc->sc_offhook && (sc->sc_flags & UAM_F_AUDIO))
			(void)uam_take_rms(sc);
		(void)msleep(&sc->sc_wchan, &sc->sc_mtx, 0, "uamidle", hz);
	}

	/* Leaving: never leave the line seized. */
	if (sc->sc_offhook)
		(void)uam_set_hook(sc, false);
	sc->sc_flags &= ~UAM_F_THREAD;
	mtx_unlock(&sc->sc_mtx);
	kthread_exit();		/* does wakeup(curthread) for detach */
}

/* ------------------------------------------------------------------ */
/* ucom callbacks (sc_mtx held)                                       */
/* ------------------------------------------------------------------ */

static void
uamodem_cfg_get_status(struct ucom_softc *ucom, uint8_t *lsr, uint8_t *msr)
{
	struct uamodem_softc *sc = ucom->sc_parent;

	*lsr = 0;
	/*
	 * Always "ready to talk".  DCD tracks the hook so that a dialer
	 * using the callin (ttyU) device sees the call in progress.
	 */
	*msr = SER_DSR | SER_CTS | (sc->sc_offhook ? SER_DCD : 0);
}

/*
 * DTR (the CDC-ACM SET_CONTROL_LINE_STATE equivalent).  With AT&D2 (the
 * default) dropping DTR hangs up, like a real modem.
 */
static void
uamodem_cfg_set_dtr(struct ucom_softc *ucom, uint8_t onoff)
{
	struct uamodem_softc *sc = ucom->sc_parent;

	if (onoff) {
		sc->sc_flags |= UAM_F_DTR;
		return;
	}
	sc->sc_flags &= ~UAM_F_DTR;
	if (sc->sc_dtr_mode >= 2 && sc->sc_offhook) {
		sc->sc_flags |= UAM_F_HANGUP;
		wakeup(&sc->sc_wchan);
		wakeup(&sc->sc_tone_left);
	}
}

static void
uamodem_cfg_set_rts(struct ucom_softc *ucom, uint8_t onoff)
{
	/* No UART, no flow control: nothing to do. */
}

static void
uamodem_cfg_open(struct ucom_softc *ucom)
{
	struct uamodem_softc *sc = ucom->sc_parent;

	sc->sc_flags |= UAM_F_OPEN;
	sc->sc_linelen = 0;
}

static void
uamodem_cfg_close(struct ucom_softc *ucom)
{
	struct uamodem_softc *sc = ucom->sc_parent;

	sc->sc_flags &= ~(UAM_F_OPEN | UAM_F_TXPEND);
	sc->sc_flags |= UAM_F_HANGUP;	/* closing the port hangs up */
	wakeup(&sc->sc_wchan);
	wakeup(&sc->sc_tone_left);
}

/*
 * The CDC-ACM SET_LINE_CODING equivalent.  There is no serial line
 * between the host and this modem, so every speed / parity / stop-bit
 * setting is valid and has no effect.
 */
static int
uamodem_pre_param(struct ucom_softc *ucom, struct termios *t)
{
	return (0);
}

static void
uamodem_start_read(struct ucom_softc *ucom)
{
	/* Responses are generated by the worker; nothing to start. */
}

static void
uamodem_stop_read(struct ucom_softc *ucom)
{
}

/* The tty has characters for the "modem": hand them to the worker. */
static void
uamodem_start_write(struct ucom_softc *ucom)
{
	struct uamodem_softc *sc = ucom->sc_parent;

	sc->sc_flags |= UAM_F_TXPEND;
	wakeup(&sc->sc_wchan);
}

static void
uamodem_stop_write(struct ucom_softc *ucom)
{
}

static void
uamodem_poll(struct ucom_softc *ucom)
{
	struct uamodem_softc *sc = ucom->sc_parent;

	usbd_transfer_poll(sc->sc_xfer, UAM_N_XFER);
}

/* ------------------------------------------------------------------ */
/* Module glue                                                        */
/* ------------------------------------------------------------------ */

static device_method_t uamodem_methods[] = {
	DEVMETHOD(device_probe, uamodem_probe),
	DEVMETHOD(device_attach, uamodem_attach),
	DEVMETHOD(device_detach, uamodem_detach),
	DEVMETHOD_END
};

static driver_t uamodem_driver = {
	.name = "uamodem",
	.methods = uamodem_methods,
	.size = sizeof(struct uamodem_softc),
};

DRIVER_MODULE(uamodem, uhub, uamodem_driver, NULL, NULL);
MODULE_DEPEND(uamodem, ucom, 1, 1, 1);
MODULE_DEPEND(uamodem, usb, 1, 1, 1);
MODULE_VERSION(uamodem, UAMODEM_MODVER);
USB_PNP_HOST_INFO(uamodem_devs);
/* kldunload must wait until no process holds the detached tty open */
UCOM_UNLOAD_DRAIN(uamodem);
