/*
 * modemprobe - step 1 probe tool for the Apple USB Modem (05ac:1401)
 *
 * Interface 0 is a CDC Telephone Control Model interface with a Direct Line
 * Management functional descriptor, so we try the standard CDC requests:
 *   SET_HOOK_STATE (0x11), GET/SET_COMM_FEATURE (0x03/0x02), ...
 * and listen to CDC notifications on the interrupt endpoint 0x81.
 * Line audio is taken from uaudio (/dev/dsp3 by default).
 *
 * Usage (as root):
 *   modemprobe info
 *   modemprobe listen <secs>
 *   modemprobe hook <0|1|2>                (0=on hook, 1=off hook, 2=snoop)
 *   modemprobe raw <bmRT> <bReq> <wValue> <wIndex> <wLength|hexdata>
 *   modemprobe offhook <secs> <out.raw>    off hook, record line, on hook
 *   modemprobe dial <digits> <secs> <out.raw>  off hook, wait, play DTMF, record
 */
#include <sys/types.h>
#include <sys/soundcard.h>
#include <err.h>
#include <fcntl.h>
#include <libusb.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define VID 0x05ac
#define PID 0x1401
#define EP_INT 0x81
#define RATE 8000

static libusb_device_handle *h;
static volatile int stop_listen;
static const char *dspdev = "/dev/dsp3";
static struct timespec t0;
static uint16_t ifno = 1;	/* telephony requests go to this interface */

static double
now(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9;
}

static void
hexdump(const unsigned char *b, int n)
{
	for (int i = 0; i < n; i++)
		printf("%02x ", b[i]);
}

static int
ctrl(uint8_t rt, uint8_t req, uint16_t val, uint16_t idx, unsigned char *d,
    uint16_t len)
{
	int r = libusb_control_transfer(h, rt, req, val, idx, d, len, 1000);
	printf("[%7.3f] ctrl rt=%02x req=%02x val=%04x idx=%04x len=%u -> ",
	    now(), rt, req, val, idx, len);
	if (r < 0)
		printf("%s\n", libusb_error_name(r));
	else {
		printf("%d", r);
		if ((rt & 0x80) && r > 0) {
			printf(" : ");
			hexdump(d, r);
		}
		printf("\n");
	}
	fflush(stdout);
	return r;
}

static const char *
notif_name(int n)
{
	switch (n) {
	case 0x00: return "NETWORK_CONNECTION";
	case 0x01: return "RESPONSE_AVAILABLE";
	case 0x08: return "AUX_JACK_HOOK_STATE";
	case 0x09: return "RING_DETECT";
	case 0x20: return "SERIAL_STATE";
	case 0x28: return "CALL_STATE_CHANGE";
	case 0x29: return "LINE_STATE_CHANGE";
	case 0x2a: return "CONNECTION_SPEED_CHANGE";
	default: return "?";
	}
}

static void *
listener(void *arg)
{
	unsigned char b[64];
	int n, r;
	(void)arg;
	while (!stop_listen) {
		r = libusb_interrupt_transfer(h, EP_INT, b, sizeof(b), &n, 200);
		if (r == LIBUSB_ERROR_TIMEOUT)
			continue;
		if (r < 0) {
			printf("[%7.3f] int: %s\n", now(), libusb_error_name(r));
			usleep(100000);
			continue;
		}
		printf("[%7.3f] NOTIFY (%d): ", now(), n);
		hexdump(b, n);
		if (n >= 2)
			printf(" <%s>", notif_name(b[1]));
		printf("\n");
		fflush(stdout);
	}
	return NULL;
}

static int
dsp_open(int mode)
{
	int fd = open(dspdev, mode);
	if (fd < 0)
		err(1, "%s", dspdev);
	int fmt = AFMT_S16_LE, ch = 1, rate = RATE;
	if (ioctl(fd, SNDCTL_DSP_SETFMT, &fmt) < 0 ||
	    ioctl(fd, SNDCTL_DSP_CHANNELS, &ch) < 0 ||
	    ioctl(fd, SNDCTL_DSP_SPEED, &rate) < 0)
		err(1, "dsp setup");
	printf("dsp %s: fmt=%#x ch=%d rate=%d\n", dspdev, fmt, ch, rate);
	return fd;
}

struct rec_arg { int fd; FILE *out; double secs; };

static void *
recorder(void *a)
{
	struct rec_arg *ra = a;
	int16_t buf[RATE / 10];
	size_t total = 0, want = (size_t)(ra->secs * RATE);
	while (total < want) {
		ssize_t n = read(ra->fd, buf, sizeof(buf));
		if (n <= 0) {
			warn("dsp read");
			break;
		}
		fwrite(buf, 1, n, ra->out);
		/* level meter per 100 ms block */
		double s = 0;
		for (int i = 0; i < n / 2; i++)
			s += (double)buf[i] * buf[i];
		total += n / 2;
		printf("[%7.3f] rms=%7.1f\n", now(), sqrt(s / (n / 2)));
		fflush(stdout);
	}
	return NULL;
}

static void
dtmf(int fd, char d, int ms)
{
	static const char *keys = "123A456B789C*0#D";
	static const double lo[] = { 697, 770, 852, 941 };
	static const double hi[] = { 1209, 1336, 1477, 1633 };
	const char *p = strchr(keys, d);
	if (p == NULL)
		return;
	int k = p - keys;
	int n = RATE * ms / 1000, gap = RATE * 80 / 1000;
	int16_t *b = calloc(n + gap, 2);
	for (int i = 0; i < n; i++)
		b[i] = 6000 * (sin(2 * M_PI * lo[k / 4] * i / RATE) +
		    sin(2 * M_PI * hi[k % 4] * i / RATE));
	write(fd, b, (n + gap) * 2);
	free(b);
}

static void
start_listener(pthread_t *t)
{
	stop_listen = 0;
	pthread_create(t, NULL, listener, NULL);
}

static void
stop_listener(pthread_t t)
{
	stop_listen = 1;
	pthread_join(t, NULL);
}

#define SET_COMM_FEATURE 0x02
#define GET_COMM_FEATURE 0x03
#define SET_HOOK_STATE 0x11
#define GET_LINE_CODING 0x21

int
main(int argc, char **argv)
{
	unsigned char buf[256];
	pthread_t lt;
	int r;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	setvbuf(stdout, NULL, _IOLBF, 0);
	if (getenv("DSP"))
		dspdev = getenv("DSP");
	if (getenv("IFNO"))
		ifno = atoi(getenv("IFNO"));
	if (argc < 2)
		errx(1, "usage: see source");
	if ((r = libusb_init(NULL)) < 0)
		errx(1, "libusb_init: %s", libusb_error_name(r));
	h = libusb_open_device_with_vid_pid(NULL, VID, PID);
	if (h == NULL)
		errx(1, "modem %04x:%04x not found (root?)", VID, PID);
	if ((r = libusb_claim_interface(h, 0)) < 0)
		warnx("claim if0: %s", libusb_error_name(r));
	if ((r = libusb_claim_interface(h, 1)) < 0)
		warnx("claim if1: %s", libusb_error_name(r));

	if (!strcmp(argv[1], "info")) {
		/* comm features: 1=ABSTRACT_STATE 2=COUNTRY_SETTING */
		for (int sel = 1; sel <= 2; sel++)
			ctrl(0xa1, GET_COMM_FEATURE, sel, ifno, buf, 2);
		ctrl(0xa1, GET_LINE_CODING, 0, ifno, buf, 7);
		/* Telephone control model GET_* requests */
		ctrl(0xa1, 0x31, 0, ifno, buf, 4);	/* GET_RINGER_PARMS */
		ctrl(0xa1, 0x33, 0, ifno, buf, 2);	/* GET_OPERATION_PARMS */
		ctrl(0xa1, 0x35, 0, ifno, buf, 64);	/* GET_LINE_PARMS */
	} else if (!strcmp(argv[1], "listen") && argc == 3) {
		start_listener(&lt);
		sleep(atoi(argv[2]));
		stop_listener(lt);
	} else if (!strcmp(argv[1], "encap") && argc >= 3) {
		/* SEND_ENCAPSULATED_COMMAND each arg (text, \r appended unless
		 * it starts with "x:" which means hex), then poll responses */
		start_listener(&lt);
		for (int a = 2; a < argc; a++) {
			int len = 0;
			if (!strncmp(argv[a], "x:", 2))
				for (const char *p = argv[a] + 2; p[0] && p[1]; p += 2)
					sscanf(p, "%2hhx", &buf[len++]);
			else
				len = snprintf((char *)buf, sizeof(buf), "%s\r",
				    argv[a]);
			ctrl(0x21, 0x00, 0, ifno, buf, len);
			for (int i = 0; i < 10; i++) {
				usleep(100000);
				r = ctrl(0xa1, 0x01, 0, ifno, buf, sizeof(buf));
				if (r > 0) {
					printf("    text: \"");
					for (int j = 0; j < r; j++)
						putchar(buf[j] >= 32 && buf[j] < 127 ?
						    buf[j] : '.');
					printf("\"\n");
				}
			}
		}
		stop_listener(lt);
	} else if (!strcmp(argv[1], "seq") && argc >= 3) {
		/*
		 * seq [-r out.raw:secs] op...
		 *   hN       SET_HOOK_STATE N on interface 0
		 *   e:TEXT   SEND_ENCAPSULATED_COMMAND TEXT\r on if1
		 *   x:HEX    SEND_ENCAPSULATED_COMMAND raw bytes on if1
		 *   r        GET_ENCAPSULATED_RESPONSE on if1
		 *   sMS      sleep MS milliseconds
		 *   c:RT:REQ:VAL:IDX:LEN|HEX  arbitrary control request
		 */
		int a = 2, fd = -1;
		pthread_t rt;
		struct rec_arg ra;
		if (!strcmp(argv[a], "-r")) {
			char *spec = strdup(argv[a + 1]), *c = strrchr(spec, ':');
			*c = 0;
			fd = dsp_open(O_RDONLY | O_NONBLOCK);
			fcntl(fd, F_SETFL, 0);
			ra.fd = fd;
			ra.out = fopen(spec, "w");
			ra.secs = atof(c + 1);
			pthread_create(&rt, NULL, recorder, &ra);
			a += 2;
		}
		start_listener(&lt);
		for (; a < argc; a++) {
			char *op = argv[a];
			int len = 0;
			if (op[0] == 'h')
				ctrl(0x21, SET_HOOK_STATE, atoi(op + 1), 0, NULL, 0);
			else if (op[0] == 's')
				usleep(atoi(op + 1) * 1000);
			else if (op[0] == 'r')
				ctrl(0xa1, 0x01, 0, 1, buf, sizeof(buf));
			else if (!strncmp(op, "e:", 2)) {
				len = snprintf((char *)buf, sizeof(buf), "%s\r", op + 2);
				ctrl(0x21, 0x00, 0, 1, buf, len);
			} else if (!strncmp(op, "x:", 2)) {
				for (const char *p = op + 2; p[0] && p[1]; p += 2)
					sscanf(p, "%2hhx", &buf[len++]);
				ctrl(0x21, 0x00, 0, 1, buf, len);
			} else if (!strncmp(op, "c:", 2)) {
				unsigned v[4];
				char data[200];
				if (sscanf(op + 2, "%x:%x:%x:%x:%199s", &v[0], &v[1],
				    &v[2], &v[3], data) != 5)
					errx(1, "bad op %s", op);
				if (v[0] & 0x80)
					len = strtoul(data, NULL, 0);
				else
					for (const char *p = data; p[0] && p[1]; p += 2)
						sscanf(p, "%2hhx", &buf[len++]);
				ctrl(v[0], v[1], v[2], v[3], buf, len);
			} else
				errx(1, "bad op %s", op);
		}
		if (fd >= 0)
			pthread_join(rt, NULL);
		usleep(300000);
		stop_listener(lt);
		if (fd >= 0) {
			fclose(ra.out);
			close(fd);
		}
	} else if (!strcmp(argv[1], "hook") && argc == 3) {
		ctrl(0x21, SET_HOOK_STATE, atoi(argv[2]), ifno, NULL, 0);
	} else if (!strcmp(argv[1], "raw") && argc == 7) {
		uint8_t rt = strtoul(argv[2], NULL, 0);
		uint16_t len = 0;
		if (rt & 0x80)
			len = strtoul(argv[6], NULL, 0);
		else
			for (const char *p = argv[6]; p[0] && p[1]; p += 2)
				sscanf(p, "%2hhx", &buf[len++]);
		ctrl(rt, strtoul(argv[3], NULL, 0), strtoul(argv[4], NULL, 0),
		    strtoul(argv[5], NULL, 0), buf, len);
	} else if ((!strcmp(argv[1], "offhook") && argc == 4) ||
	    (!strcmp(argv[1], "dial") && argc == 5)) {
		int dial = !strcmp(argv[1], "dial");
		double secs = atof(argv[dial ? 3 : 2]);
		FILE *out = fopen(argv[dial ? 4 : 3], "w");
		if (out == NULL)
			err(1, "out");
		int fd = dsp_open(dial ? O_RDWR : O_RDONLY);
		struct rec_arg ra = { fd, out, secs };
		pthread_t rt;
		start_listener(&lt);
		ctrl(0x21, SET_HOOK_STATE, 1, ifno, NULL, 0);
		pthread_create(&rt, NULL, recorder, &ra);
		if (dial) {
			sleep(2);
			printf("[%7.3f] dialing %s\n", now(), argv[2]);
			for (const char *p = argv[2]; *p; p++)
				dtmf(fd, *p, 100);
		}
		pthread_join(rt, NULL);
		ctrl(0x21, SET_HOOK_STATE, 0, ifno, NULL, 0);
		usleep(300000);
		stop_listener(lt);
		fclose(out);
		close(fd);
	} else
		errx(1, "bad command");

	libusb_release_interface(h, 1);
	libusb_release_interface(h, 0);
	libusb_close(h);
	libusb_exit(NULL);
	return 0;
}
