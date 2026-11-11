/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * uam_cid.h - on-hook Caller ID (CLIP) decoder for uamodem
 *
 * Decodes ETSI EN 300 659 / Bellcore GR-30 caller ID sent as 1200 baud FSK
 * between rings:
 *
 *   V.23     (Europe, France):  mark (1) = 1300 Hz, space (0) = 2100 Hz
 *   Bell 202 (North America):   mark (1) = 1200 Hz, space (0) = 2200 Hz
 *
 * Both demodulators run in parallel; the first one to produce a message
 * with a valid checksum wins.  The burst looks like:
 *
 *   channel seizure (0101..., ~250 ms) | mark (~150 ms) | bytes, 8N1, LSB first
 *
 * and the bytes form one message:
 *
 *   type (0x80 MDMF / 0x04 SDMF) | length | body | checksum
 *   (sum of all bytes including the checksum == 0 mod 256)
 *
 * Integer-only so it runs in the kernel.  The includer must provide
 *   static const int16_t uam_sine[256];   (Q15 sine table)
 * before including this file.
 */

#ifndef UAM_CID_H
#define	UAM_CID_H

#define	UAM_CID_RATE		8000
#define	UAM_CID_WIN		7	/* correlator window ~ 1 bit */
#define	UAM_CID_UNIT		3	/* time unit: 1/3 sample */
#define	UAM_CID_BIT		20	/* 8000/1200 samples = 20/3 */
#define	UAM_CID_MINMARK		(10 * UAM_CID_BIT)	/* marks before 1st byte */
#define	UAM_CID_MAXMSG		258
#define	UAM_CID_MINPOWER	(64LL * 64LL)	/* tone power threshold */

struct uam_cid_demod {
	/* tone 0 = mark, tone 1 = space */
	uint32_t	step[2];
	uint32_t	ph[2];
	int32_t		hist[2][2][UAM_CID_WIN];	/* [tone][I/Q][n] */
	int64_t		sum[2][2];
	int		pos;

	/* UART */
	int		state;		/* 0 hunt, 1 in byte */
	int		markrun;	/* time units of continuous mark */
	bool		synced;		/* seen enough mark: bytes allowed */
	int		clock;		/* time units until next bit sample */
	int		bitno;
	uint8_t		shift;

	/* message assembly */
	uint8_t		msg[UAM_CID_MAXMSG];
	int		len;
	int		want;		/* total bytes expected, 0 = unknown */
	unsigned	bytes;		/* UART bytes received (diagnostics) */
};

struct uam_cid {
	struct uam_cid_demod d[2];	/* 0 = V.23, 1 = Bell 202 */
	int32_t		dc;		/* DC estimate << 6 */
	/* result */
	uint8_t		msg[UAM_CID_MAXMSG];
	int		len;
	const char	*standard;
};

static void
uam_cid_demod_reset(struct uam_cid_demod *m, int mark_hz, int space_hz)
{
	memset(m, 0, sizeof(*m));
	m->step[0] = (uint32_t)(((uint64_t)mark_hz << 32) / UAM_CID_RATE);
	m->step[1] = (uint32_t)(((uint64_t)space_hz << 32) / UAM_CID_RATE);
}

static void
uam_cid_reset(struct uam_cid *c)
{
	uam_cid_demod_reset(&c->d[0], 1300, 2100);
	uam_cid_demod_reset(&c->d[1], 1200, 2200);
	c->dc = 0;
	c->len = 0;
	c->standard = NULL;
}

/* Start over on the UART/message level, keep the correlators. */
static void
uam_cid_resync(struct uam_cid_demod *m)
{
	m->state = 0;
	m->markrun = 0;
	m->synced = false;
	m->len = 0;
	m->want = 0;
}

/* A byte came out of the UART.  Returns true when a valid message is in. */
static bool
uam_cid_byte(struct uam_cid_demod *m, uint8_t b)
{
	int i;
	uint8_t sum;

	if (m->len == 0 && b != 0x80 && b != 0x04 && b != 0x82) {
		/* not a message type we know: wait for more mark */
		uam_cid_resync(m);
		return (false);
	}
	m->msg[m->len++] = b;
	if (m->len == 2)
		m->want = b + 3;	/* type + length + body + checksum */
	if (m->want == 0 || m->len < m->want)
		return (m->len >= UAM_CID_MAXMSG ? (uam_cid_resync(m), false) :
		    false);
	for (sum = 0, i = 0; i < m->len; i++)
		sum += m->msg[i];
	if (sum != 0) {
		uam_cid_resync(m);
		return (false);
	}
	return (true);
}

/* One demodulated bit decision per sample: run the async receiver. */
static bool
uam_cid_uart(struct uam_cid_demod *m, int bit, bool carrier)
{
	if (!carrier) {
		if (m->len != 0 || m->state != 0)
			uam_cid_resync(m);
		m->markrun = 0;
		m->synced = false;
		return (false);
	}
	if (m->state == 0) {
		if (bit) {
			m->markrun += UAM_CID_UNIT;
			if (m->markrun >= UAM_CID_MINMARK)
				m->synced = true;
			return (false);
		}
		/* falling edge: start bit, if we are past the seizure */
		m->markrun = 0;
		if (!m->synced)
			return (false);
		m->state = 1;
		m->bitno = 0;
		m->shift = 0;
		/* middle of the first data bit is 1.5 bits from here */
		m->clock = UAM_CID_BIT + UAM_CID_BIT / 2;
		return (false);
	}
	m->clock -= UAM_CID_UNIT;
	if (m->clock > 0)
		return (false);
	m->clock += UAM_CID_BIT;
	if (m->bitno < 8) {
		m->shift |= (bit ? 1 : 0) << m->bitno;
		m->bitno++;
		return (false);
	}
	/* stop bit */
	m->state = 0;
	if (!bit) {
		uam_cid_resync(m);	/* framing error */
		return (false);
	}
	m->bytes++;
	return (uam_cid_byte(m, m->shift));
}

static bool
uam_cid_demod_sample(struct uam_cid_demod *m, int32_t x)
{
	int64_t p[2];
	int t, k;
	int32_t v;

	for (t = 0; t < 2; t++) {
		for (k = 0; k < 2; k++) {
			/* k = 0: cos (sine + 90 deg), k = 1: sin */
			v = x * uam_sine[((m->ph[t] >> 24) + (k ? 0 : 64)) & 0xff]
			    >> 15;
			m->sum[t][k] += v - m->hist[t][k][m->pos];
			m->hist[t][k][m->pos] = v;
		}
		m->ph[t] += m->step[t];
		p[t] = m->sum[t][0] * m->sum[t][0] + m->sum[t][1] * m->sum[t][1];
	}
	if (++m->pos == UAM_CID_WIN)
		m->pos = 0;
	return (uam_cid_uart(m, p[0] > p[1],
	    p[0] + p[1] > UAM_CID_MINPOWER * UAM_CID_WIN * UAM_CID_WIN));
}

/*
 * Feed one 16-bit line sample.  Returns true when a complete, checksummed
 * message is available in c->msg / c->len.
 */
static bool
uam_cid_sample(struct uam_cid *c, int16_t s)
{
	int32_t x;
	int i;

	/* remove DC (the on-hook ADC has a large offset) */
	c->dc += s - (c->dc >> 6);
	x = (int32_t)s - (c->dc >> 6);

	for (i = 0; i < 2; i++) {
		if (uam_cid_demod_sample(&c->d[i], x)) {
			memcpy(c->msg, c->d[i].msg, c->d[i].len);
			c->len = c->d[i].len;
			c->standard = i == 0 ? "V.23" : "Bell 202";
			uam_cid_resync(&c->d[0]);
			uam_cid_resync(&c->d[1]);
			return (true);
		}
	}
	return (false);
}

/*
 * Walk the message.  Calls cb(arg, tag, value) for each field, with the
 * tags used by AT+VCID=1: DATE, TIME, NMBR, NAME, plus MESG for anything
 * else.  "O" means unavailable, "P" private (withheld).
 */
typedef void uam_cid_field_t(void *arg, const char *tag, const char *val);

static void
uam_cid_parse(const uint8_t *msg, int len, uam_cid_field_t *cb, void *arg)
{
	char val[UAM_CID_MAXMSG];
	const uint8_t *p, *end;
	int plen, i;
	uint8_t type;

	end = msg + len - 1;		/* drop checksum */
	p = msg + 2;
	if (msg[0] == 0x04) {
		/* SDMF: MMDDHHMM then the number, no parameter headers */
		if (end - p >= 8) {
			snprintf(val, sizeof(val), "%.4s", p);
			cb(arg, "DATE", val);
			snprintf(val, sizeof(val), "%.4s", p + 4);
			cb(arg, "TIME", val);
			p += 8;
		}
		snprintf(val, sizeof(val), "%.*s", (int)(end - p), p);
		cb(arg, "NMBR", val);
		return;
	}
	while (p + 2 <= end) {
		type = p[0];
		plen = p[1];
		p += 2;
		if (p + plen > end)
			break;
		for (i = 0; i < plen; i++)
			val[i] = (p[i] >= 0x20 && p[i] < 0x7f) ? p[i] : '?';
		val[plen] = '\0';
		switch (type) {
		case 0x01:	/* date and time, MMDDHHMM */
			if (plen >= 8) {
				val[8] = '\0';
				char t[5];

				memcpy(t, val + 4, 4);
				t[4] = '\0';
				val[4] = '\0';
				cb(arg, "DATE", val);
				cb(arg, "TIME", t);
			}
			break;
		case 0x02:	/* calling line identity */
		case 0x03:	/* called line identity */
			cb(arg, type == 0x02 ? "NMBR" : "DDN_NMBR", val);
			break;
		case 0x04:	/* reason for absence of number */
			cb(arg, "NMBR", val);
			break;
		case 0x07:	/* name */
			cb(arg, "NAME", val);
			break;
		case 0x08:	/* reason for absence of name */
			cb(arg, "NAME", val);
			break;
		default:
			break;
		}
		p += plen;
	}
}

#endif /* UAM_CID_H */
