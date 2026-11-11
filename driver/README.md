# uamodem — Apple USB Modem (05ac:1401) driver for FreeBSD 15.1

## Why this is not a CDC-ACM driver

The original brief (`../todo.txt`) describes a Conexant CDC-ACM modem with
bulk data endpoints and an AT interpreter in firmware. Probing this unit
(`../NOTES.md`) shows otherwise. It is a Motorola SM56 **softmodem** with a
Silicon Labs Si3050/Si3018 DAA. It has:

| if | function | used by uamodem for |
|----|----------|---------------------|
| 0 | CDC Telephone Control Model, interrupt IN 0x81 | hook switch, RING / line-state notifications |
| 1 | vendor, CDC encapsulated commands | DAA registers, firmware version, country |
| 2 | audio control | claimed (kept away from uaudio) |
| 3 | audio stream, iso IN 0x82, 8 kHz | dial-tone detection |
| 4 | audio stream, iso OUT 0x03, 8 kHz | DTMF tone dialling |
| 5 | DFU | never touched |

It has no bulk endpoints and no AT command set, so umodem(4) and a
textbook CDC-ACM driver both have nothing to bind to. uamodem implements
the AT layer in the driver and exposes it as an ordinary serial port.

## What works / what doesn't

Works:
- `/dev/cuaU<n>` (and `/dev/ttyU<n>`) via ucom(4)
- `ATH0/ATH1`, `ATA`: on-hook / off-hook
- `ATDT…` dials with DTMF tones synthesised in the driver. `ATDP…` dials with
  pulses (by toggling the DAA off-hook bit). `,` pauses (S8), `W` waits for dial tone, `!` does a
  hook flash, and a trailing `;` is accepted.
- Dial-tone detection (`ATX4`, the default) → `NO DIALTONE`
- `RING` reporting; auto-answer with `ATS0=n`
- Dropping DTR or closing the port hangs up (`AT&D2` behaviour)
- `AT+GCI=xx` / `AT+GCI?` / `AT+GCI=?`: country selection (T.35 codes). This
  programs both the firmware country feature and Si3050 registers 16/26/30/31.
- `ATI0`–`ATI3` identification; `ATI4` shows live line diagnostics (jack
  state, line voltage, loop current, audio level)
- `ATE ATQ ATV ATX ATZ AT&F ATSn=v ATSn? A/`. The commands `ATL ATM AT&C AT&K
  AT\N AT%C AT+MS` are accepted and ignored.

Doesn't (by design):
- **No data, fax or V.92 connection.** A softmodem's modulation runs on the
  host CPU as a full DSP stack. That doesn't belong in a kernel driver. A dial
  sets up a *voice* call: the line is seized, the number is dialled, the
  result is `OK`, and the call stays up until `ATH`, DTR drop or close. You
  will never see `CONNECT`.
- Termios speed/parity are accepted and ignored. There's no UART, so
  there's nothing for `SET_LINE_CODING` to configure.
- While uamodem owns the modem, its `pcm` device (from uaudio) is gone.
  Set `hw.usb.uamodem.audio=0` to leave audio to uaudio. You then only
  get pulse dialling and no dial-tone detection.

## Build

Requires the kernel sources in `/usr/src/sys` (FreeBSD 15.1).

```sh
cd ~/usbmodem/driver
make                 # -> uamodem.ko
```

## Load

```sh
sudo kldload ./uamodem.ko        # pulls in ucom.ko automatically
```

Then **unplug and replug the modem**. If uaudio already owns the audio
interfaces, uamodem still attaches, but only for pulse dialling. A replug
lets uamodem claim the interfaces before uaudio does. Check:

```sh
dmesg | tail
# uamodem0 on uhub0
# uamodem0: <Motorola, Inc. Apple USB Modem, ...> on usbus0
# uamodem0: firmware 2.02, country France, tone+pulse dialling
ls /dev/cuaU*
sysctl dev.uamodem.0
```

To load at boot (so it always wins over uaudio):

```sh
sudo make install                        # -> /boot/modules/uamodem.ko
echo 'uamodem_load="YES"' | sudo tee -a /boot/loader.conf
echo 'hw.usb.uamodem.country=0x3d' | sudo tee -a /boot/loader.conf   # France, optional
```

Unload: `sudo kldunload uamodem`. The line is put on-hook first. If a
process still has the port open, kldunload waits (printing "ucom: Waiting
for all detached TTY devices...") until it is closed.

### Tunables / sysctls

| name | meaning |
|------|---------|
| `hw.usb.uamodem.audio` (tunable, default 1) | claim the audio interfaces |
| `hw.usb.uamodem.country` (tunable, default -1) | T.35 country to apply at attach |
| `hw.usb.uamodem.debug` | debug level (kernel with `USB_DEBUG`) |
| `dev.uamodem.N.offhook`, `.line_state`, `.firmware`, `.country` | status |
| `dev.uamodem.N.last_rms` | AC level of the line audio (last 100 ms) |
| `dev.uamodem.N.dialtone_threshold` (rw, default 150) | dial-tone level |

## Test with a terminal

`cu` is in base. `minicom` is in ports (`pkg install minicom`). Your user
needs access to the device: be in group `dialer`, or use sudo.

```sh
sudo cu -l /dev/cuaU0 -s 115200      # speed is irrelevant
```

```
AT
OK
ATI1
uamodem 1.0 (FreeBSD), SM56 + Si3050/Si3018
OK
ATI3
Firmware 2.02
OK
AT+GCI=3D            <- France (B5 = USA, B4 = UK, 04 = Germany)
OK
AT+GCI?
+GCI: 3D
OK
ATI4                 <- line diagnostics, on-hook
Hook: on
Jack: unknown (line state -1)
Line voltage: -48 V  <- a healthy line shows its DC battery voltage here
Loop current: 0.0 mA (LCS 0)
OK
ATDT0123456789;      <- dial (voice), phone at the far end should ring
OK
ATH
OK
```

Exit `cu` with `~.`. With minicom: `minicom -D /dev/cuaU0`. Clear the init
string or leave it; `ATZ` and the usual init commands are accepted.

What to expect on a broken line (see `../NOTES.md`): `ATDT…` returns
`NO DIALTONE`. `ATI4` while off-hook (`ATH1` then `ATI4`) shows line
state 4 ("no line") and 0 mA loop current.

## Code map (`uamodem.c`)

- probe/attach/detach: binds to interface 0 of 05ac:1401. It parses the
  configuration to find the vendor and audio-streaming interfaces, claims
  them, selects the iso alternate settings and sets 8 kHz.
- USB callbacks: interrupt (notifications), iso OUT (DTMF generator),
  iso IN (level statistics).
- worker thread: owns the line. It runs the AT interpreter and does every
  blocking control request.
- ucom callbacks: DTR → hang up, open/close, and "start write" passes
  typed characters to the worker.

Locking: one mutex (`sc_mtx`) is shared by ucom/tty, the USB transfers
and the worker. On detach the worker is stopped first (it hangs up on its
way out), then ucom, then the transfers. The softc is freed through the
ucom refcount, so an open port can't outlive it.

## Firmware quirks the driver works around

These were found while testing on the real device (2026-11-10):

- `SET_HOOK_STATE` sometimes STALLs a request for the state the firmware is
  already in. The driver treats a STALL as success.
- After going off-hook the firmware runs a line check (LINE_STATE 7, then
  4 or 5, about 430 ms). An on-hook request during that window can be
  ACKed but ignored, leaving the line seized. The driver keeps the line
  off-hook for at least 500 ms. After every on-hook it reads the Si3050
  off-hook bit (register 5, bit 0) and retries until the bit is clear,
  forcing it clear as a last resort. `ATI4` shows the bit.
- For the same reason, dial pulses can't go through `SET_HOOK_STATE` (a 60 ms
  break 40 ms after a make would be ignored). Pulses and hook flash toggle
  register 5 bit 0 directly while the firmware keeps the line seized.
- RING_DETECT arrives for every cycle of the ringing voltage, and also for
  50 Hz mains hum on a bad line, tens of times a second. The driver reports
  one `RING` per burst (after 1 s of silence).
- The firmware forgets the country on reset. Set `hw.usb.uamodem.country`
  if you aren't in the US.

## Test results (this unit, 2026-11-10)

| test | result |
|------|--------|
| attach, firmware version, country read/set (`AT+GCI`) | OK |
| `ATH1`/`ATH0`, DAA OH bit follows | OK |
| notifications (line state, RING) | OK |
| iso capture | 8000 samples/s |
| DTMF out (DAA digital loopback) | AC RMS ≈ 4500 in loopback; 7600 samples for 10 digits at 95 ms |
| `ATX3ATDT0626852858;` blind tone dial | OK, 3.9 s |
| `ATX3ATDP0626852858;` blind pulse dial | OK, 15.8 s |
| `ATDT0626852858;` with dial-tone check | `NO DIALTONE` (line fault, below) |
| close port while off-hook → hang-up | OK |
| `kldunload` with port open and line seized | waits for close, line released, no panic |
| fallback when uaudio owns audio | pulse-only mode, OK |

The call never reaches the network. This unit's line side is faulty (see
`../NOTES.md`): as soon as it goes off-hook the firmware reports LINE_STATE
4 ("RJ11 unplugged"), loop current is 0 mA, and the ADC is pinned at a DC
level of −31703. The driver reports this correctly. It needs working
hardware for a call to go through.
