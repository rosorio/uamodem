# Apple USB Modem on FreeBSD — reverse-engineered by claude

This repo is part of my experiments, a toy not supposed to become official.

Since a while I have this [Apple Winmodem](https://en.wikipedia.org/wiki/Apple_USB_Modem) , a mistaken purchase when I was looking for a modem who handles Caller ID(CID) for a free time project.

I don't wanna spend to much time on it, so this is an excellent playground to see
if we can revive the device.
The session cost less than 5 dollars according with the `/cost` prompt, and leaves
me with a functional audio modem plus 2 sample applications.

The funniest is answerplay who convert my POTS in the "Santa Claus's line".
Each caller is treated to a randomly selected Christmas story.

```
answerplay <msg1.raw> <msg2.raw> ...
```

You can create the audio files with the sox tool

```
sox sound.wav -r 8000 -c 1 -b 16 -e signed-integer -t raw sound.raw highpass 300 lowpass 3400 norm -3

```
