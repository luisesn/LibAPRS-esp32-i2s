#ifndef PROTOCOL_HDLC_H
#define PROTOCOL_HDLC_H

// HDLC framing constants used by the AX.25 layer.
// HDLC_FLAG  (0x7E): frame delimiter transmitted before and after each packet.
// HDLC_RESET (0x7F): abort sequence; also matches an all-ones idle line.
// AX25_ESC   (0x1B): LibAPRS escape byte — inserted before any HDLC_FLAG or
//   HDLC_RESET byte that appears inside the data payload, so the FIFO layer
//   can distinguish data bytes from framing control characters.

#define HDLC_FLAG  0x7E
#define HDLC_RESET 0x7F
#define AX25_ESC   0x1B

#endif
