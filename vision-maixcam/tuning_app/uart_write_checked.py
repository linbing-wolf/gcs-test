"""Write a complete UART frame; report failure instead of false TX success."""
import time


def write_frame(serial, packet, timeout_s=1.0):
    packet = bytes(packet)
    sent = 0
    calls = 0
    started = time.monotonic()
    while sent < len(packet):
        if time.monotonic() - started >= timeout_s:
            raise TimeoutError('UART write timeout: %d/%d bytes' % (sent, len(packet)))
        count = serial.write(packet[sent:])
        calls += 1
        if not isinstance(count, int) or isinstance(count, bool):
            raise IOError('UART write invalid result %r after %d/%d bytes' % (count, sent, len(packet)))
        if count < 0:
            raise IOError('UART write error %d after %d/%d bytes' % (count, sent, len(packet)))
        if count > len(packet) - sent:
            raise IOError('UART write returned impossible length %d' % count)
        if count == 0:
            time.sleep(.001)
        else:
            sent += count
    return sent, calls, (time.monotonic()-started)*1000
