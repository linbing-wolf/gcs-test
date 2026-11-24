"""I2C6 master test mailbox at 7-bit address 0x42.

Every transaction is 32 bytes: valid length (0..31), stream bytes, zero pad.
Slave read serves command bytes; master write carries vision replies.
Original AA55/LEN/payload/CRC/0D frames may span several transactions.
Slave must return a zero-length packet when its command queue is empty.
"""
from collections import deque
import errno
import time
import json
import threading
import queue
import traceback
from pathlib import Path
from gpio_i2c_master import GPIOMaster


class SynchronousTransport:
    def __init__(self, dev, baud):
        self.bus = GPIOMaster()
        import config
        self.static_diag = bool(getattr(config, 'I2C_STATIC_INPUT_DIAG', False))
        self.static_next = 0
        self.queue = deque()
        self.next_poll = 0
        self.next_error = 0
        self.started = time.monotonic()
        self.next_summary = self.started + 10
        self.counts = dict(tx_attempt=0, tx_ok=0, tx_error=0,
                           rx_attempt=0, rx_ok=0, rx_error=0, rx_empty=0,
                           malformed=0, error_streak=0)
        self.queued_at = deque()
        self._diag('init', backend='gpio-open-drain', scl='A1', sda='A0', address_7bit='0x42', address_write='0x84',
                   address_read='0x85', max_clock_hz=10000, min_half_period_us=50, packet_bytes=32,
                   poll_ms=10, error_backoff_ms=100,
                   note='clock may be slower under Linux; no register prefix; hardware I2C6 is bypassed')
        from maix import pinmap
        for pin in ('A1', 'A0'):
            try:
                self._diag('pinmap', pin=pin, current=pinmap.get_pin_function(pin),
                           available=pinmap.get_pin_functions(pin))
            except Exception as exc:
                self._diag('pinmap_read_error', pin=pin, error=str(exc))
        for name, path in [('adapter', '/sys/class/i2c-adapter/i2c-6/name'),
                           ('dt_clock', '/sys/class/i2c-adapter/i2c-6/of_node/clock-frequency')]:
            try:
                raw = Path(path).read_bytes()
                self._diag(name, value=int.from_bytes(raw, 'big') if name == 'dt_clock' else raw.decode().strip(),
                           note='DT boot value, not runtime clock readback' if name == 'dt_clock' else '')
            except OSError as exc:
                self._diag('metadata_error', field=name, error=str(exc))
        print('[I2C] GPIO master A1/A0 max_clock=10000Hz address=0x42 packet=32; TX logs mean queued, not delivered', flush=True)
        if self.static_diag:
            self._diag('static_mode', note='I2C transfers paused; A1/A0 stay IN/PULL_UP; no clock pulses')

    def _diag(self, event, **fields):
        fields.update(event=event, elapsed_ms=round((time.monotonic()-self.started)*1000, 2))
        print('[I2C-DIAG] ' + json.dumps(fields, ensure_ascii=False), flush=True)

    def write(self, data):
        data = bytes(data)
        chunks = [data[i:i+31] for i in range(0, len(data), 31)]
        if len(self.queue) + len(chunks) > 128:
            raise RuntimeError('I2C TX queue full; peer not consuming replies')
        for chunk in chunks:
            self.queue.append(bytes([len(chunk)]) + chunk + bytes(31-len(chunk)))
            self.queued_at.append(time.monotonic())
        self._diag('enqueue', stream_hex=data.hex(' '), packets=len(chunks), queued=len(self.queue))
        return len(data)

    def read(self, *, allow_rx=True, allow_tx=True):
        now = time.monotonic()
        if getattr(self, 'static_diag', False):
            if now >= self.static_next:
                self.static_next = now + 1
                self._diag('static_inputs', scl=self.bus.scl.value(), sda=self.bus.sda.value(),
                           scl_mode=str(self.bus.scl.io.get_mode()), sda_mode=str(self.bus.sda.io.get_mode()),
                           scl_pull=str(self.bus.scl.io.get_pull()), sda_pull=str(self.bus.sda.io.get_pull()),
                           queued=len(self.queue), note='fixed input mode; no I2C transaction attempted')
            return b''
        if now < self.next_poll:
            return b''
        if now >= self.next_summary:
            self._diag('summary', **self.counts, queued=len(self.queue),
                       oldest_queued_ms=round((now-self.queued_at[0])*1000, 1) if self.queued_at else 0)
            self.next_summary = now + 10
        self.next_poll = now + .01
        phase = 'write' if self.queue and allow_tx else 'read'
        tx_delivered = False
        started = time.monotonic()
        try:
            if phase == 'write':
                self.counts['tx_attempt'] += 1
                if self.counts['tx_attempt'] <= 3:
                    self._diag('write_attempt', attempt=self.counts['tx_attempt'],
                               bytes=32, packet_hex=self.queue[0].hex(' '))
                if self.bus.write(self.queue[0]) != 32:
                    self.counts['malformed'] += 1
                    self._diag('short_write', packet_hex=self.queue[0].hex(' '))
                    raise OSError(errno.EIO, 'I2C short write')
                self._diag('write_ok', packet_hex=self.queue[0].hex(' '),
                           duration_ms=round((time.monotonic()-started)*1000, 3))
                self.queue.popleft()
                self.queued_at.popleft()
                self.counts['tx_ok'] += 1
                tx_delivered = True
                phase = 'read'
                started = time.monotonic()
                print('[I2C] TX transaction delivered', flush=True)
                if getattr(self, 'separate_write', False):
                    return b''
            if not allow_rx:
                return b''
            self.counts['rx_attempt'] += 1
            packet = self.bus.read(32)
            if len(packet) != 32 or packet[0] > 31:
                self.counts['malformed'] += 1
                self._diag('malformed_read', actual_bytes=len(packet), packet_hex=packet.hex(' '))
                raise OSError(errno.EPROTO, 'I2C malformed mailbox packet')
            self.counts['rx_ok'] += 1
            if self.counts['error_streak']:
                self._diag('recovered', prior_errors=self.counts['error_streak'])
            self.counts['error_streak'] = 0
            if packet[0]:
                self._diag('read_data', valid_bytes=packet[0], packet_hex=packet.hex(' '),
                           duration_ms=round((time.monotonic()-started)*1000, 3))
            else:
                self.counts['rx_empty'] += 1
            return packet[1:1+packet[0]]
        except OSError as exc:
            self.counts['tx_error' if phase == 'write' else 'rx_error'] += 1
            self.counts['error_streak'] += 1
            if now >= self.next_error:
                self._diag('io_error', phase=phase, errno=exc.errno, error=str(exc),
                           tx_delivered_this_poll=tx_delivered,
                           duration_ms=round((time.monotonic()-started)*1000, 3),
                           queued=len(self.queue), **self.counts,
                           packet_hex=self.queue[0].hex(' ') if phase == 'write' and self.queue else None,
                           nack_stage=self.bus.last_stage, byte_index=self.bus.last_index,
                           note='GPIO backend samples individual ACK bits; check error text for NACK stage')
                print('[I2C] peer unavailable/bus error: %s; queued=%d' % (exc, len(self.queue)), flush=True)
                self.next_error = now + 5
            self.next_poll = now + .1
            if phase == 'write' and allow_rx:
                # A peer may be waiting for its command to be read before it
                # accepts replies. A failed write must not starve that read.
                self.counts['rx_attempt'] += 1
                try:
                    packet = self.bus.read(32)
                    if len(packet) != 32 or packet[0] > 31:
                        self.counts['malformed'] += 1
                        self.counts['rx_error'] += 1
                        self._diag('malformed_fallback_read', actual_bytes=len(packet), packet_hex=packet.hex(' '))
                        return b''
                    self.counts['rx_ok'] += 1
                    if packet[0]:
                        self._diag('read_after_write_failure', valid_bytes=packet[0], packet_hex=packet.hex(' '), queued=len(self.queue))
                    else:
                        self.counts['rx_empty'] += 1
                    return packet[1:1+packet[0]]
                except OSError as read_error:
                    self.counts['rx_error'] += 1
                    if now >= getattr(self, 'next_fallback_error', 0):
                        self.next_fallback_error = now + 5
                        self._diag('fallback_read_error', errno=read_error.errno, error=str(read_error), queued=len(self.queue))
            return b''

    def close(self):
        if self.bus is not None:
            self._diag('close', **self.counts, queued=len(self.queue))
            self.bus.close()
            self.bus = None


class I2CTransport:
    """Single GPIO owner; application reads/writes only bounded memory queues."""
    frame_timeout_ms = 600

    def __init__(self, dev, baud):
        self._tx = queue.Queue(maxsize=128)
        self._rx = queue.Queue(maxsize=256)
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._ready = threading.Event()
        self._error = None
        self._closed = False
        self._thread = threading.Thread(target=self._run, args=(dev, baud), name='software-i2c', daemon=True)
        self._thread.start()
        if not self._ready.wait(10):
            self._stop.set()
            self._wake.set()
            raise RuntimeError('I2C worker initialization timed out')
        self._check()

    def _check(self):
        if self._error is not None:
            raise RuntimeError('I2C worker failed: '+str(self._error)) from self._error
        if self._closed:
            raise RuntimeError('I2C transport is closed')

    def write(self, data):
        self._check()
        data = bytes(data)
        if len(data) > 31*128:
            raise ValueError('I2C message exceeds bounded queue capacity')
        try:
            self._tx.put_nowait(data)
        except queue.Full as exc:
            raise RuntimeError('I2C application TX queue full') from exc
        self._wake.set()
        return len(data)

    def read(self):
        self._check()
        try:
            data = self._rx.get_nowait()
        except queue.Empty:
            return b''
        self._wake.set()
        return data

    def _run(self, dev, baud):
        link = None
        try:
            link = SynchronousTransport(dev, baud)
            link.separate_write = True
            self._ready.set()
            print('[I2C-ASYNC] worker enabled; complete TX frames stay contiguous; scheduled RX between frames', flush=True)
            next_read = 0
            retry_at = 0
            frame_in_progress = False
            while not self._stop.is_set():
                self._wake.clear()
                # Only this thread touches link.queue, GPIO, and counters.
                if not link.queue:
                    frame_in_progress = False
                    try:
                        link.write(self._tx.get_nowait())
                    except queue.Empty:
                        pass
                now = time.monotonic()
                can_read = not self._rx.full()
                read_due = can_read and now >= next_read
                write_due = bool(link.queue) and now >= retry_at
                # Application writes contain one complete AA55 frame. Once its
                # first mailbox is sent, finish its remaining mailboxes before
                # a normal command read: a GPIO read can itself take >100ms.
                write_poll = write_due and (frame_in_progress or not read_due)
                if not read_due and not write_due:
                    deadlines = [now + .2]
                    if can_read:
                        deadlines.append(next_read)
                    if link.queue:
                        deadlines.append(retry_at)
                    self._wake.wait(max(0, min(deadlines)-now))
                    continue
                link.next_poll = 0
                failures = link.counts['tx_error'] + link.counts['rx_error']
                queued_before = len(link.queue)
                data = link.read(allow_rx=can_read, allow_tx=write_poll)
                completed = time.monotonic()
                failed = link.counts['tx_error'] + link.counts['rx_error'] > failures
                if len(link.queue) < queued_before:
                    frame_in_progress = bool(link.queue)
                if write_poll:
                    retry_at = completed + (.1 if failed else 0)
                if (read_due and not write_poll) or data:
                    next_read = completed + (.1 if failed else .02 if data else .05)
                if data:
                    # Only this worker adds RX data. If the queue fills, leave
                    # further bytes in the peer mailbox until the app drains it.
                    self._rx.put_nowait(data)
        except Exception as exc:
            self._error = exc
            traceback.print_exc()
        finally:
            self._ready.set()
            if link is not None:
                try:
                    link.close()
                except Exception as exc:
                    self._error = exc
                    traceback.print_exc()

    def close(self):
        self._closed = True
        self._stop.set()
        self._wake.set()
        self._thread.join(5)
        if self._thread.is_alive():
            raise RuntimeError('I2C worker did not stop; GPIO still owned by worker')
        if self._error is not None:
            raise RuntimeError('I2C worker failed during lifetime') from self._error
