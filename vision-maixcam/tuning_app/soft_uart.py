"""Experimental Linux GPIO UART. Independent process, 8N1, half duplex.

Timing is best effort: Linux scheduling can corrupt bits at 115200 baud.
Wire A1 TX -> peer RX; A0 RX <- peer TX; common 3.3V GND.
"""
import os
import socket
import subprocess
import sys
import time


class SoftUART:
    def __init__(self, dev, baud):
        self.sock, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.pending = bytearray()
        self.proc = subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), str(child.fileno()), str(baud)],
            pass_fds=(child.fileno(),))
        child.close()
        self.sock.settimeout(15)
        try:
            msg = self.sock.recv(4096)
            if msg != b'OK':
                raise RuntimeError('SoftUART startup: ' + msg.decode(errors='replace'))
        except BaseException:
            self.close()
            raise
        self.sock.setblocking(False)

    def _message(self, msg):
        if not msg:
            raise RuntimeError('SoftUART worker disconnected')
        if msg[:1] == b'R':
            self.pending.extend(msg[1:])
            if len(self.pending) > 65536:
                raise RuntimeError('SoftUART RX overflow')
        elif msg[:1] == b'E':
            raise RuntimeError(msg[1:].decode(errors='replace'))

    def read(self):
        while True:
            try:
                msg = self.sock.recv(4096)
            except BlockingIOError:
                break
            self._message(msg)
        data = bytes(self.pending)
        self.pending.clear()
        return data

    def write(self, data):
        data = bytes(data)
        if len(data) > 4095:
            raise ValueError('SoftUART packet too large')
        self.sock.settimeout(3 + len(data) * 12 / 115200)
        try:
            self.sock.sendall(b'T' + data)
            while True:
                msg = self.sock.recv(4096)
                if msg == b'A':
                    return len(data)
                self._message(msg)
        finally:
            self.sock.setblocking(False)

    def close(self):
        if getattr(self, 'proc', None) is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
            self.proc = None
        if getattr(self, 'sock', None) is not None:
            self.sock.close()


def worker(fd, baud):
    channel = socket.socket(fileno=fd)
    tx = None
    try:
        from maix import gpio, pinmap
        for pin in ('A1', 'A0'):
            func = 'GPIO' + pin
            if func not in pinmap.get_pin_functions(pin):
                raise RuntimeError('GPIO unavailable: ' + pin)
            pinmap.set_pin_function(pin, func)
            if pinmap.get_pin_function(pin) != func:
                raise RuntimeError('GPIO mapping failed: ' + pin)
        tx = gpio.GPIO('GPIOA1', gpio.Mode.OUT)
        rx = gpio.GPIO('GPIOA0', gpio.Mode.IN)
        tx.value(1)
        channel.send(b'OK')
        channel.setblocking(False)
        bit = 1_000_000_000 / baud
        clock = time.perf_counter_ns

        def wait(deadline):
            while clock() < deadline:
                pass

        previous = rx.value()
        while True:
            try:
                command = channel.recv(4096)
            except BlockingIOError:
                command = None
            if command == b'':
                break
            if command is not None:
                if command[:1] != b'T':
                    raise RuntimeError('Unknown SoftUART command')
                for byte in command[1:]:
                    start = clock()
                    tx.value(0)
                    for index in range(8):
                        wait(start + (index + 1) * bit)
                        tx.value((byte >> index) & 1)
                    wait(start + 9 * bit)
                    tx.value(1)
                    wait(start + 10 * bit)
                channel.send(b'A')
                previous = rx.value()
                continue
            level = rx.value()
            if previous and not level:
                start = clock()
                wait(start + bit * .5)
                if rx.value():
                    previous = 1
                    continue
                value = 0
                late = False
                for index in range(8):
                    deadline = start + (1.5 + index) * bit
                    wait(deadline)
                    if clock() - deadline > bit * .4:
                        late = True
                    value |= int(bool(rx.value())) << index
                wait(start + 9.5 * bit)
                stop = rx.value()
                if stop and not late:
                    channel.send(b'R' + bytes([value]))
                elif not late:
                    print('[SoftUART] RX framing error', flush=True)
                else:
                    print('[SoftUART] RX scheduling overrun; byte dropped', flush=True)
                previous = stop
            else:
                previous = level
    except Exception as exc:
        channel.setblocking(True)
        channel.send(b'E' + str(exc).encode())
        raise
    finally:
        if tx is not None:
            tx.value(1)
        channel.close()


if __name__ == '__main__':
    worker(int(sys.argv[1]), int(sys.argv[2]))
