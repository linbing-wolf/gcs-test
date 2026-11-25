"""Low-speed open-drain I2C master; each SCL phase is at least 50us.

IN releases the line; the peer must provide pull-up. OUT drives zero.
Never drive high. Linux scheduling may lengthen phases (below 10kHz).
"""
import time
import sys


class OpenDrainLine:
    def __init__(self, pin):
        from maix import gpio, pinmap
        self.gpio = gpio
        self.pinmap = pinmap
        self.pin = pin
        function = 'GPIO' + pin
        pinmap.set_pin_function(pin, function)
        if pinmap.get_pin_function(pin) != function:
            raise RuntimeError('GPIO mapping failed: ' + pin)
        self.io = gpio.GPIO(function, gpio.Mode.IN, gpio.Pull.PULL_UP)
        self.released = True

    def ensure_mapping(self):
        function = 'GPIO' + self.pin
        try:
            current = self.pinmap.get_pin_function(self.pin)
        except Exception as exc:
            # Camera mux value 0 is not represented by the pinmap API.
            current = 'unreadable: ' + str(exc)
        if current != function:
            self.pinmap.set_pin_function(self.pin, function)
            if self.pinmap.get_pin_function(self.pin) != function:
                raise RuntimeError('GPIO mapping restore failed: ' + self.pin)
            print('[I2C-PIN-RESTORE] %s %s -> %s' % (self.pin, current, function), flush=True)

    def release(self):
        if not self.released:
            from maix import err
            err.check_raise(self.io.reset(self.gpio.Mode.IN, self.gpio.Pull.PULL_UP), self.pin+' release with internal pull-up failed')
            self.released = True

    def low(self):
        if self.released:
            from maix import err
            err.check_raise(self.io.reset(self.gpio.Mode.OUT, self.gpio.Pull.PULL_NONE), self.pin+' low failed')
            self.released = False

    def value(self):
        value = self.io.value()
        if value not in (0, 1):
            raise OSError(5, 'GPIO read failed: %s value=%r' % (self.pin, value))
        return value


class GPIOMaster:
    def __init__(self, line_factory=OpenDrainLine, address=0x42):
        self.scl = line_factory('A1')
        self.sda = line_factory('A0')
        self.address = address
        self.last_stage = 'idle'
        self.last_index = None
        self.real_lines = line_factory is OpenDrainLine
        print('[I2C-GPIO] open-drain A1=SCL A0=SDA max_clock=10000Hz; peer pull-up required; local bias unverified', flush=True)
        if line_factory is OpenDrainLine:
            print('[I2C-GPIO] idle readback SCL=%d SDA=%d; configured pull SCL=%s SDA=%s' %
                  (self.scl.value(), self.sda.value(), self.scl.io.get_pull(), self.sda.io.get_pull()), flush=True)

    def delay(self):
        deadline = time.perf_counter_ns() + 50000
        while time.perf_counter_ns() < deadline:
            pass

    def clock_high(self):
        self.scl.release()
        until = time.monotonic() + .02
        while not self.scl.value():
            if time.monotonic() >= until:
                raise OSError(110, 'SCL did not rise within 20ms; stage=%s SDA=%s (stretch/wiring/scheduling)' % (self.last_stage,self.sda.value()))
        self.delay()

    def start(self):
        if self.real_lines:
            self.scl.ensure_mapping()
            self.sda.ensure_mapping()
        self.sda.release()
        self.clock_high()
        if not self.sda.value():
            raise OSError(16, 'SDA held low before START; check pull-up/wiring/peer state')
        self.sda.low()
        self.delay()
        self.scl.low()
        self.delay()

    def stop(self):
        self.scl.low()
        self.sda.low()
        self.delay()
        try:
            self.clock_high()
        finally:
            self.sda.release()
            self.scl.release()
        self.delay()

    def send_byte(self, value, stage, index=None):
        self.last_stage, self.last_index = stage, index
        for shift in range(7, -1, -1):
            if (value >> shift) & 1:
                self.sda.release()
            else:
                self.sda.low()
            self.delay()
            self.clock_high()
            self.scl.low()
            self.delay()
        self.sda.release()
        self.delay()
        self.clock_high()
        ack = not self.sda.value()
        self.scl.low()
        self.delay()
        if not ack:
            raise OSError(121, 'NACK stage=%s byte_index=%s sent=0x%02X address_7bit=0x%02X' % (stage,index,value,self.address))

    def finish(self):
        primary = sys.exc_info()[1]
        try:
            self.stop()
        except Exception as stop_error:
            if primary is not None:
                raise OSError('I2C transaction failed: %s; STOP also failed: %s' % (primary,stop_error)) from primary
            raise

    def receive_byte(self, last):
        self.sda.release()
        value = 0
        for _ in range(8):
            self.clock_high()
            value = (value << 1) | int(bool(self.sda.value()))
            self.scl.low()
            self.delay()
        if last:
            self.sda.release()
        else:
            self.sda.low()
        self.delay()
        self.clock_high()
        self.scl.low()
        self.delay()
        self.sda.release()
        return value

    def write(self, packet):
        try:
            self.start()
            self.send_byte(self.address << 1, 'address_write')
            for index,value in enumerate(packet):
                self.send_byte(value, 'data_write', index)
        finally:
            self.finish()
        return len(packet)

    def read(self, length):
        try:
            self.start()
            self.send_byte((self.address << 1) | 1, 'address_read')
            self.last_stage = 'data_read'
            packet = bytes(self.receive_byte(i == length-1) for i in range(length))
        finally:
            self.finish()
        return packet

    def close(self):
        self.sda.release()
        self.scl.release()
