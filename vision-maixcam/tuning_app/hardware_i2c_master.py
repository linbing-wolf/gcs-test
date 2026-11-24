"""Linux I2C6 hardware master. Requested frequency is not measured SCL."""
import fcntl
import os
import time


class HardwareMaster:
    def __init__(self):
        from maix import i2c
        self.clock_owner = i2c.I2C(6, i2c.Mode.MASTER, 10000)
        self.fd = os.open('/dev/i2c-6', os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, 0x0703, 0x42)
        except BaseException:
            os.close(self.fd)
            raise
        self.last_stage = 'idle'
        self.last_index = None
        self.next_pin_check = 0

    def ensure_pins(self):
        now = time.monotonic()
        if now < self.next_pin_check:
            return
        from maix import pinmap
        for pin, function in [('A1','I2C6_SCL'),('A0','I2C6_SDA')]:
            try:
                current = pinmap.get_pin_function(pin)
            except Exception as exc:
                current = 'unreadable: ' + str(exc)
            if current != function:
                pinmap.set_pin_function(pin, function)
                actual = pinmap.get_pin_function(pin)
                if actual != function:
                    raise OSError('I2C pin restore failed: '+pin+' '+str(actual))
                print('[I2C-PIN-RESTORE] %s %s -> %s' % (pin,current,actual), flush=True)
        self.next_pin_check = now + 1

    def write(self, packet):
        self.ensure_pins()
        self.last_stage = 'hardware_write'
        return os.write(self.fd, packet)

    def read(self, length):
        self.ensure_pins()
        self.last_stage = 'hardware_read'
        return os.read(self.fd, length)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
