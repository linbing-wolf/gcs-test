from collections import deque
import unittest
from unittest.mock import patch
from gpio_i2c_master import GPIOMaster, OpenDrainLine
from unittest.mock import Mock


class GPIOI2CTests(unittest.TestCase):
    def test_camera_unknown_mux_is_restored(self):
        line = OpenDrainLine.__new__(OpenDrainLine)
        line.pin = 'A1'
        line.pinmap = Mock()
        line.pinmap.get_pin_function.side_effect = [RuntimeError('unknown value: 0'), 'GPIOA1']
        line.ensure_mapping()
        line.pinmap.set_pin_function.assert_called_once_with('A1', 'GPIOA1')

    def test_gpio_negative_read_is_error(self):
        line = OpenDrainLine.__new__(OpenDrainLine)
        line.pin = 'A0'
        line.io = Mock()
        line.io.value.return_value = -5
        with self.assertRaisesRegex(OSError, 'GPIO read failed'):
            line.value()

    def setup_bus(self, sda_values=(), scl_value=1):
        lines = {}
        clocks = []
        class Line:
            def __init__(self, name):
                self.name = name
                self.released = True
                self.samples = deque(sda_values)
                lines[name] = self
            def release(self):
                self.released = True
                if self.name == 'A1':
                    clocks.append(lines['A0'].released)
            def low(self):
                self.released = False
            def value(self):
                if self.name == 'A1':
                    return scl_value
                return self.samples.popleft() if self.samples else 1
        bus = GPIOMaster(line_factory=Line)
        bus.delay = lambda: None
        return bus, lines, clocks

    def test_address_bits_msb_first_and_release_for_ack(self):
        bus, _, clocks = self.setup_bus([0])
        bus.send_byte(0x84, 'address_write')
        self.assertEqual(clocks, [True,False,False,False,False,True,False,False,True])

    def test_address_nack_and_stop_releases_bus(self):
        bus, lines, _ = self.setup_bus([1,1])
        with self.assertRaisesRegex(OSError, 'NACK stage=address_write'):
            bus.write(b'x')
        self.assertTrue(all(x.released for x in lines.values()))

    def test_data_nack_has_byte_index(self):
        bus, lines, _ = self.setup_bus([1,0,0,1])
        with self.assertRaisesRegex(OSError, 'NACK stage=data_write byte_index=1'):
            bus.write(b'xy')
        self.assertTrue(all(x.released for x in lines.values()))

    def test_read_byte_last_sends_nack(self):
        bus, _, clocks = self.setup_bus([1,0,1,0,0,1,0,1])
        self.assertEqual(bus.receive_byte(last=True), 0xA5)
        self.assertTrue(clocks[-1])

    def test_read_byte_nonlast_sends_ack(self):
        bus, _, clocks = self.setup_bus([0]*8)
        self.assertEqual(bus.receive_byte(last=False), 0)
        self.assertFalse(clocks[-1])

    def test_clock_stuck_has_bounded_timeout(self):
        bus, _, _ = self.setup_bus(scl_value=0)
        with patch('gpio_i2c_master.time.monotonic', side_effect=[0,.03]):
            with self.assertRaisesRegex(OSError, 'SCL did not rise'):
                bus.clock_high()

    def test_sda_busy_rejected_and_released(self):
        bus, lines, _ = self.setup_bus([0])
        with self.assertRaisesRegex(OSError, 'SDA held low'):
            bus.write(b'x')
        self.assertTrue(all(x.released for x in lines.values()))


if __name__ == '__main__':
    unittest.main()
