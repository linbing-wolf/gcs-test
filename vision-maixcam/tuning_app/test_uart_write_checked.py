import unittest
from unittest.mock import Mock
from uart_write_checked import write_frame


class UARTWriteTests(unittest.TestCase):
    def test_short_write_only_sends_unsent_suffix(self):
        serial = Mock()
        serial.write.side_effect = [2, 1, 3]
        self.assertEqual(write_frame(serial, b'abcdef')[:2], (6, 3))
        self.assertEqual([c.args[0] for c in serial.write.call_args_list], [b'abcdef', b'cdef', b'def'])

    def test_negative_result_is_error(self):
        serial = Mock()
        serial.write.side_effect = [2, -5]
        with self.assertRaisesRegex(IOError, '2/6'):
            write_frame(serial, b'abcdef')

    def test_none_is_not_success(self):
        serial = Mock()
        serial.write.return_value = None
        with self.assertRaises(IOError):
            write_frame(serial, b'abc')

    def test_zero_write_is_bounded(self):
        serial = Mock()
        serial.write.return_value = 0
        with self.assertRaises(TimeoutError):
            write_frame(serial, b'abc', timeout_s=.005)


if __name__ == '__main__':
    unittest.main()
