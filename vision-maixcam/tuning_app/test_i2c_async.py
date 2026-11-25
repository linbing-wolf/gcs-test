from itertools import chain, repeat
import threading
import time
import unittest
from unittest.mock import patch
from i2c_transport import I2CTransport


class AsyncTests(unittest.TestCase):
    def test_slow_bus_does_not_block_application_and_tx_wakes_worker(self):
        entered = threading.Event()
        release = threading.Event()
        sent = threading.Event()
        closed = threading.Event()
        class Link:
            def __init__(self, *args):
                self.queue = []
                self.counts = dict(tx_error=0, rx_error=0)
            def write(self, data):
                self.queue.append(data)
            def read(self, *, allow_rx=True, allow_tx=True):
                if self.queue and allow_tx:
                    self.queue.pop(0)
                    sent.set()
                    return b''
                entered.set()
                if not release.wait(2):
                    raise RuntimeError('test bus release timeout')
                return b'R'
            def close(self):
                closed.set()
        with patch('i2c_transport.SynchronousTransport', Link):
            app = I2CTransport('', 0)
            try:
                self.assertTrue(entered.wait(1))
                started = time.monotonic()
                self.assertEqual(app.read(), b'')
                self.assertEqual(app.write(b'READY'), 5)
                self.assertLess(time.monotonic()-started, .05)
                release.set()
                self.assertTrue(sent.wait(1))
                self.assertEqual(app.read(), b'R')
            finally:
                release.set()
                app.close()
            self.assertTrue(closed.is_set())

    def test_malformed_mailbox_does_not_stop_worker(self):
        from test_i2c_diagnostics import DiagnosticsTests
        link = DiagnosticsTests().transport()
        link.queue.clear()
        link.queued_at.clear()
        link.bus.read.side_effect = chain((bytes([32]) + bytes(31), b'\x01R' + bytes(30)), repeat(bytes(32)))
        with patch('i2c_transport.SynchronousTransport', return_value=link):
            app = I2CTransport('', 0)
            try:
                deadline = time.monotonic() + 1
                data = b''
                while not data and time.monotonic() < deadline:
                    data = app.read()
                    if not data:
                        time.sleep(.01)
                self.assertEqual(data, b'R')
                self.assertEqual(link.counts['malformed'], 1)
            finally:
                app.close()

    def test_worker_initialization_error_is_visible(self):
        with patch('i2c_transport.SynchronousTransport', side_effect=OSError('GPIO unavailable')):
            with self.assertRaisesRegex(RuntimeError, 'GPIO unavailable'):
                I2CTransport('', 0)


if __name__ == '__main__':
    unittest.main()
