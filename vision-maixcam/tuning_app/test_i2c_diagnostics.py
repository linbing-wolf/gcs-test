import contextlib
import io
import sys
import types
import unittest
from collections import deque
from unittest.mock import patch, Mock

sys.modules.setdefault('fcntl', types.SimpleNamespace())
from i2c_transport import SynchronousTransport as I2CTransport


class DiagnosticsTests(unittest.TestCase):
    def transport(self):
        t = I2CTransport.__new__(I2CTransport)
        t.bus = Mock()
        t.bus.last_stage = 'address_write'
        t.bus.last_index = None
        t.queue = deque()
        t.queued_at = deque()
        t.next_poll = t.next_error = t.started = 0
        t.next_summary = float('inf')
        t.counts = dict(tx_attempt=0, tx_ok=0, tx_error=0, rx_attempt=0,
                        rx_ok=0, rx_error=0, rx_empty=0, malformed=0, error_streak=0)
        t.write(b'READY')
        return t

    def test_failed_write_preserves_packet_and_still_reads_command(self):
        t = self.transport()
        with patch.object(t.bus, 'write', side_effect=OSError(121, 'Remote I/O error')), patch.object(t.bus, 'read', return_value=b'\x01R'+bytes(30)) as read:
            with contextlib.redirect_stdout(io.StringIO()) as log:
                self.assertEqual(t.read(), b'R')
            self.assertIn('"phase": "write"', log.getvalue())
            self.assertIn('"tx_delivered_this_poll": false', log.getvalue())
            read.assert_called_once_with(32)
        self.assertEqual(len(t.queue), 1)
        self.assertEqual(t.counts['tx_error'], 1)

    def test_read_error_does_not_requeue_delivered_reply(self):
        t = self.transport()
        with patch.object(t.bus, 'write', return_value=32), patch.object(t.bus, 'read', side_effect=OSError(121, 'Remote I/O error')):
            with contextlib.redirect_stdout(io.StringIO()) as log:
                t.read()
            self.assertIn('"phase": "read"', log.getvalue())
            self.assertIn('"tx_delivered_this_poll": true', log.getvalue())
        self.assertEqual(len(t.queue), 0)
        self.assertEqual(t.counts['tx_ok'], 1)
        self.assertEqual(t.counts['rx_error'], 1)

    def test_short_write_retains_packet_for_retry(self):
        t = self.transport()
        t.separate_write = True
        with patch.object(t.bus, 'write', side_effect=(31, 32)) as write:
            self.assertEqual(t.read(allow_rx=False), b'')
            self.assertEqual(len(t.queue), 1)
            self.assertEqual(t.counts['tx_error'], 1)
            t.next_poll = 0
            self.assertEqual(t.read(allow_rx=False), b'')
            self.assertFalse(t.queue)
            self.assertEqual(t.counts['tx_ok'], 1)
            self.assertEqual(write.call_args_list[0], write.call_args_list[1])
        t.bus.read.assert_not_called()

    def test_valid_read_returns_only_valid_stream_bytes(self):
        t = self.transport()
        with patch.object(t.bus, 'write', return_value=32), patch.object(t.bus, 'read', return_value=b'\x01R'+bytes(30)):
            self.assertEqual(t.read(), b'R')
        self.assertEqual(t.counts['rx_ok'], 1)

    def test_static_mode_never_clocks_bus_with_reply_queued(self):
        t = self.transport()
        t.static_diag = True
        t.static_next = 0
        t.bus.scl.value.return_value = 1
        t.bus.sda.value.return_value = 1
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(t.read(), b'')
        t.bus.write.assert_not_called()
        t.bus.read.assert_not_called()
        self.assertEqual(len(t.queue), 1)


if __name__ == '__main__':
    unittest.main()
