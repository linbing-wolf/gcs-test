import unittest

import h_packet


class HExtendedPacketTests(unittest.TestCase):
    def test_pack_sorts_colors_and_includes_arrival_state(self):
        payload = h_packet.pack_h_disc2(
            1,
            [("3", 500, 240), ("1", 100, 230), ("2", 300, 250)],
        )
        self.assertEqual(
            payload,
            "HDISC2,1,3,1,100,230,2,300,250,3,500,240",
        )

    def test_rejects_invalid_arrival_or_position_values(self):
        with self.assertRaises(ValueError):
            h_packet.pack_h_disc2(2, [])
        with self.assertRaises(ValueError):
            h_packet.pack_h_disc2(
                1, [("1", 10, 20), ("1", 30, 40)])


if __name__ == "__main__":
    unittest.main()
