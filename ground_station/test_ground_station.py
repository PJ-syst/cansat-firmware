import unittest

from ground_station.cansat_ground_station import format_command, parse_telemetry


PACKET = (
    "TEAM1,00:00:01,1,FLIGHT,ASCENT,12.3,24.5,99.12,4.10,NA,"
    "1.0,2.0,3.0,0.1,0.2,0.9,12:35:19,550.0,0.347596,32.582520,9,CXON\r"
)


class GroundStationTests(unittest.TestCase):
    def test_valid_packet(self):
        packet = parse_telemetry(PACKET, "TEAM1")
        self.assertEqual(packet["packet_count"], 1)
        self.assertAlmostEqual(packet["altitude_m"], 12.3)
        self.assertIsNone(packet["current_a"])

    def test_wrong_field_count_and_team(self):
        with self.assertRaises(ValueError):
            parse_telemetry("too,few", "TEAM1")
        with self.assertRaises(ValueError):
            parse_telemetry(PACKET, "OTHER")

    def test_command_format(self):
        self.assertEqual(format_command("TEAM1", "CXON"), b"TEAM1,CXON\r")
        with self.assertRaises(ValueError):
            format_command("BAD,TEAM", "CXON")


if __name__ == "__main__":
    unittest.main()
