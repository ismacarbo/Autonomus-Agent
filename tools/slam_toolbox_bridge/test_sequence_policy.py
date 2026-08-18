#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sequence_policy import classify_packet


class SequencePolicyTest(unittest.TestCase):
    def test_new_session_is_the_only_reset_trigger(self) -> None:
        self.assertEqual(classify_packet("old", 80, "new", 1), "new_session")

    def test_duplicate_and_reordered_packets_are_stale(self) -> None:
        self.assertEqual(classify_packet("run", 80, "run", 80), "stale")
        self.assertEqual(classify_packet("run", 80, "run", 79), "stale")

    def test_monotonic_packet_is_accepted(self) -> None:
        self.assertEqual(classify_packet("run", 80, "run", 81), "accept")


if __name__ == "__main__":
    unittest.main()
