import unittest
from Communication.src.utils import uhash, b_to_mb

class TestUtils(unittest.TestCase):
    def test_uhash(self):
        # Hashing the same content must always yeild the same hash
        data = b"data"
        self.assertEqual(uhash(data), uhash(data))
    
    def test_b_to_mb(self):
        self.assertEqual(0.0, b_to_mb(0))
        self.assertEqual(0.0032, b_to_mb(3200))
        