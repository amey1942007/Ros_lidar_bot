import unittest
from agent.stopwords import check


class TestStopWords(unittest.TestCase):
    def test_each_word_case_insensitive(self):
        for w in ("halt", "HALT", "Freeze", "ruko", "Tham", "tHaM"):
            self.assertIsNotNone(check(w), w)
            self.assertFalse(check(w)["estop"])

    def test_emergency_fires_estop(self):
        r = check("EMERGENCY!!")
        self.assertTrue(r["estop"])
        self.assertTrue(check("halt, this is an emergency")["estop"])

    def test_whole_words_only(self):
        for t in ("go to the halter", "thames river", "freezer", "rukone", "emergencylight", "look at the table"):
            self.assertIsNone(check(t), t)

    def test_punctuation_and_sentences(self):
        self.assertEqual(check("please ruko.")["words"], ["ruko"])
        self.assertEqual(check("(freeze) now")["words"], ["freeze"])
        self.assertIsNone(check(""))
        self.assertIsNone(check(None))


if __name__ == "__main__":
    unittest.main()
