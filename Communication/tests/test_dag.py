import unittest
from dag import Node, EMPTY_CONTENT

class TestNode(unittest.TestCase):
    def test_init(self):
        n = Node()
        self.assertEqual(n.content, EMPTY_CONTENT)
        self.assertEqual(n.signature, "")
        self.assertEqual(n.children, [])
    
    def test_hash(self):
        # Two empty nodes should be the same when hashed
        n1 = Node()
        n2 = Node()
        h1, h2 = n1.hash(), n2.hash()
        self.assertEqual(h1, h2)
        # Changing content of effect its hash
        n2.content = 1
        h1, h2 = n1.hash(), n2.hash()
        self.assertNotEqual(h1, h2)
        # Changing children will effect its hash
        n3 = Node()
        n3.children = [n2.signature]
        h1, h3 = n1.hash(), n3.hash()
        self.assertNotEqual(h1, h3)
        # Calling hash on a node will change the signature
        n4 = Node()
        n4.content = 30
        n4.children = [n2.signature]
        self.assertEqual("", n4.signature)
        h4 = n4.hash()
        self.assertEqual(n4.signature, h4)
    
    def test_empty_content(self):
        n = Node()
        self.assertTrue(n.empty_content())
        n.content = 0
        self.assertFalse(n.empty_content())
        n.content = EMPTY_CONTENT
        self.assertTrue(n.empty_content())
    
    def test_is_leaf(self):
        n = Node()
        self.assertFalse(n.is_leaf())
        n.children = ["something"]
        self.assertFalse(n.is_leaf())
        n.content = 21
        self.assertFalse(n.is_leaf())
        n.children = []
        self.assertTrue(n.is_leaf())

if __name__ == '__main__':
    unittest.main()