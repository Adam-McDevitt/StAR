import unittest
from warehouse import Junction, Shelf, Van


class TestPutPackage(unittest.TestCase):

    def test_small_van(self):
        van = Van(2, 1, 2)

        self.assertEqual((1, 0, 0), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((1, 0, 1), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((0, 0, 0), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((0, 0, 1), van.get_next_pos())
        van.put_package("package")

        self.assertEqual(None, van.get_next_pos())
        with self.assertRaises(ValueError):
            van.put_package("package")

    def test_large_van(self):
        van = Van(3, 2, 2)

        self.assertEqual((2, 0, 0), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((2, 0, 1), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((2, 1, 0), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((2, 1, 1), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((1, 0, 0), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((1, 0, 1), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((1, 1, 0), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((1, 1, 1), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((0, 0, 0), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((0, 0, 1), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((0, 1, 0), van.get_next_pos())
        van.put_package("package")

        self.assertEqual((0, 1, 1), van.get_next_pos())
        van.put_package("package")

        self.assertEqual(None, van.get_next_pos())
        with self.assertRaises(ValueError):
            van.put_package("package")

class TestExampleLayout(unittest.TestCase):

    def setUp(self):

        # Example Warehouse
        #
        # S1     S2      S3     S1 = shelf1
        # |       |       |     S2 = shelf2
        # |_______J_______|     S3 = shelf3
        #         |              J = junction
        #         |_______V      V = van
        #
        # Each shelf is 2 packages high
        # Van is 2 packages deep, 2 packages wide and 1 package tall

        self.van = Van(2, 1, 2)
        self.junction = Junction()
        self.shelf1 = Shelf(["A", "B"])
        self.shelf2 = Shelf(["C", "D"])
        self.shelf3 = Shelf(["E", "F"])

        self.van.west = self.junction

        self.junction.north = self.shelf2
        self.junction.east = self.shelf3
        self.junction.south = self.van
        self.junction.west = self.shelf1

        self.shelf1.south = self.junction

        self.shelf2.south = self.junction

        self.shelf3.south = self.junction

    def test_navigation(self):
        curr = self.van  # van
        self.assertEqual(curr, self.van)
        curr = curr.west  # junction
        self.assertEqual(curr, self.junction)
        curr = curr.west  # shelf1
        self.assertEqual(curr, self.shelf1)
        curr = curr.south  # junction
        self.assertEqual(curr, self.junction)
        curr = curr.north  # shelf2
        self.assertEqual(curr, self.shelf2)
        curr = curr.south  # junction
        self.assertEqual(curr, self.junction)
        curr = curr.east  # shelf3
        self.assertEqual(curr, self.shelf3)
        curr = curr.south  # junction
        self.assertEqual(curr, self.junction)
        curr = curr.south  # van
        self.assertEqual(curr, self.van)

    def test_package_removal(self):
        shelf = self.shelf1

        self.assertEqual("A", shelf.package_at(0))
        shelf.remove_package_at(0)
        self.assertEqual(None, shelf.package_at(0))

        self.assertEqual("B", shelf.package_at(1))
        shelf.remove_package_at(1)
        self.assertEqual(None,shelf.package_at(1))




