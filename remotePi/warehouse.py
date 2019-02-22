# Used to represent junctions. Subclasseses represent shelves and vans.
class Junction:
    def __init__(self, north=None, east=None, south=None, west=None):
        # each cardinal direction should be set to the junction that will eventually be reached if the robot follows the
        # path that starts in that direction. j.west is not necessarily actually to the west of j, but the uninterrupted
        # path (which may contain corners and turns) IS to the west of j. See warehouse_test.py for an example
        self.north = north
        self.east = east
        self.south = south
        self.west = west


class Shelf(Junction):
    def __init__(self, packages, north=None, east=None, south=None, west=None):
        self.__packages = packages
        Junction.__init__(self, north, east, south, west)

    def package_at(self, height):
        return self.__packages[height]

    def remove_package_at(self, height):
        package = self.__packages[height]
        self.__packages[height] = None
        return package


class Van(Junction):
    def __init__(self, depth, width, height, north=None, east=None, south=None, west=None):
        self.__packages = []
        self.width = width  # width between left and right wall when looking into van, measured in packages
        self.height = height  # height, measured in packages
        self.depth = depth  # depth of van from front to back, measured in packages
        Junction.__init__(self, north, east, south, west)

    def put_package(self, package):
        if len(self.__packages) >= self.depth * self.width * self.height:
            raise ValueError("Van is at maximum capacity")
        else:
            self.__packages.append(package)

    # Returns depth (van entrance to back of truck), width (left to right) and height (ground level to upper lever)
    # at which next package should be positioned. Values are zero-indexed. Packages are first placed at the back left
    # of the van.
    def get_next_pos(self):
        current_count = len(self.__packages)
        if current_count >= self.depth * self.width * self.height:
            return None
        height = current_count % self.height
        width = (current_count / self.height) % self.width
        depth = self.depth - 1 - (current_count / (self.height * self.width))
        return depth, width, height
