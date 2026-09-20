import sys
from sim_main import MMSMaze, MMSFloodfill, dijkstra_fastest_path, decompose_path_into_segments, execute_segment, DIR_DELTAS, DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST
import API

# Parse example5.num
def load_maze(filename):
    # walls[x][y] = [N, E, S, W]
    walls = [[ [False]*4 for _ in range(16) ] for _ in range(16)]
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 6:
                x, y = int(parts[0]), int(parts[1])
                n, e, s, w = int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])
                walls[x][y][0] = bool(n)
                walls[x][y][1] = bool(e)
                walls[x][y][2] = bool(s)
                walls[x][y][3] = bool(w)
    return walls

real_walls = load_maze("sim/example5.num")

class MockAPI:
    def __init__(self, walls):
        self.walls = walls
        self.x = 0
        self.y = 0
        self.heading = DIR_NORTH # 0: N, 1: E, 2: S, 3: W (can be 0.5 for 45 deg)
        self.semi_x = 1 # Semi-coords: center of (x, y) is (2x+1, 2y+1)
        self.semi_y = 1
        # semi_h: 0: N, 1: NE, 2: E, 3: SE, 4: S, 5: SW, 6: W, 7: NW
        self.semi_h = 0
        self.crashed = False
        self.moves_count = 0

    def mazeWidth(self): return 16
    def mazeHeight(self): return 16
    def wasReset(self): return False
    def ackReset(self): pass
    def setColor(self, x, y, c): pass
    def clearAllColor(self): pass
    def clearAllText(self): pass
    def setText(self, x, y, t): pass

    def get_cell(self):
        # Center is at odd coordinates
        return self.semi_x // 2, self.semi_y // 2

    def wallFront(self, half_steps=1):
        cx, cy = self.get_cell()
        d = self.semi_h // 2
        return self.walls[cx][cy][d]

    def wallRight(self, half_steps=1):
        cx, cy = self.get_cell()
        d = (self.semi_h // 2 + 1) % 4
        return self.walls[cx][cy][d]

    def wallLeft(self, half_steps=1):
        cx, cy = self.get_cell()
        d = (self.semi_h // 2 + 3) % 4
        return self.walls[cx][cy][d]

    def wallBack(self, half_steps=1):
        cx, cy = self.get_cell()
        d = (self.semi_h // 2 + 2) % 4
        return self.walls[cx][cy][d]

    def turnRight(self):
        self.semi_h = (self.semi_h + 2) % 8

    def turnLeft(self):
        self.semi_h = (self.semi_h - 2 + 8) % 8

    def turnRight45(self):
        self.semi_h = (self.semi_h + 1) % 8

    def turnLeft45(self):
        self.semi_h = (self.semi_h - 1 + 8) % 8

    def moveForward(self, count=1):
        self.moveForwardHalf(count * 2)

    def moveForwardHalf(self, count=1):
        # In MMS:
        # N: y+=count
        # E: x+=count
        # S: y-=count
        # W: x-=count
        # NE: x+=count, y+=count
        # SE: x+=count, y-=count
        # SW: x-=count, y-=count
        # NW: x-=count, y+=count
        deltas = [
            (0, 1),   # 0: N
            (1, 1),   # 1: NE
            (1, 0),   # 2: E
            (1, -1),  # 3: SE
            (0, -1),  # 4: S
            (-1, -1), # 5: SW
            (-1, 0),  # 6: W
            (-1, 1)   # 7: NW
        ]
        dx, dy = deltas[self.semi_h]
        for step in range(count):
            self.semi_x += dx
            self.semi_y += dy
            self.moves_count += 1
            # Check if out of bounds or crashed
            if self.semi_x < 0 or self.semi_x >= 32 or self.semi_y < 0 or self.semi_y >= 32:
                self.crashed = True
                raise API.MouseCrashedError(f"Crashed out of bounds at semi ({self.semi_x}, {self.semi_y})")

mock = MockAPI(real_walls)
API.mazeWidth = mock.mazeWidth
API.mazeHeight = mock.mazeHeight
API.wasReset = mock.wasReset
API.ackReset = mock.ackReset
API.setColor = mock.setColor
API.clearAllColor = mock.clearAllColor
API.clearAllText = mock.clearAllText
API.setText = mock.setText
API.wallFront = mock.wallFront
API.wallRight = mock.wallRight
API.wallLeft = mock.wallLeft
API.wallBack = mock.wallBack
API.turnRight = mock.turnRight
API.turnLeft = mock.turnLeft
API.turnRight45 = mock.turnRight45
API.turnLeft45 = mock.turnLeft45
API.moveForward = mock.moveForward
API.moveForwardHalf = mock.moveForwardHalf

if __name__ == "__main__":
    print("Running simulation test...")
    import sim_main
    sim_main.main()
