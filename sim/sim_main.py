import sys
import heapq
from collections import deque
import API

# Cardinal Directions: 0 = North, 1 = East, 2 = South, 3 = West
DIR_NORTH = 0
DIR_EAST  = 1
DIR_SOUTH = 2
DIR_WEST  = 3

DIR_CHARS = ['n', 'e', 's', 'w']
DIR_DELTAS = [(0, 1), (1, 0), (0, -1), (-1, 0)] # (dx, dy)

def log(msg):
    sys.stderr.write(f"[MOUSE] {msg}\n")
    sys.stderr.flush()

class MMSMaze:
    def __init__(self, width=16, height=16):
        self.width = width
        self.height = height
        self.walls = [[set() for _ in range(height)] for _ in range(width)]
        self.visited = [[False for _ in range(height)] for _ in range(width)]

        # Initialize outer boundary walls
        for x in range(width):
            self.add_wall(x, 0, DIR_SOUTH, send_to_api=False)
            self.add_wall(x, height - 1, DIR_NORTH, send_to_api=False)
        for y in range(height):
            self.add_wall(0, y, DIR_WEST, send_to_api=False)
            self.add_wall(width - 1, y, DIR_EAST, send_to_api=False)

        # Standard micromouse starting cell east wall
        self.add_wall(0, 0, DIR_EAST, send_to_api=True)

    def add_wall(self, x, y, direction, send_to_api=True):
        if not (0 <= x < self.width and 0 <= y < self.height):
            return
        d_char = DIR_CHARS[direction]
        if d_char not in self.walls[x][y]:
            self.walls[x][y].add(d_char)
            if send_to_api:
                API.setWall(x, y, d_char)

            # Update opposite neighbor wall
            dx, dy = DIR_DELTAS[direction]
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                opp_char = DIR_CHARS[(direction + 2) % 4]
                self.walls[nx][ny].add(opp_char)

    def has_wall(self, x, y, direction):
        if not (0 <= x < self.width and 0 <= y < self.height):
            return True
        return DIR_CHARS[direction] in self.walls[x][y]

class MMSFloodfill:
    def __init__(self, maze):
        self.maze = maze
        self.distance = [[9999 for _ in range(maze.height)] for _ in range(maze.width)]
        self.goals = set()

    def set_goal_center(self):
        cx1 = (self.maze.width // 2) - 1
        cx2 = self.maze.width // 2
        cy1 = (self.maze.height // 2) - 1
        cy2 = self.maze.height // 2
        self.goals = {(cx1, cy1), (cx1, cy2), (cx2, cy1), (cx2, cy2)}

        for gx, gy in self.goals:
            API.setColor(gx, gy, 'o') # Orange goal

    def set_goal_start(self):
        self.goals = {(0, 0)}
        API.setColor(0, 0, 'b') # Blue start

    def recalculate(self):
        for x in range(self.maze.width):
            for y in range(self.maze.height):
                self.distance[x][y] = 9999

        queue = deque()
        for gx, gy in self.goals:
            self.distance[gx][gy] = 0
            queue.append((gx, gy))

        # BFS shortest-path expansion
        while queue:
            cx, cy = queue.popleft()
            curr_dist = self.distance[cx][cy]

            for d in range(4):
                if not self.maze.has_wall(cx, cy, d):
                    dx, dy = DIR_DELTAS[d]
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.maze.width and 0 <= ny < self.maze.height:
                        if self.distance[nx][ny] > curr_dist + 1:
                            self.distance[nx][ny] = curr_dist + 1
                            queue.append((nx, ny))

        # Display distance numbers inside cells
        for x in range(self.maze.width):
            for y in range(self.maze.height):
                d = self.distance[x][y]
                if d < 9999:
                    API.setText(x, y, str(d))

    def get_next_direction(self, curr_x, curr_y, curr_heading):
        min_dist = 9999
        best_dir = None

        turn_preference = [0, 1, 3, 2] # Straight, Right, Left, Turnaround

        for turn in turn_preference:
            candidate_dir = (curr_heading + turn) % 4
            if not self.maze.has_wall(curr_x, curr_y, candidate_dir):
                dx, dy = DIR_DELTAS[candidate_dir]
                nx, ny = curr_x + dx, curr_y + dy
                if 0 <= nx < self.maze.width and 0 <= ny < self.maze.height:
                    d = self.distance[nx][ny]
                    if d < min_dist:
                        min_dist = d
                        best_dir = candidate_dir

        return best_dir

    def extract_full_path(self, start_x, start_y, initial_heading=DIR_NORTH):
        """Traces the shortest path of cells from start to goal."""
        path = [(start_x, start_y)]
        curr_x, curr_y = start_x, start_y
        heading = initial_heading

        while (curr_x, curr_y) not in self.goals:
            next_d = self.get_next_direction(curr_x, curr_y, heading)
            if next_d is None:
                break
            dx, dy = DIR_DELTAS[next_d]
            curr_x += dx
            curr_y += dy
            path.append((curr_x, curr_y))
            heading = next_d
            if len(path) > 300: # Safety guard against cycles
                break

        return path

def turn_in_place(curr_h, target_h):
    """Standard in-place turn."""
    diff = (target_h - curr_h + 4) % 4
    if diff == 1:
        API.turnRight()
    elif diff == 3:
        API.turnLeft()
    elif diff == 2:
        API.turnRight()
        API.turnRight()
    return target_h

def find_slalom_length(dirs, start_idx):
    """
    Detects an up-and-down / left-and-right zigzag (slalom pattern).
    Pattern: alternating progression direction and opposite cross-directions:
    e.g. W, S, W, N, W, S, W, N, W (length 9)
    Requires an odd length >= 5 so the maneuver enters and exits along d_prog.
    """
    n = len(dirs)
    if start_idx + 4 >= n:
        return 0
    d_prog = dirs[start_idx]
    d_c1 = dirs[start_idx + 1]
    if (d_c1 - d_prog) % 2 != 0: # perpendicular
        d_c2 = (d_c1 + 2) % 4
        length = 2
        while start_idx + length < n:
            rel = length
            if rel % 2 == 0:
                if dirs[start_idx + length] != d_prog:
                    break
            else:
                exp_c = d_c1 if (rel // 2) % 2 == 0 else d_c2
                if dirs[start_idx + length] != exp_c:
                    break
            length += 1
        while length >= 5 and length % 2 == 0:
            length -= 1
        if length >= 5:
            return length
    return 0

def dijkstra_fastest_path(maze, start_x, start_y, start_h, goals):
    """
    Computes the TRUE FASTEST ROUTE for the Speed Run using a weighted state-space search.
    Accounts for:
    - Long straightaway acceleration discounts (sprints are 3x faster than twisty routes)
    - Deceleration penalties for 90-degree turns (+1.5 cost)
    - 45-degree diagonal staircases (one big continuous diagonal)
    - Up-and-down zigzag slaloms (continuous diagonal carving with 90° arc curves)
    """
    width, height = maze.width, maze.height
    pq = [(0.0, start_x, start_y, start_h)]
    best_cost = {}
    parent = {}

    best_goal_state = None

    while pq:
        cost, x, y, h = heapq.heappop(pq)
        state = (x, y, h)
        if state in best_cost and best_cost[state] <= cost:
            continue
        best_cost[state] = cost

        if (x, y) in goals:
            best_goal_state = state
            break

        # 1. In-place turn (90 deg left / right)
        for td in [1, 3]:
            nh = (h + td) % 4
            tc = cost + 1.5
            nstate = (x, y, nh)
            if nstate not in best_cost or best_cost[nstate] > tc:
                heapq.heappush(pq, (tc, x, y, nh))
                if nstate not in parent or tc < best_cost.get(nstate, float('inf')):
                    parent[nstate] = ((x, y, h), [])

        # 2. Straight sprint of length L (1 to 15 cells)
        dx, dy = DIR_DELTAS[h]
        for L in range(1, 16):
            px = x + dx * (L - 1)
            py = y + dy * (L - 1)
            if maze.has_wall(px, py, h):
                break
            nx = x + dx * L
            ny = y + dy * L
            if not (0 <= nx < width and 0 <= ny < height):
                break
            if not maze.visited[nx][ny]:
                break

            sprint_cost = cost + 1.0 + 0.35 * (L - 1)
            nstate = (nx, ny, h)
            if nstate not in best_cost or best_cost[nstate] > sprint_cost:
                heapq.heappush(pq, (sprint_cost, nx, ny, h))
                seg = [(x + dx * s, y + dy * s) for s in range(1, L + 1)]
                parent[nstate] = ((x, y, h), seg)

        # 3. Big Diagonal Staircase (M >= 2)
        for td in [1, 3]:
            d1 = (h + td) % 4
            cur_x, cur_y = x, y
            diag_seg = []
            for m_step in range(1, 16):
                s_dir = h if m_step % 2 == 1 else d1
                if maze.has_wall(cur_x, cur_y, s_dir):
                    break
                sdx, sdy = DIR_DELTAS[s_dir]
                cur_x += sdx
                cur_y += sdy
                if not (0 <= cur_x < width and 0 <= cur_y < height):
                    break
                if not maze.visited[cur_x][cur_y]:
                    break
                diag_seg.append((cur_x, cur_y))
                if m_step >= 2:
                    diag_cost = cost + 1.8 + (m_step - 1) * 0.5
                    end_h = s_dir
                    nstate = (cur_x, cur_y, end_h)
                    if nstate not in best_cost or best_cost[nstate] > diag_cost:
                        heapq.heappush(pq, (diag_cost, cur_x, cur_y, end_h))
                        parent[nstate] = ((x, y, h), diag_seg[:])

        # 4. Up-and-Down Slalom Zigzag (Length >= 5, odd)
        for td in [1, 3]:
            d_c1 = (h + td) % 4
            d_c2 = (d_c1 + 2) % 4
            cur_x, cur_y = x, y
            slalom_seg = []
            for m_idx in range(16):
                if m_idx % 2 == 0:
                    s_dir = h
                else:
                    s_dir = d_c1 if (m_idx // 2) % 2 == 0 else d_c2

                if maze.has_wall(cur_x, cur_y, s_dir):
                    break
                sdx, sdy = DIR_DELTAS[s_dir]
                cur_x += sdx
                cur_y += sdy
                if not (0 <= cur_x < width and 0 <= cur_y < height):
                    break
                if not maze.visited[cur_x][cur_y]:
                    break
                slalom_seg.append((cur_x, cur_y))
                m_count = m_idx + 1
                if m_count >= 5 and m_count % 2 == 1:
                    num_w = (m_count - 1) // 2
                    slalom_cost = cost + 1.5 + num_w * 1.2 + 0.5
                    end_h = h
                    nstate = (cur_x, cur_y, end_h)
                    if nstate not in best_cost or best_cost[nstate] > slalom_cost:
                        heapq.heappush(pq, (slalom_cost, cur_x, cur_y, end_h))
                        parent[nstate] = ((x, y, h), slalom_seg[:])

    if best_goal_state is None:
        return []

    # Reconstruct full path
    full_path = []
    curr = best_goal_state
    while curr in parent:
        prev_state, seg = parent[curr]
        full_path = seg + full_path
        curr = prev_state
    full_path = [(start_x, start_y)] + full_path
    return full_path

def decompose_path_into_segments(path):
    """
    Decomposes a continuous path of cells into optimal execution primitives:
    - SLALOM: Up-and-down / left-and-right continuous zigzags
    - DIAGONAL: Diagonal staircases and 45° corner cuts
    - STRAIGHT: High-speed corridor sprints
    """
    if len(path) < 2:
        return []

    dirs = []
    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        dirs.append(DIR_DELTAS.index((dx, dy)))

    actions = []
    i = 0
    n = len(dirs)

    while i < n:
        # 1. Check for up-and-down slalom zigzag starting at i
        slen = find_slalom_length(dirs, i)
        if slen >= 5:
            slalom_moves = dirs[i : i + slen]
            actions.append(('SLALOM', slalom_moves, path[i], path[i + slen]))
            i += slen
            continue

        # If the NEXT step starts a slalom, don't bundle dirs[i] into a 2-step diagonal!
        if i + 1 < n and find_slalom_length(dirs, i + 1) >= 5:
            actions.append(('STRAIGHT', dirs[i], 1, path[i], path[i + 1]))
            i += 1
            continue

        # 2. Check for diagonal staircase starting at i
        diag_len = 0
        if i + 1 < n:
            d_a = dirs[i]
            d_b = dirs[i+1]
            diff = (d_b - d_a) % 4
            if diff in [1, 3]:
                k = 2
                while i + k < n:
                    exp = d_a if k % 2 == 0 else d_b
                    if dirs[i + k] == exp:
                        k += 1
                    else:
                        break
                diag_len = k

        if diag_len >= 2:
            diag_moves = dirs[i : i + diag_len]
            actions.append(('DIAGONAL', diag_moves, path[i], path[i + diag_len]))
            i += diag_len
        else:
            # 3. Straight run
            d = dirs[i]
            cnt = 1
            while i + cnt < n and dirs[i + cnt] == d:
                # If next step can start a slalom or diagonal, stop straight run early
                if find_slalom_length(dirs, i + cnt) >= 5:
                    break
                if i + cnt + 1 < n and find_slalom_length(dirs, i + cnt + 1) >= 5:
                    break
                if i + cnt + 1 < n:
                    p_diff = (dirs[i + cnt + 1] - d) % 4
                    if p_diff in [1, 3]:
                        break
                cnt += 1
            actions.append(('STRAIGHT', d, cnt, path[i], path[i + cnt]))
            i += cnt

    return actions

def execute_segment(segment, current_heading, maze, color):
    """
    Executes a single planned segment (Straight Sprint, Diagonal Glide, or Slalom Weave).
    Returns (new_x, new_y, new_heading).
    """
    seg_type = segment[0]

    if seg_type == 'STRAIGHT':
        _, d, cnt, start_c, end_c = segment
        if current_heading != d:
            current_heading = turn_in_place(current_heading, d)

        if cnt > 1:
            log(f"🚀 Sprinting {cnt} cells STRAIGHT {DIR_CHARS[d].upper()} from {start_c} to {end_c}!")
        API.moveForward(cnt)

        # Mark visited and color
        dx, dy = DIR_DELTAS[d]
        for s in range(1, cnt + 1):
            cx = start_c[0] + dx * s
            cy = start_c[1] + dy * s
            maze.visited[cx][cy] = True
            API.setColor(cx, cy, color)

        return end_c[0], end_c[1], d

    elif seg_type == 'DIAGONAL':
        _, diag_moves, start_c, end_c = segment
        M = len(diag_moves)
        d1 = diag_moves[0]
        d2 = diag_moves[1]
        d_last = diag_moves[-1]
        d_prev = diag_moves[-2]

        diag_name = None
        f_ch, s_ch = DIR_CHARS[d1].upper(), DIR_CHARS[d2].upper()
        if (f_ch, s_ch) in [('N', 'E'), ('E', 'N')]: diag_name = 'NORTHEAST'
        elif (f_ch, s_ch) in [('N', 'W'), ('W', 'N')]: diag_name = 'NORTHWEST'
        elif (f_ch, s_ch) in [('S', 'E'), ('E', 'S')]: diag_name = 'SOUTHEAST'
        elif (f_ch, s_ch) in [('S', 'W'), ('W', 'S')]: diag_name = 'SOUTHWEST'

        if current_heading != d1:
            current_heading = turn_in_place(current_heading, d1)

        turn_in_diff = (d2 - d1 + 4) % 4
        turn_out_diff = (d_last - d_prev + 4) % 4
        diag_half_steps = M - 1

        if M > 2:
            log(f"⚡ [BIG DIAGONAL] Gliding {diag_half_steps} half-steps straight through {diag_name} across {M} cells from {start_c} to {end_c}!")
        else:
            t_str = "RIGHT" if turn_in_diff == 1 else "LEFT"
            log(f"⚡ [DIAGONAL] Cutting corner 45° {t_str} through ({start_c[0] + DIR_DELTAS[d1][0]}, {start_c[1] + DIR_DELTAS[d1][1]}) into {end_c}!")

        # 1. Approach edge
        API.moveForwardHalf(1)

        # 2. Turn 45° into diagonal
        if turn_in_diff == 1:
            API.turnRight45()
        else:
            API.turnLeft45()

        # 3. ONE BIG CONTINUOUS DIAGONAL SPRINT!
        API.moveForwardHalf(diag_half_steps)

        # 4. Turn 45° to align with exit direction
        if turn_out_diff == 1:
            API.turnRight45()
        else:
            API.turnLeft45()

        # 5. Enter destination cell center
        API.moveForwardHalf(1)

        # Mark all cells in diagonal
        cur_x, cur_y = start_c
        for m in diag_moves:
            cur_x += DIR_DELTAS[m][0]
            cur_y += DIR_DELTAS[m][1]
            maze.visited[cur_x][cur_y] = True
            API.setColor(cur_x, cur_y, color)

        return end_c[0], end_c[1], d_last

    elif seg_type == 'SLALOM':
        _, slalom_moves, start_c, end_c = segment
        d_prog = slalom_moves[0]
        d_c1 = slalom_moves[1]
        num_waves = (len(slalom_moves) - 1) // 2

        prog_name = DIR_CHARS[d_prog].upper()
        c1_name = DIR_CHARS[d_c1].upper()
        c2_name = DIR_CHARS[(d_c1 + 2) % 4].upper()
        log(f"⚡ [UP-AND-DOWN ZIGZAG] Carving {num_waves} continuous diagonal slalom waves ({c1_name}/{c2_name}) progressing {prog_name} across {len(slalom_moves)} cells from {start_c} to {end_c}!")

        # 1. Turn in place if not facing d_prog
        if current_heading != d_prog:
            current_heading = turn_in_place(current_heading, d_prog)

        # 2. Approach entrance edge
        API.moveForwardHalf(1)

        # 3. Turn 45° into first diagonal
        t_in_diff = (d_c1 - d_prog + 4) % 4
        if t_in_diff == 1:
            API.turnRight45()
        else:
            API.turnLeft45()

        # 4. Weave through all waves
        for w in range(num_waves):
            API.moveForwardHalf(2) # Glide 2 half-steps across this wave
            if w < num_waves - 1:
                # 90° curve into the next wave
                if (t_in_diff == 1 and w % 2 == 0) or (t_in_diff == 3 and w % 2 == 1):
                    API.turnLeft()
                else:
                    API.turnRight()

        # 5. Align with exit heading (d_prog)
        if (t_in_diff == 1 and (num_waves - 1) % 2 == 0) or (t_in_diff == 3 and (num_waves - 1) % 2 == 1):
            API.turnLeft45()
        else:
            API.turnRight45()

        # 6. Enter destination cell center
        API.moveForwardHalf(1)

        # Mark all cells in slalom as visited and color
        cur_x, cur_y = start_c
        for m in slalom_moves:
            cur_x += DIR_DELTAS[m][0]
            cur_y += DIR_DELTAS[m][1]
            maze.visited[cur_x][cur_y] = True
            API.setColor(cur_x, cur_y, color)

        return end_c[0], end_c[1], d_prog

def main():
    log("Initializing Antigravitieee High-Speed Diagonal Navigator...")

    width = API.mazeWidth()
    height = API.mazeHeight()
    maze = MMSMaze(width, height)
    flood = MMSFloodfill(maze)

    x, y = 0, 0
    heading = DIR_NORTH

    flood.set_goal_center()
    flood.recalculate()

    state = "EXPLORING"
    log(f"Maze dimensions: {width}x{height}. Ready.")

    while True:
        if API.wasReset():
            API.ackReset()
            API.clearAllColor()
            API.clearAllText()
            x, y = 0, 0
            heading = DIR_NORTH
            maze = MMSMaze(width, height)
            flood = MMSFloodfill(maze)
            flood.set_goal_center()
            flood.recalculate()
            state = "EXPLORING"
            log("Simulator Reset. Restarting.")

        maze.visited[x][y] = True

        color = 'g' if state == "EXPLORING" else ('c' if state == "RETURNING" else 'y')
        API.setColor(x, y, color)

        # 1. Read virtual sensors (All 4 directions to eliminate unseen walls!)
        wall_front = API.wallFront()
        wall_right = API.wallRight()
        wall_left  = API.wallLeft()
        wall_back  = API.wallBack()

        dir_front = heading
        dir_right = (heading + 1) % 4
        dir_left  = (heading + 3) % 4
        dir_back  = (heading + 2) % 4

        if wall_front:
            maze.add_wall(x, y, dir_front)
        if wall_right:
            maze.add_wall(x, y, dir_right)
        if wall_left:
            maze.add_wall(x, y, dir_left)
        if wall_back:
            maze.add_wall(x, y, dir_back)

        # 2. State Transitions & High-Speed Execution
        if state == "EXPLORING":
            if (x, y) in flood.goals:
                log(f"🎯 CENTER REACHED at ({x}, {y})! Rapidly returning to Start (0,0) via FASTEST DIAGONAL ROUTE...")
                state = "RETURNING"

        if state == "RETURNING":
            # Compute fastest return path to (0,0) using Dijkstra
            return_path = dijkstra_fastest_path(maze, x, y, heading, {(0, 0)})
            if len(return_path) < 2:
                # Fallback to BFS floodfill if no Dijkstra path found
                flood_return = MMSFloodfill(maze)
                flood_return.goals = {(0, 0)}
                flood_return.recalculate()
                return_path = flood_return.extract_full_path(x, y, heading)

            segments = decompose_path_into_segments(return_path)
            log(f"🚀 [RETURNING] Sprinting {len(return_path)} cells across {len(segments)} high-speed segments back to (0,0)...")
            for seg in segments:
                try:
                    x, y, heading = execute_segment(seg, heading, maze, 'c')
                except API.MouseCrashedError:
                    log(f"💥 CRASH DETECTED at ({x}, {y}) returning!")
                    return
            log("🚀 START REACHED at (0,0)! Maze fully mapped. Launching MAXIMUM-VELOCITY DIAGONAL SPEED RUN!")
            state = "SPEEDRUN"

        if state == "SPEEDRUN":
            # Compute TRUE FASTEST ROUTE using state-space Dijkstra
            fastest_path = dijkstra_fastest_path(maze, x, y, heading, flood.goals)
            if len(fastest_path) < 2:
                log("Goal reached or path found.")
                break

            # Highlight the fastest route in yellow on the maze
            for px, py in fastest_path:
                API.setColor(px, py, 'y')

            # Decompose entire fastest path into straight sprints, big diagonals, and slaloms
            segments = decompose_path_into_segments(fastest_path)
            log(f"🏆 Speedrun Path: {len(fastest_path)} cells, {len(segments)} high-speed segments.")

            # Execute all segments at maximum velocity
            for seg in segments:
                try:
                    x, y, heading = execute_segment(seg, heading, maze, 'y')
                except API.MouseCrashedError:
                    log(f"💥 CRASH DETECTED at ({x}, {y}) during speed run!")
                    return
            log("🏆 CHAMPIONSHIP RUN COMPLETE! Center reached at maximum velocity with BIG CONTINUOUS DIAGONALS!")
            break

        else: # EXPLORING
            flood.recalculate()
            path = flood.extract_full_path(x, y, heading)
            if len(path) < 2:
                log(f"Trapped or center reached at ({x}, {y})")
                continue

            # Check if next step is unvisited
            next_x, next_y = path[1]
            if not maze.visited[next_x][next_y]:
                # Move carefully into unvisited cell to map walls
                d0 = DIR_DELTAS.index((next_x - x, next_y - y))
                if heading != d0:
                    heading = turn_in_place(heading, d0)
                try:
                    API.moveForward(1)
                except API.MouseCrashedError:
                    log(f"💥 CRASH DETECTED entering ({next_x}, {next_y})!")
                    break
                x, y = next_x, next_y
            else:
                # Moving through visited cells: check if we can execute a segment
                v_len = 1
                while v_len < len(path) and maze.visited[path[v_len][0]][path[v_len][1]]:
                    v_len += 1

                subpath = path[:v_len]
                segments = decompose_path_into_segments(subpath)
                if segments:
                    seg = segments[0]
                    try:
                        x, y, heading = execute_segment(seg, heading, maze, 'g')
                    except API.MouseCrashedError:
                        log(f"💥 CRASH DETECTED backtracking at ({x}, {y})!")
                        break
                else:
                    d0 = DIR_DELTAS.index((next_x - x, next_y - y))
                    if heading != d0:
                        heading = turn_in_place(heading, d0)
                    try:
                        API.moveForward(1)
                    except API.MouseCrashedError:
                        log(f"💥 CRASH DETECTED at ({next_x}, {next_y})!")
                        break
                    x, y = next_x, next_y

if __name__ == "__main__":
    main()
