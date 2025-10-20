from collections import deque

# Hướng di chuyển
DIRS = {
    "U": (-1, 0),
    "D": (1, 0),
    "L": (0, -1),
    "R": (0, 1)
}

def bfs_solver(walls, goals, player_start, boxes_start):
    start_state = (player_start, frozenset(boxes_start))
    queue = deque([(start_state, "")])   # (state, path)
    visited = set([start_state])

    while queue:
        (player, boxes), path = queue.popleft()

        # Goal test
        if boxes == goals:
            return path

        # Duyệt 4 hướng
        for action, (dx, dy) in DIRS.items():
            new_player = (player[0] + dx, player[1] + dy)

            if new_player in walls:
                continue

            new_boxes = set(boxes)

            # Nếu có box → thử push
            if new_player in boxes:
                new_box = (new_player[0] + dx, new_player[1] + dy)
                if new_box in walls or new_box in boxes:
                    continue  # Push không hợp lệ
                new_boxes.remove(new_player)
                new_boxes.add(new_box)

            # State mới
            new_state = (new_player, frozenset(new_boxes))

            if new_state not in visited:
                visited.add(new_state)
                queue.append((new_state, path + action))

    return None  # Không tìm thấy

# Map mini
# walls = {(0,0),(0,1),(0,2),(0,3),(0,4),
#          (1,0),                  (1,4),
#          (2,0),                  (2,4),
#          (3,0),                  (3,4),
#          (4,0),(4,1),(4,2),(4,3),(4,4)}

# goals = {(1,1)}              # vị trí goal
# player_start = (2,2)         # @
# boxes_start = {(2,1)}        # $

# solution = bfs_solver(walls, goals, player_start, boxes_start)
# print("Solution:", solution)
