from collections import deque
import time
import tracemalloc
import psutil
import os

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

import heapq
def heuristic(boxes, goals):
    """
    Heuristic function: Calculate manhattan distance from boxes to nearest goals
    estimate the cost to reach the goal state.
    """
    if not boxes or not goals:
        return 0

    total = 0
    goals_list = list(goals)
    boxes_list = list(boxes)

    # Tính tổng khoảng cách Manhattan tối thiểu
    for box in boxes_list:
        min_dist = float('inf')
        for goal in goals_list:
            dist = abs(box[0] - goal[0]) + abs(box[1] - goal[1])
            min_dist = min(min_dist, dist)
        total += min_dist

    return total


def is_deadlock(box_pos, walls, goals):
    """
    Check if the box is in a deadlock position
    """
    if box_pos in goals:
        return False

    r, c = box_pos
    # Kiểm tra góc
    if ((r - 1, c) in walls or (r + 1, c) in walls) and \
            ((r, c - 1) in walls or (r, c + 1) in walls):
        return True

    return False


def can_push(player, box, direction, walls, boxes):
    """
    Kiểm tra xem có thể đẩy box theo hướng direction không
    """
    dr, dc = DIRS[direction]
    new_box = (box[0] + dr, box[1] + dc)

    # Vị trí mới của box không được là tường hoặc box khác
    if new_box in walls or new_box in boxes:
        return False

    return True


def get_next_states(state, walls, goals):
    """
    Return a list of possible next states from the current state
    """
    player, boxes = state
    next_states = []
    boxes_set = set(boxes)

    for direction, (dr, dc) in DIRS.items():
        new_player = (player[0] + dr, player[1] + dc)

        # Kiểm tra va chạm tường
        if new_player in walls:
            continue

        # Nếu di chuyển vào vị trí có box
        if new_player in boxes_set:
            new_box = (new_player[0] + dr, new_player[1] + dc)

            # Kiểm tra xem có thể đẩy box không
            if new_box in walls or new_box in boxes_set:
                continue

            # Kiểm tra deadlock
            if is_deadlock(new_box, walls, goals):
                continue

            # Tạo state mới với box đã di chuyển
            new_boxes = boxes_set - {new_player} | {new_box}
            next_states.append((new_player, frozenset(new_boxes), direction))
        else:
            # Di chuyển bình thường không đẩy box
            next_states.append((new_player, boxes, direction))

    return next_states


def Astar(walls, goals, player, boxes):
    """
    A* search algorithm to solve Sokoban
    """
    start_time = time.time()
    
    # Bắt đầu theo dõi bộ nhớ
    tracemalloc.start()
    process = psutil.Process(os.getpid())
    mem_before = process.memory_info().rss / 1024 / 1024
    start_state = (player, frozenset(boxes))
    goals_set = set(goals)

    # Using Priority queue: (f_score, g_score, state, path)
    # f_score = g_score + h_score
    h_start = heuristic(boxes, goals_set)
    pq = [(h_start, 0, start_state, [])]

    # Store the best g_score for each visited state
    visited = {start_state: 0}

    nodes_explored = 0
    max_queue_size = 0

    while pq:
        f_score, g_score, current_state, path = heapq.heappop(pq)
        current_player, current_boxes = current_state

        nodes_explored += 1

        # Kiểm tra điều kiện thắng
        if current_boxes == frozenset(goals_set):
            # Kết thúc đo thời gian
            end_time = time.time()
            execution_time = end_time - start_time
            
            # Lấy thông tin bộ nhớ
            current, peak = tracemalloc.get_traced_memory()
            tracemalloc.stop()
            mem_after = process.memory_info().rss / 1024 / 1024  # MB
            
            # In kết quả
            print("=" * 70)
            print("KẾT QUẢ TÌM KIẾM A*")
            print("=" * 70)
            print(f"✓ Tìm thấy lời giải!")
            print(f"✓ Số bước: {len(path)}")
            print(f"✓ Số node đã duyệt: {nodes_explored}")
            print(f"✓ Kích thước queue tối đa: {max_queue_size}")
            print(f"✓ Số state đã lưu: {len(visited)}")
            print("-" * 60)
            print(f"⏱️  Thời gian thực thi: {execution_time:.4f} giây ({execution_time*1000:.2f} ms)")
            print("-" * 60)
            print(f"💾 Tăng bộ nhớ: {mem_after - mem_before:.2f} MB")
            print("=" * 60)
            
            return path

        # Nếu đã tìm được đường tốt hơn đến state này, bỏ qua
        if g_score > visited.get(current_state, float('inf')):
            continue

        # Duyệt các state kế tiếp
        for next_player, next_boxes, direction in get_next_states(
                current_state, walls, goals_set
        ):
            next_state = (next_player, next_boxes)
            new_g_score = g_score + 1

            # Chỉ xét nếu tìm được đường tốt hơn
            if new_g_score < visited.get(next_state, float('inf')):
                visited[next_state] = new_g_score
                h_score = heuristic(next_boxes, goals_set)
                new_f_score = new_g_score + h_score
                new_path = path + [direction]

                heapq.heappush(pq, (new_f_score, new_g_score, next_state, new_path))
                
                # Cập nhật kích thước queue tối đa
                max_queue_size = max(max_queue_size, len(pq))

    # Không tìm thấy lời giải
    end_time = time.time()
    execution_time = end_time - start_time
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    mem_after = process.memory_info().rss / 1024 / 1024
    print("Không tìm thấy lời giải!")
    print(f"Số node đã duyệt: {nodes_explored}")
    return None


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
