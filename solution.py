from collections import deque
import time
import tracemalloc
import psutil
import os
import heapq

# Hướng di chuyển (row, col)
DIRS = {
    "U": (-1, 0),
    "D": (1, 0),
    "L": (0, -1),
    "R": (0, 1)
}

def format_solution(path, use_arrows=True):
    """
    Format solution path for better readability.
    Example:
        Input: ['U','L','L','U']
        Output (arrows=True):
            ↑ ← ← ↑
            (Total steps: 4)
    """
    if not path:
        return "(Không có lời giải)"

    # Mapping hướng -> mũi tên
    arrow_map = {
        "U": "↑",
        "D": "↓",
        "L": "←",
        "R": "→"
    }

    # Dạng hiển thị
    if use_arrows:
        formatted = " ".join(arrow_map.get(p, p) for p in path)
    else:
        formatted = " ".join(path)

    return f"{formatted}\n(Tổng cộng: {len(path)} bước)"

def print_stats(stats, name="Algorithm"):
    """In tóm tắt stats đẹp."""
    print("=" * 70)
    print(f"KẾT QUẢ TÌM KIẾM: {name}")
    print("=" * 70)
    if stats.get("found"):
        print("✓ Tìm thấy lời giải!")
    else:
        print("✗ Không tìm thấy lời giải.")
    print(f"• Độ dài lời giải: {stats.get('solution_length', 0)}")
    print(f"• Số bước (g_score): {stats.get('g_score', 0)}")
    print(f"• Số node đã duyệt: {stats.get('nodes_explored', 0)}")
    print(f"• Kích thước queue tối đa: {stats.get('max_queue_size', 0)}")
    print(f"• Số state đã lưu (visited): {stats.get('visited_states', 0)}")
    print(f"• Thời gian thực thi: {stats.get('execution_time', 0):.6f} giây")
    print(f"• Bộ nhớ trước khi chạy (RSS): {stats.get('mem_before_mb', 0):.2f} MB")
    print(f"• Bộ nhớ sau khi chạy  (RSS): {stats.get('mem_after_mb', 0):.2f} MB")
    print(f"• Bộ nhớ đang dùng (tracemalloc current): {stats.get('tracemalloc_current_kb', 0):.2f} KB")
    print(f"• Bộ nhớ đỉnh (tracemalloc peak): {stats.get('tracemalloc_peak_kb', 0):.2f} KB")
    if stats.get("limit_hit"):
        print(f"• Lưu ý: giới hạn '{stats['limit_hit']}' đã bị chạm.")
    print("=" * 70)


def bfs_solver(walls, goals, player_start, boxes_start, *,
                     max_time=None, max_nodes=None):
    """
    BFS solver with measurement stats.
    Inputs:
      - walls: set of (r,c)
      - goals: set of (r,c)
      - player_start: (r,c)
      - boxes_start: iterable of (r,c)
    Optional:
      - max_time: seconds (float) to abort early
      - max_nodes: maximum nodes to explore
    Returns: (solution_str_or_None, stats_dict)
    """
    # prepare tracing
    tracemalloc.start()
    proc = psutil.Process(os.getpid())
    mem_before = proc.memory_info().rss / 1024 / 1024  # MB
    start_time = time.time()

    start_state = (player_start, frozenset(boxes_start))
    queue = deque([(start_state, "")])
    visited = {start_state}
    nodes_explored = 0
    max_queue_size = 1

    found = False
    solution = None
    limit_hit = None

    while queue:
        # limits check
        if max_time is not None and (time.time() - start_time) > max_time:
            limit_hit = "max_time"
            break
        if max_nodes is not None and nodes_explored >= max_nodes:
            limit_hit = "max_nodes"
            break

        (player, boxes), path = queue.popleft()
        nodes_explored += 1

        # goal test
        if frozenset(boxes) == frozenset(goals):
            found = True
            solution = path
            break

        # expand neighbors
        for action, (dr, dc) in DIRS.items():
            new_player = (player[0] + dr, player[1] + dc)
            if new_player in walls:
                continue

            new_boxes = set(boxes)
            # push?
            if new_player in boxes:
                new_box = (new_player[0] + dr, new_player[1] + dc)
                if new_box in walls or new_box in boxes:
                    continue
                new_boxes.remove(new_player)
                new_boxes.add(new_box)

            new_state = (new_player, frozenset(new_boxes))
            if new_state not in visited:
                visited.add(new_state)
                queue.append((new_state, path + action))

        max_queue_size = max(max_queue_size, len(queue))

    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    mem_after = proc.memory_info().rss / 1024 / 1024  # MB

    stats = {
        "found": found,
        "solution_length": len(solution) if solution else 0,
        "solution": solution,
        "nodes_explored": nodes_explored,
        "max_queue_size": max_queue_size,
        "visited_states": len(visited),
        "execution_time": end_time - start_time,
        "mem_before_mb": mem_before,
        "mem_after_mb": mem_after,
        "tracemalloc_current_kb": current / 1024,
        "tracemalloc_peak_kb": peak / 1024,
        "limit_hit": limit_hit
    }

    return solution, stats


def heuristic(boxes, goals):
    """Manhattan sum heuristic: sum of min distance per box to any goal."""
    if not boxes or not goals:
        return 0
    total = 0
    goals_list = list(goals)
    for box in boxes:
        min_dist = min(abs(box[0] - g[0]) + abs(box[1] - g[1]) for g in goals_list)
        total += min_dist
    return total


def is_deadlock(box_pos, walls, goals):
    """
    Basic deadlock detection: box in a corner (two perpendicular walls)
    and not on a goal -> deadlock.
    """
    if box_pos in goals:
        return False
    r, c = box_pos
    # corner: vertical neighbor wall AND horizontal neighbor wall
    if ((r - 1, c) in walls or (r + 1, c) in walls) and \
       ((r, c - 1) in walls or (r, c + 1) in walls):
        return True
    return False


def get_next_states(state, walls, goals):
    """
    Generate next states (player, boxes, direction).
    Does basic deadlock pruning for pushed box.
    """
    player, boxes = state
    boxes_set = set(boxes)
    next_states = []

    for direction, (dr, dc) in DIRS.items():
        new_player = (player[0] + dr, player[1] + dc)
        if new_player in walls:
            continue

        if new_player in boxes_set:
            new_box = (new_player[0] + dr, new_player[1] + dc)
            if new_box in walls or new_box in boxes_set:
                continue
            if is_deadlock(new_box, walls, goals):
                continue
            new_boxes = boxes_set - {new_player} | {new_box}
            next_states.append((new_player, frozenset(new_boxes), direction))
        else:
            # move without pushing
            next_states.append((new_player, boxes, direction))

    return next_states


def Astar(walls, goals, player, boxes, *,
                max_time=None, max_nodes=None):
    """
    A* search with statistics. Returns (path_list_or_None, stats_dict).
    path_list is a list of directions: ['U','R',...]
    """
    # prepare tracing
    tracemalloc.start()
    proc = psutil.Process(os.getpid())
    mem_before = proc.memory_info().rss / 1024 / 1024  # MB
    start_time = time.time()

    start_state = (player, frozenset(boxes))
    goals_set = set(goals)

    # priority queue entries: (f_score, g_score, count, state, path_list)
    # count ensures stable ordering
    counter = 0
    h0 = heuristic(boxes, goals_set)
    pq = [(h0, 0, counter, start_state, [])]
    visited_g = {start_state: 0}

    nodes_explored = 0
    max_queue_size = 1
    found = False
    solution = None
    limit_hit = None

    while pq:
        if max_time is not None and (time.time() - start_time) > max_time:
            limit_hit = "max_time"
            break
        if max_nodes is not None and nodes_explored >= max_nodes:
            limit_hit = "max_nodes"
            break

        f_score, g_score, _, current_state, path = heapq.heappop(pq)
        nodes_explored += 1

        # goal check
        if current_state[1] == frozenset(goals_set):
            found = True
            solution = path
            break

        # prune if we already have better g for this state
        if g_score > visited_g.get(current_state, float('inf')):
            continue

        for next_player, next_boxes, direction in get_next_states(current_state, walls, goals_set):
            next_state = (next_player, next_boxes)
            new_g = g_score + 1
            if new_g < visited_g.get(next_state, float('inf')):
                visited_g[next_state] = new_g
                h = heuristic(next_boxes, goals_set)
                counter += 1
                heapq.heappush(pq, (new_g + h, new_g, counter, next_state, path + [direction]))
                max_queue_size = max(max_queue_size, len(pq))

    end_time = time.time()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    mem_after = proc.memory_info().rss / 1024 / 1024  # MB

    stats = {
        "found": found,
        "solution_length": len(solution) if solution else 0,
        "solution": solution,
        "g_score": len(solution) if solution else None,
        "nodes_explored": nodes_explored,
        "max_queue_size": max_queue_size,
        "visited_states": len(visited_g),
        "execution_time": end_time - start_time,
        "mem_before_mb": mem_before,
        "mem_after_mb": mem_after,
        "tracemalloc_current_kb": current / 1024,
        "tracemalloc_peak_kb": peak / 1024,
        "limit_hit": limit_hit
    }

    return solution, stats
