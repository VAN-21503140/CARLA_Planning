import carla
import heapq


def waypoint_id(waypoint):
    loc = waypoint.transform.location
    road_id = waypoint.road_id
    section_id = waypoint.section_id
    lane_id = waypoint.lane_id
    s = round(waypoint.s, 2)
    return (road_id, section_id, lane_id, s,
            round(loc.x, 2), round(loc.y, 2), round(loc.z, 2))


def waypoint_distance(wp1, wp2):
    return wp1.transform.location.distance(wp2.transform.location)


def get_closest_node_id(nodes, location):
    best_id = None
    best_dist = float("inf")

    for node_id, waypoint in nodes.items():
        dist = waypoint.transform.location.distance(location)
        if dist < best_dist:
            best_dist = dist
            best_id = node_id

    return best_id


def reconstruct_path(parent, start_id, goal_id):
    if start_id == goal_id:
        return [start_id]

    if goal_id not in parent:
        return []

    path = [goal_id]
    current = goal_id

    while current != start_id:
        current = parent[current]
        path.append(current)

    path.reverse()
    return path


def dijkstra(graph, start_id, goal_id):
    pq = [(0.0, start_id)]
    dist = {start_id: 0.0}
    parent = {}
    visited = set()

    while pq:
        current_dist, current = heapq.heappop(pq)

        if current in visited:
            continue
        visited.add(current)

        if current == goal_id:
            break

        for neighbor, cost in graph.get(current, []):
            new_dist = current_dist + cost

            if neighbor not in dist or new_dist < dist[neighbor]:
                dist[neighbor] = new_dist
                parent[neighbor] = current
                heapq.heappush(pq, (new_dist, neighbor))

    return reconstruct_path(parent, start_id, goal_id)


def add_edge(graph, a_id, b_id, cost):
    if a_id not in graph:
        graph[a_id] = []
    graph[a_id].append((b_id, cost))


from collections import defaultdict


def build_graph(carla_map, sampling_resolution=2.0, lane_change_cost_factor=1.5):
    """
    构建更完整的 CARLA waypoint graph

    返回:
        graph: {
            wp_id: [(neighbor_wp_id, cost), ...],
            ...
        }
        nodes: {
            wp_id: waypoint,
            ...
        }

    设计要点:
    1. 先基于 topology 对每条 segment 做采样，收集尽可能完整的节点
    2. 先连接 segment 内部顺序边
    3. 再对每个节点补充所有 next() 分支，确保路口分支完整
    4. 再补左右变道边
    """

    topology = carla_map.get_topology()

    graph = defaultdict(list)
    nodes = {}
    sampled_paths = []

    # =========================================================
    # Step 1. 基于 topology 对每条 segment 采样
    # 这里只负责“沿当前 segment 走到 exit”
    # 不负责“保留所有路口分支”
    # =========================================================
    for entry_wp, exit_wp in topology:
        path = _sample_segment_path(entry_wp, exit_wp, sampling_resolution)

        if not path:
            continue

        sampled_paths.append(path)

        for wp in path:
            wp_id = waypoint_id(wp)
            nodes[wp_id] = wp
            _ensure_node(graph, wp_id)

    # =========================================================
    # Step 2. 连接每条 sampled segment 内部的顺序边
    # =========================================================
    for path in sampled_paths:
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            a_id = waypoint_id(a)
            b_id = waypoint_id(b)
            cost = waypoint_distance(a, b)
            _add_edge(graph, a_id, b_id, cost)

    # =========================================================
    # Step 3. 对所有节点补“所有 next 分支”
    # 这是保证路口/岔路口不漏分支的关键
    # =========================================================
    # 注意：这里用 list(nodes.items())，避免遍历中途 dict 变化报错
    initial_node_items = list(nodes.items())

    for wp_id, wp in initial_node_items:
        next_wps = wp.next(sampling_resolution)
        for nxt in next_wps:
            nxt_id = waypoint_id(nxt)

            if nxt_id not in nodes:
                nodes[nxt_id] = nxt
                _ensure_node(graph, nxt_id)

            cost = waypoint_distance(wp, nxt)
            _add_edge(graph, wp_id, nxt_id, cost)

    # =========================================================
    # Step 4. 补横向变道边
    # 不强依赖“左右车道 waypoint 必须精确命中已采样节点”
    # 而是去对应 lane 上找最近已存在节点
    # =========================================================
    all_node_items = list(nodes.items())

    for wp_id, wp in all_node_items:
        # 左变道
        left_wp = wp.get_left_lane()
        if left_wp and is_valid_lane_change(wp, left_wp):
            left_id = _find_closest_existing_node_in_lane(nodes, left_wp, max_dist=sampling_resolution * 1.5)
            if left_id is not None and left_id != wp_id:
                lat_cost = waypoint_distance(wp, nodes[left_id]) * lane_change_cost_factor
                _add_edge(graph, wp_id, left_id, lat_cost)

        # 右变道
        right_wp = wp.get_right_lane()
        if right_wp and is_valid_lane_change(wp, right_wp):
            right_id = _find_closest_existing_node_in_lane(nodes, right_wp, max_dist=sampling_resolution * 1.5)
            if right_id is not None and right_id != wp_id:
                lat_cost = waypoint_distance(wp, nodes[right_id]) * lane_change_cost_factor
                _add_edge(graph, wp_id, right_id, lat_cost)
    for wp_id, wp in list(nodes.items()):
        if not wp.is_junction:
            continue

        uturn_target_id = _find_junction_uturn_target(nodes, wp, max_dist=18.0)
        if uturn_target_id is not None and uturn_target_id != wp_id:
            uturn_cost = waypoint_distance(wp, nodes[uturn_target_id]) * 5.0
            _add_edge(graph, wp_id, uturn_target_id, uturn_cost)
    return dict(graph), nodes


# =========================================================
# 辅助函数
# =========================================================
def _find_junction_uturn_target(nodes, wp, max_dist=18.0):
    """
    只允许从 junction 内的点发起掉头。
    目标点不要求也在 junction 内，不强制同 road_id。
    更关注：
    1. 是 Driving 车道
    2. lane_id 符号相反（通常表示对向）
    3. 朝向接近反向
    4. 距离合理
    """
    best_id = None
    best_score = float("inf")

    src_loc = wp.transform.location
    src_yaw = wp.transform.rotation.yaw

    for nid, cand in nodes.items():
        if nid == waypoint_id(wp):
            continue

        if cand.lane_type != carla.LaneType.Driving:
            continue

        # 至少不要还是同一条车道
        if cand.road_id == wp.road_id and cand.lane_id == wp.lane_id:
            continue

        # 优先考虑对向车道
        if wp.lane_id * cand.lane_id >= 0:
            continue

        d = src_loc.distance(cand.transform.location)
        if d > max_dist:
            continue

        yaw_diff = abs((cand.transform.rotation.yaw - src_yaw + 180) % 360 - 180)
        heading_score = abs(yaw_diff - 180.0)

        # 偏好更近、方向更接近 180 度的点
        score = d + heading_score * 0.12

        if score < best_score:
            best_score = score
            best_id = nid

    return best_id
def _sample_segment_path(entry_wp, exit_wp, sampling_resolution):
    """
    沿 topology 中的单个 (entry_wp, exit_wp) segment 进行采样。
    这里仍然只选“一条通向 exit 的路径”，因为它的职责只是把这条 segment 采样出来。
    所有路口分支将在后续 build_graph 的 Step 3 统一补齐。
    """
    path = [entry_wp]
    current_wp = entry_wp
    visited = set()

    while True:
        current_id = waypoint_id(current_wp)
        if current_id in visited:
            break
        visited.add(current_id)

        dist_to_exit = current_wp.transform.location.distance(exit_wp.transform.location)
        if dist_to_exit <= sampling_resolution:
            break

        next_wps = current_wp.next(sampling_resolution)
        if not next_wps:
            break

        # 这里只是为了沿着“当前 topology segment”逼近 exit_wp
        # 不是为了保留全分支
        next_wp = min(
            next_wps,
            key=lambda cand: cand.transform.location.distance(exit_wp.transform.location)
        )

        if waypoint_id(next_wp) == current_id:
            break

        path.append(next_wp)
        current_wp = next_wp

    # 收尾补上 exit_wp
    if waypoint_id(path[-1]) != waypoint_id(exit_wp):
        path.append(exit_wp)

    return _deduplicate_consecutive_waypoints(path)


def _deduplicate_consecutive_waypoints(path):
    """
    去掉 path 中连续重复 waypoint
    """
    if not path:
        return path

    new_path = [path[0]]
    for wp in path[1:]:
        if waypoint_id(wp) != waypoint_id(new_path[-1]):
            new_path.append(wp)
    return new_path


def _ensure_node(graph, wp_id):
    if wp_id not in graph:
        graph[wp_id] = []


def _add_edge(graph, src_id, dst_id, cost, eps=1e-6):
    """
    防重复加边
    """
    for nid, old_cost in graph[src_id]:
        if nid == dst_id and abs(old_cost - cost) < eps:
            return
    graph[src_id].append((dst_id, cost))


def _find_closest_existing_node_in_lane(nodes, target_wp, max_dist=3.0):
    """
    在 nodes 中寻找与 target_wp 属于同一 road/lane，且距离最近的已存在节点
    这样可避免因为采样不完全对齐导致 left_id/right_id 不在 nodes 里
    """
    best_id = None
    best_dist = float("inf")

    target_loc = target_wp.transform.location

    for nid, wp in nodes.items():
        if wp.road_id == target_wp.road_id and wp.lane_id == target_wp.lane_id:
            d = wp.transform.location.distance(target_loc)
            if d < best_dist:
                best_dist = d
                best_id = nid

    if best_dist <= max_dist:
        return best_id
    return None


def is_valid_lane_change(current_wp, neighbor_wp):
    """
    判断是否允许变道到 neighbor_wp
    """
    # 1. 必须在同一条 Road 和 Section 上
    if (current_wp.road_id != neighbor_wp.road_id or
            current_wp.section_id != neighbor_wp.section_id):
        return False

    # 2. 防止逆行：车道 ID 的符号必须相同
    # 在 CARLA 中，正数车道通常表示一个方向，负数表示反方向
    if current_wp.lane_id * neighbor_wp.lane_id <= 0:
        return False

    # 3. (可选) 检查车道类型
    # 避免变道到路肩、人行道或隔离带
    # LaneType 枚举: Driving=1, Stop=4, Sidewalk=15 等
    if neighbor_wp.lane_type != carla.LaneType.Driving:
        return False

    return True

def plan_route(carla_map, start_location, goal_location, sampling_resolution=2.0):
    graph, nodes = build_graph(carla_map, sampling_resolution=sampling_resolution)

    # 先把起终点投影到道路上
    start_wp = carla_map.get_waypoint(start_location, project_to_road=True)
    goal_wp = carla_map.get_waypoint(goal_location, project_to_road=True)

    if start_wp is None or goal_wp is None:
        return []

    # 在连续图中找最近节点
    start_id = get_closest_node_id(nodes, start_wp.transform.location)
    goal_id = get_closest_node_id(nodes, goal_wp.transform.location)

    if start_id is None or goal_id is None:
        return []

    path_ids = dijkstra(graph, start_id, goal_id)

    if not path_ids:
        return []

    route = [nodes[node_id] for node_id in path_ids]
    return route
#可以变道