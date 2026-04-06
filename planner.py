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


def build_graph(carla_map, sampling_resolution=2.0):
    topology = carla_map.get_topology()
    graph = {}
    nodes = {}

    # 第一步：构建纵向路径（保持你原有的逻辑，稍微优化了循环）
    for entry_wp, exit_wp in topology:
        path = [entry_wp]
        current_wp = entry_wp

        while True:
            dist_to_exit = current_wp.transform.location.distance(exit_wp.transform.location)
            if dist_to_exit <= sampling_resolution:
                break

            next_wps = current_wp.next(sampling_resolution)
            if not next_wps:
                break

            # 简单的防死循环和同车道检查
            next_wp = next_wps[0]
            if waypoint_id(next_wp) == waypoint_id(current_wp):
                break

            path.append(next_wp)
            current_wp = next_wp

        # 补上 exit_wp
        if len(path) == 0 or waypoint_id(path[-1]) != waypoint_id(exit_wp):
            path.append(exit_wp)

        # 将点加入节点字典
        for wp in path:
            wp_id = waypoint_id(wp)
            nodes[wp_id] = wp
            if wp_id not in graph:
                graph[wp_id] = []

        # 添加纵向边（前后连接）
        for i in range(len(path) - 1):
            a_id = waypoint_id(path[i])
            b_id = waypoint_id(path[i + 1])
            cost = waypoint_distance(path[i], path[i + 1])
            graph[a_id].append((b_id, cost))

    # ==========================================
    # 第二步：新增横向连接（变道逻辑）
    # ==========================================
    # 遍历图中所有的节点，尝试连接相邻车道
    for wp_id, wp in nodes.items():
        # 检查左侧车道
        left_wp = wp.get_left_lane()
        if left_wp and is_valid_lane_change(wp, left_wp):
            left_id = waypoint_id(left_wp)
            # 如果左侧车道的点也在我们的图中（通常都在），则建立双向连接
            if left_id in nodes:
                # 计算横向移动的成本（通常比纵向行驶成本高，以鼓励少变道）
                lat_cost = waypoint_distance(wp, left_wp) * 1.5

                # 当前 -> 左侧
                graph[wp_id].append((left_id, lat_cost))
                # 左侧 -> 当前 (允许变回来)
                if left_id not in graph: graph[left_id] = []
                graph[left_id].append((wp_id, lat_cost))

        # 检查右侧车道
        right_wp = wp.get_right_lane()
        if right_wp and is_valid_lane_change(wp, right_wp):
            right_id = waypoint_id(right_wp)
            if right_id in nodes:
                lat_cost = waypoint_distance(wp, right_wp) * 1.5

                # 当前 -> 右侧
                graph[wp_id].append((right_id, lat_cost))
                # 右侧 -> 当前
                if right_id not in graph: graph[right_id] = []
                graph[right_id].append((wp_id, lat_cost))

    return graph, nodes


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