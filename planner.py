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

    for entry_wp, exit_wp in topology:
        path = [entry_wp]
        current_wp = entry_wp

        # 沿着当前 lane 连续采样，直到接近 exit_wp
        while True:
            dist_to_exit = current_wp.transform.location.distance(
                exit_wp.transform.location
            )
            if dist_to_exit <= sampling_resolution:
                break

            next_wps = current_wp.next(sampling_resolution)
            if not next_wps:
                break

            # 只选择同一条 lane 上最自然的前进点
            next_wp = None
            for cand in next_wps:
                if (cand.road_id == current_wp.road_id and
                    cand.section_id == current_wp.section_id and
                    cand.lane_id == current_wp.lane_id):
                    next_wp = cand
                    break

            if next_wp is None:
                next_wp = next_wps[0]

            # 防止死循环
            if waypoint_id(next_wp) == waypoint_id(current_wp):
                break

            path.append(next_wp)
            current_wp = next_wp

        # 补上 exit_wp
        if waypoint_id(path[-1]) != waypoint_id(exit_wp):
            path.append(exit_wp)

        # 将连续 waypoint 加入图
        for i in range(len(path)):
            wp_id = waypoint_id(path[i])
            nodes[wp_id] = path[i]
            if wp_id not in graph:
                graph[wp_id] = []

        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            a_id = waypoint_id(a)
            b_id = waypoint_id(b)
            cost = waypoint_distance(a, b)
            add_edge(graph, a_id, b_id, cost)

    return graph, nodes


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