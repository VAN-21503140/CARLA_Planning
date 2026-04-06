import carla
import math
import heapq


def waypoint_id(waypoint):
    loc = waypoint.transform.location
    return (round(loc.x, 2), round(loc.y, 2), round(loc.z, 2))


def waypoint_distance(wp1, wp2):
    return wp1.transform.location.distance(wp2.transform.location)


def build_graph(carla_map, sampling_resolution=2.0):
    topology = carla_map.get_topology()

    graph = {}
    nodes = {}

    for entry_wp, exit_wp in topology:
        entry_id = waypoint_id(entry_wp)
        exit_id = waypoint_id(exit_wp)

        nodes[entry_id] = entry_wp
        nodes[exit_id] = exit_wp

        if entry_id not in graph:
            graph[entry_id] = []
        if exit_id not in graph:
            graph[exit_id] = []

        cost = waypoint_distance(entry_wp, exit_wp)
        graph[entry_id].append((exit_id, cost))

    return graph, nodes


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
    if goal_id not in parent and goal_id != start_id:
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


def plan_route(carla_map, start_location, goal_location, sampling_resolution=2.0):
    graph, nodes = build_graph(carla_map, sampling_resolution=sampling_resolution)

    start_id = get_closest_node_id(nodes, start_location)
    goal_id = get_closest_node_id(nodes, goal_location)

    path_ids = dijkstra(graph, start_id, goal_id)

    if not path_ids:
        return []

    route = [nodes[node_id] for node_id in path_ids]
    return route