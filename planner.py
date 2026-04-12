import carla
import heapq
from collections import defaultdict


class CarlaGraphPlanner:
    """
    基于 CARLA waypoint 构建有向图，并进行路径规划。

    设计目标：
    1. 初始化时一次性建图
    2. 后续支持多次 route planning
    3. 保留：
       - topology 采样
       - segment 顺序边
       - next() 分支补全
       - 左右变道边
       - junction 内掉头边
    """

    def __init__(
        self,
        carla_map,
        sampling_resolution=2.0,
        lane_change_cost_factor=1.5,
        uturn_cost_factor=5.0,
        enable_lane_change=True,
        enable_uturn=True,
    ):
        self.carla_map = carla_map
        self.sampling_resolution = sampling_resolution
        self.lane_change_cost_factor = lane_change_cost_factor
        self.uturn_cost_factor = uturn_cost_factor
        self.enable_lane_change = enable_lane_change
        self.enable_uturn = enable_uturn

        self.graph = defaultdict(list)   # {node_id: [(neighbor_id, cost), ...]}
        self.nodes = {}                  # {node_id: waypoint}
        self._lane_index = defaultdict(list)

        self.build_graph()

    # =========================================================
    # 对外接口
    # =========================================================
    def rebuild_graph(self):
        """外部可手动重建图。"""
        self.graph.clear()
        self.nodes.clear()
        self._lane_index.clear()
        self.build_graph()

    def plan(self, start_location, goal_location):
        """
        输入起点/终点 location，返回 waypoint route。
        """
        start_wp = self.carla_map.get_waypoint(start_location, project_to_road=True)
        goal_wp = self.carla_map.get_waypoint(goal_location, project_to_road=True)

        if start_wp is None or goal_wp is None:
            return []

        start_id = self._get_closest_node_id(start_wp.transform.location, hint_wp=start_wp)
        goal_id = self._get_closest_node_id(goal_wp.transform.location, hint_wp=goal_wp)

        if start_id is None or goal_id is None:
            return []

        path_ids = self._astar(start_id, goal_id)
        if not path_ids:
            return []

        return [self.nodes[node_id] for node_id in path_ids]

    def debug_summary(self):
        edge_count = sum(len(v) for v in self.graph.values())
        return {
            "num_nodes": len(self.nodes),
            "num_edges": edge_count,
            "sampling_resolution": self.sampling_resolution,
            "lane_change_enabled": self.enable_lane_change,
            "uturn_enabled": self.enable_uturn,
        }

    # =========================================================
    # 图构建主流程
    # =========================================================
    def build_graph(self):
        """
        构建完整 waypoint graph。
        """
        topology = self.carla_map.get_topology()
        sampled_paths = []

        # Step 1. 采样 topology segment
        for entry_wp, exit_wp in topology:
            path = self._sample_segment_path(entry_wp, exit_wp)
            if not path:
                continue

            sampled_paths.append(path)

            for wp in path:
                wp_id = self._waypoint_id(wp)
                self.nodes[wp_id] = wp
                self._ensure_node(wp_id)

        # Step 2. 连接每条 sampled path 内部顺序边
        self._connect_sampled_paths(sampled_paths)

        # Step 3. 对所有节点补 next() 分支
        self._connect_next_branches()

        # Step 4. 建 lane index，便于变道找目标点
        self._build_lane_index()

        # Step 5. 补左右变道边
        if self.enable_lane_change:
            self._connect_lane_changes()

        # Step 6. 补 junction 内掉头边
        if self.enable_uturn:
            self._connect_uturns()

    # =========================================================
    # Step 1: 采样 topology segment
    # =========================================================
    def _sample_segment_path(self, entry_wp, exit_wp):
        """
        沿 topology 中的单个 (entry_wp, exit_wp) segment 采样。
        这里只负责采样出“当前 segment 的一条逼近 exit 的路径”。
        所有路口全分支在后续统一通过 next() 补齐。
        """
        path = [entry_wp]
        current_wp = entry_wp
        visited = set()

        while True:
            current_id = self._waypoint_id(current_wp)
            if current_id in visited:
                break
            visited.add(current_id)

            dist_to_exit = current_wp.transform.location.distance(exit_wp.transform.location)
            if dist_to_exit <= self.sampling_resolution:
                break

            next_wps = current_wp.next(self.sampling_resolution)
            if not next_wps:
                break

            # 这里只选最接近 exit_wp 的一个，保留 segment 主路径
            next_wp = min(
                next_wps,
                key=lambda cand: cand.transform.location.distance(exit_wp.transform.location)
            )

            if self._waypoint_id(next_wp) == current_id:
                break

            path.append(next_wp)
            current_wp = next_wp

        # 若尾点还不是 exit_wp，则补上
        if self._waypoint_id(path[-1]) != self._waypoint_id(exit_wp):
            # 避免出现特别离谱的长边
            if path[-1].transform.location.distance(exit_wp.transform.location) <= self.sampling_resolution * 1.5:
                path.append(exit_wp)
            else:
                # 仍然补上 exit_wp，尽量与原逻辑一致
                path.append(exit_wp)

        return self._deduplicate_consecutive_waypoints(path)

    # =========================================================
    # Step 2: 连接采样路径顺序边
    # =========================================================
    def _connect_sampled_paths(self, sampled_paths):
        for path in sampled_paths:
            for i in range(len(path) - 1):
                a = path[i]
                b = path[i + 1]
                a_id = self._waypoint_id(a)
                b_id = self._waypoint_id(b)
                cost = self._waypoint_distance(a, b)
                self._add_edge(a_id, b_id, cost)

    # =========================================================
    # Step 3: next() 全分支补齐
    # =========================================================
    def _connect_next_branches(self):
        initial_node_items = list(self.nodes.items())

        for wp_id, wp in initial_node_items:
            next_wps = wp.next(self.sampling_resolution)

            for nxt in next_wps:
                nxt_id = self._waypoint_id(nxt)

                if nxt_id not in self.nodes:
                    self.nodes[nxt_id] = nxt
                    self._ensure_node(nxt_id)

                cost = self._waypoint_distance(wp, nxt)
                self._add_edge(wp_id, nxt_id, cost)

    # =========================================================
    # Step 4: lane index
    # =========================================================
    def _build_lane_index(self):
        self._lane_index.clear()
        for node_id, wp in self.nodes.items():
            key = (wp.road_id, wp.section_id, wp.lane_id)
            self._lane_index[key].append(node_id)

    # =========================================================
    # Step 5: 左右变道边
    # =========================================================
    def _connect_lane_changes(self):
        all_node_items = list(self.nodes.items())

        for wp_id, wp in all_node_items:
            # 左变道
            left_wp = wp.get_left_lane()
            if left_wp and self._is_valid_lane_change(wp, left_wp, direction="left"):
                left_id = self._find_closest_existing_node_in_lane(
                    left_wp,
                    max_dist=self.sampling_resolution * 1.5
                )
                if left_id is not None and left_id != wp_id:
                    lat_cost = self._waypoint_distance(wp, self.nodes[left_id]) * self.lane_change_cost_factor
                    self._add_edge(wp_id, left_id, lat_cost)

            # 右变道
            right_wp = wp.get_right_lane()
            if right_wp and self._is_valid_lane_change(wp, right_wp, direction="right"):
                right_id = self._find_closest_existing_node_in_lane(
                    right_wp,
                    max_dist=self.sampling_resolution * 1.5
                )
                if right_id is not None and right_id != wp_id:
                    lat_cost = self._waypoint_distance(wp, self.nodes[right_id]) * self.lane_change_cost_factor
                    self._add_edge(wp_id, right_id, lat_cost)

    # =========================================================
    # Step 6: junction 内掉头边
    # =========================================================
    def _connect_uturns(self):
        for wp_id, wp in list(self.nodes.items()):
            if not wp.is_junction:
                continue

            uturn_target_id = self._find_junction_uturn_target(wp, max_dist=18.0)
            if uturn_target_id is not None and uturn_target_id != wp_id:
                uturn_cost = self._waypoint_distance(wp, self.nodes[uturn_target_id]) * self.uturn_cost_factor
                self._add_edge(wp_id, uturn_target_id, uturn_cost)

    # =========================================================
    # 搜索算法：A*
    # =========================================================
    def _astar(self, start_id, goal_id):
        if start_id == goal_id:
            return [start_id]

        pq = [(0.0, start_id)]
        g_cost = {start_id: 0.0}
        parent = {}
        visited = set()

        while pq:
            _, current = heapq.heappop(pq)

            if current in visited:
                continue
            visited.add(current)

            if current == goal_id:
                break

            for neighbor, edge_cost in self.graph.get(current, []):
                tentative_g = g_cost[current] + edge_cost

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g
                    parent[neighbor] = current
                    f = tentative_g + self._heuristic(neighbor, goal_id)
                    heapq.heappush(pq, (f, neighbor))

        return self._reconstruct_path(parent, start_id, goal_id)

    def _heuristic(self, a_id, b_id):
        return self.nodes[a_id].transform.location.distance(
            self.nodes[b_id].transform.location
        )

    def _reconstruct_path(self, parent, start_id, goal_id):
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

    # =========================================================
    # 节点 / 边 / 几何辅助
    # =========================================================
    def _waypoint_id(self, waypoint):
        """
        用 road 参考系 + 量化 s 作为节点标识。
        比直接把 xyz 全塞进去更稳定。
        """
        return (
            waypoint.road_id,
            waypoint.section_id,
            waypoint.lane_id,
            round(waypoint.s, 2),
        )

    def _waypoint_distance(self, wp1, wp2):
        return wp1.transform.location.distance(wp2.transform.location)

    def _ensure_node(self, wp_id):
        if wp_id not in self.graph:
            self.graph[wp_id] = []

    def _add_edge(self, src_id, dst_id, cost, eps=1e-6):
        for nid, old_cost in self.graph[src_id]:
            if nid == dst_id and abs(old_cost - cost) < eps:
                return
        self.graph[src_id].append((dst_id, cost))

    def _deduplicate_consecutive_waypoints(self, path):
        if not path:
            return path

        new_path = [path[0]]
        for wp in path[1:]:
            if self._waypoint_id(wp) != self._waypoint_id(new_path[-1]):
                new_path.append(wp)
        return new_path

    # =========================================================
    # 最近节点查找
    # =========================================================
    def _get_closest_node_id(self, location, hint_wp=None):
        """
        优先在同 lane 内找最近节点；找不到再全局搜索。
        """
        best_id = None
        best_dist = float("inf")

        # 优先用车道索引缩小搜索范围
        candidate_ids = None
        if hint_wp is not None:
            lane_key = (hint_wp.road_id, hint_wp.section_id, hint_wp.lane_id)
            if lane_key in self._lane_index and self._lane_index[lane_key]:
                candidate_ids = self._lane_index[lane_key]

        if candidate_ids is None:
            candidate_items = self.nodes.items()
        else:
            candidate_items = ((nid, self.nodes[nid]) for nid in candidate_ids)

        for node_id, waypoint in candidate_items:
            dist = waypoint.transform.location.distance(location)
            if dist < best_dist:
                best_dist = dist
                best_id = node_id

        return best_id

    def _find_closest_existing_node_in_lane(self, target_wp, max_dist=3.0):
        """
        在同 road/section/lane 的已存在节点中，找离 target_wp 最近的节点。
        """
        best_id = None
        best_dist = float("inf")

        key = (target_wp.road_id, target_wp.section_id, target_wp.lane_id)
        candidate_ids = self._lane_index.get(key, [])

        target_loc = target_wp.transform.location
        for nid in candidate_ids:
            wp = self.nodes[nid]
            d = wp.transform.location.distance(target_loc)
            if d < best_dist:
                best_dist = d
                best_id = nid

        if best_dist <= max_dist:
            return best_id
        return None

    # =========================================================
    # 变道逻辑
    # =========================================================
    def _is_valid_lane_change(self, current_wp, neighbor_wp, direction):
        """
        判断是否允许从 current_wp 变道到 neighbor_wp。
        """
        if neighbor_wp is None:
            return False

        # 1. 必须在同一条 Road + Section
        if (current_wp.road_id != neighbor_wp.road_id or
                current_wp.section_id != neighbor_wp.section_id):
            return False

        # 2. 防止跨到对向
        if current_wp.lane_id * neighbor_wp.lane_id <= 0:
            return False

        # 3. 只允许进入 Driving 车道
        if neighbor_wp.lane_type != carla.LaneType.Driving:
            return False

        # 4. 根据 lane_change 标志判断是否允许
        lane_change = current_wp.lane_change

        if direction == "left":
            return lane_change in (carla.LaneChange.Left, carla.LaneChange.Both)
        elif direction == "right":
            return lane_change in (carla.LaneChange.Right, carla.LaneChange.Both)

        return False

    # =========================================================
    # 掉头逻辑
    # =========================================================
    def _find_junction_uturn_target(self, wp, max_dist=18.0):
        """
        只允许从 junction 内的点发起掉头。

        目标点筛选标准：
        1. Driving 车道
        2. 不是当前同一条车道
        3. lane_id 符号相反（通常为对向）
        4. 距离在 max_dist 内
        5. 朝向尽量接近 180 度相反
        """
        best_id = None
        best_score = float("inf")

        src_loc = wp.transform.location
        src_yaw = wp.transform.rotation.yaw
        src_id = self._waypoint_id(wp)
        if not wp.is_junction:
            return None
        for nid, cand in self.nodes.items():
            if nid == src_id:
                continue

            if cand.lane_type != carla.LaneType.Driving:
                continue

            # 至少不能还是原车道
            if cand.road_id == wp.road_id and cand.lane_id == wp.lane_id:
                continue

            # 偏向对向车道
            if wp.lane_id * cand.lane_id >= 0:
                continue

            d = src_loc.distance(cand.transform.location)
            if d > max_dist:
                continue

            yaw_diff = abs((cand.transform.rotation.yaw - src_yaw + 180) % 360 - 180)
            heading_score = abs(yaw_diff - 180.0)

            # 越近越好，越接近反向越好
            score = d + heading_score * 0.12

            if score < best_score:
                best_score = score
                best_id = nid

        return best_id
