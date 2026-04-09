import carla
import random
import time
from planner import CarlaGraphPlanner


def draw_route(world, route, life_time=30.0):
    if not route:
        print("No route found.")
        return

    for i, wp in enumerate(route):
        loc = wp.transform.location + carla.Location(z=0.5)

        # 起点红色，终点绿色，中间点黄色
        if i == 0:
            color = carla.Color(255, 0, 0)
            size = 0.16
        elif i == len(route) - 1:
            color = carla.Color(0, 255, 0)
            size = 0.16
        else:
            color = carla.Color(255, 255, 0)
            size = 0.12

        world.debug.draw_point(
            loc,
            size=size,
            color=color,
            life_time=life_time
        )

        if i < len(route) - 1:
            next_loc = route[i + 1].transform.location + carla.Location(z=0.5)
            world.debug.draw_line(
                loc,
                next_loc,
                thickness=0.08,
                color=carla.Color(0, 0, 255),
                life_time=life_time
            )


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    carla_map = world.get_map()

    # 1. 创建规划器（这里会建图）
    planner = CarlaGraphPlanner(
        carla_map,
        sampling_resolution=2.0,
        lane_change_cost_factor=1.5,
        uturn_cost_factor=5.0,
        enable_lane_change=True,
        enable_uturn=True,
    )

    print("Planner summary:", planner.debug_summary())

    # 2. 随机选起点终点
    spawn_points = carla_map.get_spawn_points()
    if len(spawn_points) < 2:
        print("Not enough spawn points.")
        return

    start_point = random.choice(spawn_points)
    goal_point = random.choice(spawn_points)

    # 防止起终点一样
    while goal_point.location.distance(start_point.location) < 5.0:
        goal_point = random.choice(spawn_points)

    print("Start:", start_point.location)
    print("Goal :", goal_point.location)

    # 3. 规划路径
    route = planner.plan(
        start_point.location,
        goal_point.location
    )

    print("Route length:", len(route))

    # 4. 可视化
    draw_route(world, route, life_time=60.0)

    # 额外把起终点原始位置也画出来，方便对比
    world.debug.draw_point(
        start_point.location + carla.Location(z=1.0),
        size=0.2,
        color=carla.Color(255, 0, 255),
        life_time=60.0
    )
    world.debug.draw_point(
        goal_point.location + carla.Location(z=1.0),
        size=0.2,
        color=carla.Color(0, 255, 255),
        life_time=60.0
    )

    time.sleep(1000)


if __name__ == "__main__":
    main()