import carla
import random
import time
from planner import plan_route


def draw_route(world, route, life_time=30.0):
    if not route:
        return

    for i, wp in enumerate(route):
        loc = wp.transform.location + carla.Location(z=0.5)

        # 起点红色，终点绿色，中间点黄色
        if i == 0:
            color = carla.Color(255, 0, 0)      # red
            size = 0.16
        elif i == len(route) - 1:
            color = carla.Color(0, 255, 0)      # green
            size = 0.16
        else:
            color = carla.Color(255, 255, 0)    # yellow
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
                color=carla.Color(0, 0, 255),   # 路径线保持蓝色
                life_time=life_time
            )


client = carla.Client("localhost", 2000)
client.set_timeout(5.0)

world = client.get_world()
carla_map = world.get_map()

spawn_points = carla_map.get_spawn_points()
start_point = random.choice(spawn_points)
goal_point = random.choice(spawn_points)

route = plan_route(
    carla_map,
    start_point.location,
    goal_point.location
)

draw_route(world, route)
print("route length:", len(route))

time.sleep(1000)