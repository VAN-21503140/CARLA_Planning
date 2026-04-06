import carla
import random
import time
from planner import plan_route


client = carla.Client("localhost", 2000)
client.set_timeout(5.0)

world = client.get_world()
carla_map = world.get_map()


def densify_route(route, step=2.0):
    if len(route) < 2:
        return route

    dense_route = []

    for i in range(len(route) - 1):
        current_wp = route[i]
        target_wp = route[i + 1]

        dense_route.append(current_wp)
        temp_wp = current_wp

        for _ in range(200):
            next_wps = temp_wp.next(step)
            if not next_wps:
                break

            temp_wp = min(
                next_wps,
                key=lambda wp: wp.transform.location.distance(
                    target_wp.transform.location
                )
            )

            if temp_wp.transform.location.distance(target_wp.transform.location) < step:
                break

            dense_route.append(temp_wp)

    dense_route.append(route[-1])
    return dense_route


def draw_sparse_route(world, route, life_time=30.0):
    for wp in route:
        loc = wp.transform.location + carla.Location(z=1.0)
        world.debug.draw_point(
            loc,
            size=0.2,
            color=carla.Color(255, 0, 0),
            life_time=life_time
        )


def draw_dense_route(world, route, life_time=30.0):
    for i, wp in enumerate(route):
        loc = wp.transform.location + carla.Location(z=0.5)

        world.debug.draw_point(
            loc,
            size=0.12,
            color=carla.Color(0, 255, 0),
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


spawn_points = carla_map.get_spawn_points()
start_point = random.choice(spawn_points)
goal_point = random.choice(spawn_points)

while goal_point.location.distance(start_point.location) < 30.0:
    goal_point = random.choice(spawn_points)

route = plan_route(
    carla_map,
    start_point.location,
    goal_point.location
)

print("start:", start_point.location)
print("goal:", goal_point.location)
print("route length:", len(route))

if not route:
    print("No valid route found.")
else:
    dense_route = densify_route(route, step=2.0)
    print("dense route length:", len(dense_route))

    # 起点终点标记
    world.debug.draw_point(
        start_point.location + carla.Location(z=1.5),
        size=0.25,
        color=carla.Color(255, 255, 0),
        life_time=30.0
    )
    world.debug.draw_point(
        goal_point.location + carla.Location(z=1.5),
        size=0.25,
        color=carla.Color(255, 0, 255),
        life_time=30.0
    )

    # 红点：原始 Dijkstra 路径
    draw_sparse_route(world, route, life_time=30.0)

    # 绿点+蓝线：加密后的路径
    draw_dense_route(world, dense_route, life_time=30.0)

    time.sleep(10)