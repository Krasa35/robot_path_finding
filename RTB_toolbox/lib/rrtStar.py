import numpy as np
from spatialgeometry import Sphere
from spatialmath import SO3
from lib.callbacks import update_obj, Node, COLL_BOX_RADIUS_MULTIPLIER, COLL_ROBOT_RADIUS_MULTIPLIER
from lib.robot import validate_robot

def _distance(a : Node, b : Node):
    return np.linalg.norm(a.pose() - b.pose())

def _is_collision(pos : Node, objects):
    """
    pos: [x, y, z] list or numpy array
    objects: dictionary returned from your setup
    """
    probe_sphere_box = Sphere(radius=objects["start"].radius * COLL_BOX_RADIUS_MULTIPLIER)
    probe_sphere_robot = Sphere(radius=objects["start"].radius * COLL_ROBOT_RADIUS_MULTIPLIER)
    update_obj(probe_sphere_box, pos.pose())
    update_obj(probe_sphere_robot, pos.pose())

    # Check with Panda
    if "panda" in objects and objects["panda"].iscollided(objects["panda"].q, probe_sphere_robot):
        return True

    # Check with Boxes
    if "box" in objects:
        for b in objects["box"]:
            if probe_sphere_box.iscollided(b):
                return True

    return False

def _is_connection_in_collision(p1 : Node, p2 : Node, objects, resolution=0.05):
    """
    p1, p2: start and end positions (e.g., [x, y, z])
    resolution: distance between points to check
    """
    dist = _distance(p1, p2)
    steps = int(dist / resolution)
    if steps == 0:
        return _is_collision(p1, objects) or _is_collision(p2, objects)
    for i in range(steps + 1):
        t = i / steps
        pos = Node(
            (1 - t) * p1.x + t * p2.x,
            (1 - t) * p1.y + t * p2.y,
            (1 - t) * p1.z + t * p2.z
        )
        if _is_collision(pos, objects):
            return True
    return False

def _sample_free(bounds, dest_node, c_best, c_min, start_node, ) -> Node:
    x = np.random.uniform(bounds[0][0], bounds[0][1])
    y = np.random.uniform(bounds[1][0], bounds[1][1])
    z = np.random.uniform(bounds[2][0], bounds[2][1])
    return Node(x, y, z, start_node.rot_matrix)    
    
def _get_nearest(nodes : list, node) -> Node:
    parent = min(nodes, key=lambda n: _distance(n, node))
    return parent

def _steer(from_node, to_node, step_size) -> Node:
    dist = _distance(from_node, to_node)
    if dist <= step_size:
        return Node(to_node.x, to_node.y, to_node.z, to_node.rot_matrix, from_node, from_node.cost + dist)
    else:
        direction = np.array([to_node.x - from_node.x, to_node.y - from_node.y, to_node.z - from_node.z])
        norm = np.linalg.norm(direction)
        direction = direction / norm
        new_x = from_node.x + step_size * direction[0]
        new_y = from_node.y + step_size * direction[1]
        new_z = from_node.z + step_size * direction[2]
        return Node(new_x, new_y, new_z, to_node.rot_matrix, from_node, from_node.cost + step_size)

def _show_path(env, end_node, radius = 0.04):
    path_spheres = []
    node = end_node
    while node:
        path_spheres.append(Sphere(radius=radius, color=(150, 150, 0)))
        update_obj(path_spheres[-1], [node.x, node.y, node.z])
        env.add(path_spheres[-1])
        node = node.parent
    return path_spheres

def _find_robot_path(env, end_node, objects, radius = 0.04, rotation_limits=None):
    path_nodes = []
    node = end_node
    node = validate_robot(env, objects, node, node)
    while node:
        path_nodes.append(node)
        sphere = Sphere(radius=radius, color=(150, 150, 0))
        update_obj(sphere, node.pose())
        env.add(sphere)
        prev_node = node
        node = node.parent
        if node is not None:
            node = validate_robot(env, objects, node, prev_node, rotation_limits=rotation_limits)

    return path_nodes[::-1]

def find_tree(env, bounds, objects, step_size=0.1, rotation_limits=None):
    """
    Attempts to generate a new position for the robot, checking for collisions and proximity to the destination.
    Updates Temp if a better, collision-free position is found.
    Returns the updated Temp.
    """
    # np.random.seed(40)

    start_node = Node(objects["start"].T[0,3], objects["start"].T[1,3], objects["start"].T[2,3], SO3(objects["panda"].fkine(objects["panda"].q).R))
    dest_node = Node(objects["dest"].T[0,3], objects["dest"].T[1,3], objects["dest"].T[2,3], SO3(objects["panda"].fkine(objects["panda"].q).R))

    nodes = [start_node]
    spheres = [objects["start"]]
    best_goal_node = None
    c_best = float('inf')
    c_min = _distance(start_node, dest_node)

    while not objects["dest"].iscollided(spheres[-1]):
        if best_goal_node:
            new_node = _steer(best_goal_node, dest_node, step_size)
        else:
            rnd_node = _sample_free(bounds, dest_node, c_best, c_min, start_node)
            nearest = _get_nearest(nodes, rnd_node)
            new_node = _steer(nearest, rnd_node, step_size)
        if not _is_collision(new_node, objects) and not _is_connection_in_collision(nearest, new_node, objects):
            nodes.append(new_node)
            spheres.append(Sphere(radius=objects["start"].radius, color=(150, 150, 150)))
            update_obj(spheres[-1], new_node.pose())
            env.add(spheres[-1])
            if _distance(new_node, dest_node) <= 0.25 and not _is_connection_in_collision(new_node, dest_node, objects):
                best_goal_node = new_node
                c_best = best_goal_node.cost

    dest_node = Node(dest_node.x, dest_node.y, dest_node.z, dest_node.rot_matrix, best_goal_node, best_goal_node.cost + _distance(best_goal_node, dest_node), objects["panda"].qr)
    nodes.append(dest_node)
    best_goal_node = dest_node
    if best_goal_node == dest_node:
        # path_nodes = _show_path(env, best_goal_node, radius=objects["start"].radius)
        path_nodes = _find_robot_path(env, best_goal_node, objects, radius=objects["start"].radius, rotation_limits=rotation_limits)
        return path_nodes, spheres, nodes

    print("Valid path not found.")
    return None

