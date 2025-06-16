import pandas
from spatialgeometry import Sphere
import numpy as np
import lib.callbacks as call
from spatialmath import SO3, SE3
import roboticstoolbox as rtb
import time
from lib.rrtStar import find_tree
from lib.callbacks import Node
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-i", help="path to input file with restricted areas.")
parser.add_argument("-o", help="path to output file with generated points.")
args = parser.parse_args()

PATH_IN = call.handle_path("restricted_area.csv" if args.i is None else args.i)
PATH_OUT = call.handle_path("points.csv" if args.o is None else args.o)

df = pandas.read_csv(PATH_IN)
panda_qr = rtb.models.Panda().fkine(rtb.models.Panda().qr).t
panda_qr = np.array([panda_qr[0], panda_qr[1], panda_qr[2]-0.2])
limits = [[-0.6, 0.6], [-0.6, 0.6], [0.0, 0.8]]
resources = {"radius":                  0.04,
             "point_to_check_color":    (0, 0, 0),
             "map_limits":              [-2, 2, -2, 2,-1, 3],
            #  "start_loc":               [np.random.uniform(limits[index][0], limits[index][1]) for index, _ in enumerate(limits)],
            #  "start_loc":               [-0.16, -0.1, 0.67],
            #  "start_loc":               [0.1, -0.3, 0.5],
             "start_loc":               panda_qr,
             "start_color":             (0, 255, 0),
             "dest_color":              (0, 0, 255),
            #  "dest_loc":                [np.random.uniform(limits[index][0], limits[index][1]) for index,_ in enumerate(limits)],
            #  "dest_loc":                [0.26, 0.40, 0.69],
            #  "dest_loc":                [-0.35, -0.35, 0.40],
             "dest_loc":                [-0.19715782, -0.04114254,  0.79557244],
             "iterations":              10,
             "box_info":                df.values,
             "limits":                  limits}

objects, env = call.setup_env(panda = True,
                              start=True, 
                              dest = True, 
                              boxes = True,
                              resources = resources)

print(f'Start point: {objects["start"].T[:3,3]}, dest point: {objects["dest"].T[:3,3]}.')

current = Sphere(radius=resources["radius"], color=(255,255,0))

path, spheres, _ = find_tree(env,
                             resources["limits"],
                             objects,
                             step_size=0.1,
                             rotation_limits=[[-np.pi*3/2, np.pi*3/2], [-np.pi*3/2, np.pi*3/2], [-np.pi*3/2, np.pi*3/2]])

headers = [f'j_{joint}' for joint in range(0, len(objects["panda"].q))]
call.generate_csv(PATH_OUT, headers=headers, array=[path[i].q for i in range(len(path))])
print(f'Path found with {len(path)} nodes within {len(spheres)} vertices.')
# print("Press Enter to close the window.")
# input()

env.close()
del env