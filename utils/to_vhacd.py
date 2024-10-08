import pybullet as p
import sys

obj_name = sys.argv[1]
output_obj_name = sys.argv[1][:-4] + "_v.obj"

p.vhacd(obj_name, output_obj_name,
    'vhacd_log.txt',)