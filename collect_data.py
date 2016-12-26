#!/usr/bin/python
import sys
import os
from distutils.dir_util import copy_tree

result_dir = "/home/harada/bin/data/Results"
move_to = "/home/harada/src/OpenHRP3.0/Controller/IOserver/plugin/forceSensorPlugin_Juan/server/generated_failures"
experiment_name = sys.argv[1]

copy_tree(result_dir, os.path.join(move_to, experiment_name))
