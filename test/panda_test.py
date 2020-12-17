import os
import unittest
import pymongo as pm
import yaml

from ropod.structs.task import TaskRequest
from task_planner.panda_interface import PANDAInterface

def get_planner_config(config_file_path):
    planner_config_params = None
    with open(config_file_path, 'r') as config_file:
        planner_config_params = yaml.load(config_file)
    return planner_config_params


test_kb_name = 'test_robot_transport'

planner_config_params = get_planner_config('/mnt/DATEN/Dokumente/Master_Autonomous_Systems/3rd_Semester/Software_Development_Project/HDDL_Parser/Python_Approach/Code/task-planner/config/panda_config.yaml')
domain_file = planner_config_params['domain_file']
planner_cmd = planner_config_params['planner_cmd']
plan_file_path = "planner_output/"
planner_interface = PANDAInterface(test_kb_name, domain_file, planner_cmd, plan_file_path, debug=True)

task_request = TaskRequest()
#task_request.load_id = 'mobidik'
#task_request.delivery_pose.id = 'DELIVERY_LOCATION'
task_goals = []
retval, plan = planner_interface.plan(task_request, 'frank', task_goals)

#output results
print("\n\nAction Names:")
for action in plan:
    print(action.type)