import os
import unittest
import pymongo as pm
import yaml

from ropod.structs.task import TaskRequest
from task_planner.panda_interface import PANDAInterface

def get_planner_config(config_file_path):
    planner_config_params = None
    with open(config_file_path, 'r') as config_file:
        planner_config_params = yaml.safe_load(config_file)
    return planner_config_params

def get_db_host_and_port():
    host = 'localhost'
    port = 27017
    if 'DB_HOST' in os.environ:
        host = os.environ['DB_HOST']
    if 'DB_PORT' in os.environ:
        port = int(os.environ['DB_PORT'])
    return (host, port)

# sudo service mongod start
test_kb_name = 'test_PANDA_Domestic'
host, port = get_db_host_and_port()
client = pm.MongoClient(host=host, port=port)

planner_config_params = get_planner_config('/mnt/DATEN/Dokumente/Master_Autonomous_Systems/3rd_Semester/Software_Development_Project/HDDL_Parser/Python_Approach/Code/task-planner/config/panda_config.yaml')
domain_file = planner_config_params['domain_file']
planner_cmd = planner_config_params['planner_cmd']
plan_file_path = "planner_output/"
planner_interface = PANDAInterface(test_kb_name, domain_file, planner_cmd, plan_file_path, debug=True)

#Knowledge--------------------------------------------------------------------
predicates = [('robotName',[('Robot','Frank')]),
              ('emptyGripper',[('Robot','Frank')]),
              ('furnitureAt',[('Furniture','Fridge'),('Waypoint','Kitchen')]),
              ('hasDoor',[('Furniture', 'Fridge')]),
              ('doorAt',[('Door','Fridge_Door'),('Waypoint','Kitchen')]),
              ('inside',[('Object0','Beer'),('Object1','Fridge')]),
              ('known',[('Person','Lou')])]
fluents = [('robotAt',[('Robot','Frank')],"DockingStation"),
           ('objectAt',[('Object','Beer')],'Kitchen'),
           ('personAt',[('Person', 'Lou')],'Couch')]

planner_interface.kb_interface.insert_facts(predicates)
planner_interface.kb_interface.insert_fluents(fluents)

#Task Definition---------------------------------------------------------------
task_request = TaskRequest()
task_goals = [('bring_object',[('p','Lou'),('o','Beer'),('r', 'Frank')])]

retval, plan = planner_interface.plan(task_request, 'Frank', task_goals)

#output results
print("\n\nActions:")
for action in plan:
    print("{} {}".format(action.type,action.parameters))

#drop database
if test_kb_name in client.list_database_names():
    client.drop_database(test_kb_name)


#Notes:
#Panda Planner is very sensitive to the format of Problem and Domain File:
#Problem File:
#       goal section (htn) must be before init_state section
#
#Domain File:
#      tasks and methods cannot be mixed! First define all tasks then all methods!
#
# If there is a syntax error in one of the files, the returned error message only makes few sense!
#
# Panda Option:  -continueOnSolution may result in multiple solutions