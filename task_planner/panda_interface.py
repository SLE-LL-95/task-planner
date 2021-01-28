import os
from os import listdir
from os.path import join
from typing import Tuple, Sequence
import uuid
import subprocess
import numpy as np
import logging

#TRY to get independent from ropod library!!
#Define own Action Struct
from ropod.structs.task import TaskRequest
from action.action import Action

from task_planner.planner_interface import TaskPlannerInterface
from task_planner.knowledge_base_interface import Predicate, Task
from task_planner.action_models_hddl import ActionModelLibrary
from task_planner.knowledge_models_hddl import HDDLPredicateLibrary, HDDLFluentLibrary, HDDLNumericFluentLibrary


class PANDAInterface(TaskPlannerInterface):
    _plan_file_name = 'plan.txt'

    def __init__(self, kb_database_name, domain_file,
                 planner_cmd, plan_file_path, debug=False):
        super(PANDAInterface, self).__init__(kb_database_name, domain_file,
                                            planner_cmd, plan_file_path,
                                            debug)
        self.logger = logging.getLogger('task.planner')

    def plan(self, task_request: TaskRequest, robot: str, task_goals: list=None):
        '''
        task_goals must be a list of htn tasks
        '''
        # TODO: check if there are already goals in the knowledge base and,
        # if yes, add them to the task_goals list

        htn_task_goals = []
        for task_goal in task_goals:
            if isinstance(task_goal, Task):
                htn_task_goals.append(task_goal)
            elif isinstance(task_goal, tuple):
                htn_task_goals.append(Task.from_tuple(task_goal))
            elif isinstance(task_goal, dict):
                htn_task_goals.append(Task.from_dict(task_goal))
            else:
                raise Exception('Invalid type to task_goal encountered')

        kb_predicate_assertions = self.kb_interface.get_predicate_assertions()
        kb_fluent_assertions = self.kb_interface.get_fluent_assertions()

        print("Generating Problem File...")
        self.logger.info('Generating problem file')
        problem_file = self.generate_problem_file(kb_predicate_assertions,
                                                  kb_fluent_assertions,
                                                  htn_task_goals)


        self.planner_cmd = self.planner_cmd.replace('PROBLEM', problem_file)
        self.planner_cmd = self.planner_cmd.replace('DOMAIN', self.domain_file)

        planner_cmd_elements = self.planner_cmd.split()

        #Call Planner
        print("Planning Task...")
        self.logger.info('Planning task...')
        print(self.planner_cmd)
        #planner_output=subprocess.run(planner_cmd_elements, capture_output=True)
        planner_output=subprocess.run(planner_cmd_elements,stdout=subprocess.PIPE)

        #catch error
        if(planner_output.stderr):
            self.logger.info('Planner returned Error')
            print("Planner returned an error:")
            print(str(planner_output.stderr,'utf-8'))
            return False, []

        #write console output to file
        text_file = open(self.plan_file_path+self._plan_file_name, "w")
        text_file.write(str(planner_output.stdout,'utf-8'))
        text_file.close()
        self.logger.info('Planning finished')

        print("Parsing Plan...")
        self.logger.info('Parsing plans...')
        plan_found, plan = self.parse_plan(task_request.load_type, robot)

        print("Removing Problem File...")
        self.logger.info('Removing problem file...')
        os.remove(problem_file)
        self.logger.info('Planner done')
    
        return plan_found, plan

    def generate_problem_file(self, predicate_assertions: list,
                              fluent_assertions: list,
                              task_goals: Sequence[Task]) -> str:
        obj_types = {}
        init_state_str = ''
        
        print("Reading Predicate Assertions...")
        # we generate strings from the predicate assertions of the form
        # (predicate_name param_1 param_2 ... param_n)
        for assertion in predicate_assertions:
            ordered_param_list, obj_types = HDDLPredicateLibrary.get_assertion_param_list(assertion.name,
                                                                                          assertion.params,
                                                                                          obj_types)
            assertion_str = '        ({0} {1})\n'.format(assertion.name, ' '.join(ordered_param_list))
            init_state_str += assertion_str

        print("Reading Fluent Assertions...")
        # for numeric fluents, we generate strings of the form
        # (= (fluent_name param_1 param_2 ... param_n) fluent_value); otherwise,
        # we generate strings just like for predicate assertions
        for assertion in fluent_assertions:
            if hasattr(HDDLPredicateLibrary, assertion.name):
                ordered_param_list, obj_types = HDDLPredicateLibrary.get_assertion_param_list(assertion.name,
                                                                                              assertion.params,
                                                                                              obj_types)
                assertion_str = '        ({0} {1} {2})\n'.format(assertion.name,
                                                                 ' '.join(ordered_param_list),
                                                                 assertion.value)
            elif hasattr(HDDLFluentLibrary, assertion.name):
                ordered_param_list, obj_types = HDDLFluentLibrary.get_assertion_param_list(assertion.name,
                                                                                           assertion.params,
                                                                                           assertion.value,
                                                                                           obj_types)
                assertion_str = '        ({0} {1} {2})\n'.format(assertion.name,
                                                                 ' '.join(ordered_param_list),
                                                                 assertion.value)
            else:
                ordered_param_list, obj_types = HDDLNumericFluentLibrary.get_assertion_param_list(assertion.name,
                                                                                                  assertion.params,
                                                                                                  obj_types)
                assertion_str = '        (= ({0} {1}) {2})\n'.format(assertion.name,
                                                                     ' '.join(ordered_param_list),
                                                                     assertion.value)
            init_state_str += assertion_str

        # we combine the assertion strings into an initial state string of the form
        # (:init
        #     assertions
        # )
        init_state_str = '    (:init\n{0}\n    )\n\n'.format(init_state_str)

        print("Generating Objects List...")
        # we generate a string with the object types of the form
        # (:objects
        #     obj_11 obj_12 - type_1
        #     ...
        #     obj_n1 - type_n
        # )
        obj_type_str = ''
        for obj_type in obj_types:
            obj_type_str += '        {0} - {1}\n'.format(' '.join(obj_types[obj_type]), obj_type)
        obj_type_str = '    (:objects\n{0}    )\n\n'.format(obj_type_str)
        
        print("Generating Goal String...")
        # we generate a string with the planning goals of the form
        # (:htn
        #       :parameters()
        #       :subtasks (and
        #           (task1 param_1, ... , param_n)
        #           (...)     
        #           (taskn param_1, ... , param_n)    
        #       )
        #       :ordering()
        # )
        #TODO: Allow Task ordering

        goal_str = ''
        task_lines = ''
        for task_no,task_goal in enumerate(task_goals):
            goal_task, goal_params = task_goal.name, task_goal.params
            task_lines += '            (task{0} ({1} {2}))\n'.format(task_no,goal_task,
                                                                     ' '.join([param.value for param in goal_params]))
        goal_str += '    (:htn\n'
        goal_str += '        :parameters ()\n'
        goal_str += '        :subtasks (and\n'
        goal_str += '{0}'.format(task_lines)
        goal_str += '         )\n'
        goal_str += '         :ordering ()\n'
        goal_str += '    )\n\n'

        print("Writing Problem File...")
        # we finally write the problem file, which will be in the format
        # (define (problem problem-name)
        #     (:domain domain-name)
        #     (:objects
        #         ...
        #     )
        #     (:htn
        #         ...
        #     )
        #     (:init
        #         ...
        #     )
        # )
        problem_file_name = 'problem_{0}.hddl'.format(str(uuid.uuid4()))
        problem_file_abs_path = join(self.plan_file_path, problem_file_name)
        self.logger.info('Generating planning problem...')
        with open(problem_file_abs_path, 'w') as problem_file:
            header = '(define (problem domestic)\n'
            header += '    (:domain {0})\n'.format(self.domain_name)
            problem_file.write(header)
            problem_file.write(obj_type_str)
            problem_file.write(goal_str)
            problem_file.write(init_state_str)
            problem_file.write(')\n')
        return problem_file_abs_path

    def parse_plan(self, task: str, robot: str) -> Tuple[bool, list]:

        #find all files in directory containing the std file name
        #dont know why there could be multiple files, just copied from LamaInterface
        plan_files = [f for f in listdir(self.plan_file_path)
                      if f.find(self._plan_file_name) != -1]
        if not plan_files:
            self.logger.error('Plan for task %s and robot %s not found', task, robot)
            return False, []

        plans = []
        action_strings_per_plan = []
        for plan_file_name in plan_files:
            plan = []
            plan_action_strings = []
            current_plan_file_path = join(self.plan_file_path, plan_file_name)

            with open(current_plan_file_path, 'r') as plan_file:
                
                #find start of solution
                for i,line in enumerate(plan_file):
                    if line == "SOLUTION SEQUENCE\n":
                        break
                
                plan_sequence = [line for line in plan_file] #this works because a text file has an internal pointer

                #A Line normally looks like this: "0: drive(truck_0,city_loc_2,city_loc_1)"
                last_action_number = -1
                for line in plan_sequence:
                    action_number, action_line = line.split(": ")
                    action_number = int(action_number)

                    #check if action numbers are increasing 1 by 1
                    if action_number == (last_action_number + 1):
                        last_action_number = action_number
                        self.logger.debug(action_line)
                        plan_action_strings.append(action_line)
                        new_action = self.process_action_str(action_line)

                        if new_action != None:
                            plan.append(new_action)
                    else:
                        print("Something is not as usual!")

                plans.append(plan)
                action_strings_per_plan.append(plan_action_strings)

            #os.remove(current_plan_file_path)

        plan_lengths = [len(plan) for plan in plans]
        shortest_plan_idx = np.argmin(plan_lengths)

        self.logger.info('Plan for task %s and robot %s found', task, robot)
        self.logger.debug('Action sequence:')
        self.logger.debug('-------------------------------')
        for action_string in action_strings_per_plan[shortest_plan_idx]:
            self.logger.debug(action_string)
        self.logger.debug('-------------------------------')
        return True, plans[shortest_plan_idx]

    def process_action_str(self, action_line: str) -> Action:
        '''Process single line of planner output
        Example: drive(truck_0,city_loc_2,city_loc_1)
        '''

        #split name/parameters
        action_name, action_parameter_string = action_line.split("(")

        #if a method has a precondition, it appears as SHOP_method in the plan
        #we can just ignore this line
        if (action_name.startswith('SHOP_method')):
            return None
        #remove closing brace and newline character
        action_parameter_string = action_parameter_string.replace(")","")
        action_parameter_string = action_parameter_string.replace("\n","")

        #split parameters
        action_params = action_parameter_string.split(",")

        #Create Action from library
        action = ActionModelLibrary.get_action_model(action_name.upper(), action_params)
        action.assign_parameters(action_params)

        return action