#!/usr/bin/env python3

import os
import unittest
import pymongo as pm
import yaml

from ropod.structs.task import TaskRequest
from task_planner.lama_interface import LAMAInterface

def get_planner_config(config_file_path):
    planner_config_params = None
    with open(config_file_path, 'r') as config_file:
        planner_config_params = yaml.load(config_file)
    return planner_config_params

class LamaPlannerTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.test_kb_name = 'test_robot_store'
        host, port = self._get_db_host_and_port()
        self.client = pm.MongoClient(host=host, port=port)

        planner_config_params = get_planner_config('../config/planner_config.yaml')
        domain_file = planner_config_params['domain_file']
        planner_cmd = planner_config_params['planner_cmd']
        plan_file_path = planner_config_params['plan_file_path']
        self.planner_interface = LAMAInterface(self.test_kb_name, domain_file,
                                               planner_cmd, plan_file_path,
                                               debug=True)

    @classmethod
    def tearDownClass(self):
        self._drop_test_db()

    def test_robot_cart_same_location_delivery_same_floor(self):
        state_facts = [('empty_gripper', [('bot', 'frank')])]
        state_fluents = [('robot_at', [('bot', 'frank')], 'PICKUP_LOCATION'),
                         ('load_at', [('load', 'mobidik')], 'PICKUP_LOCATION')]

        floor_facts = [('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR0')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR1')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR2')])]
        floor_fluents = [('robot_floor', [('bot', 'frank')], 'floor0'),
                         ('load_floor', [('load', 'mobidik')], 'floor0'),
                         ('elevator_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('destination_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('location_floor', [('loc', 'PICKUP_LOCATION')], 'floor0'),
                         ('location_floor', [('loc', 'DELIVERY_LOCATION')], 'floor0'),
                         ('location_floor', [('loc', 'ELEVATOR0')], 'floor0'),
                         ('location_floor', [('loc', 'ELEVATOR2')], 'floor2')]

        self.planner_interface.kb_interface.insert_facts(state_facts)
        self.planner_interface.kb_interface.insert_facts(floor_facts)
        self.planner_interface.kb_interface.insert_fluents(state_fluents)
        self.planner_interface.kb_interface.insert_fluents(floor_fluents)

        task_request = TaskRequest()
        task_request.load_id = 'mobidik'
        task_request.delivery_pose.id = 'DELIVERY_LOCATION'

        task_goals = [('load_at', [('load', task_request.load_id),
                                   ('loc', task_request.delivery_pose.id)]),
                      ('empty_gripper', [('bot', 'frank')])]
        plan_found, plan = self.planner_interface.plan(task_request, 'frank', task_goals)

        self.planner_interface.kb_interface.remove_facts(state_facts)
        self.planner_interface.kb_interface.remove_facts(floor_facts)
        self.planner_interface.kb_interface.remove_fluents(state_fluents)
        self.planner_interface.kb_interface.remove_fluents(floor_fluents)

        assert plan_found

        # the expected plan is:
        # 1. DOCK
        # 2. GOTO DELIVERY_LOCATION
        # 3. UNDOCK
        assert len(plan) == 3

        expected_action_sequence = ['DOCK', 'GOTO', 'UNDOCK']
        obtained_action_sequence = [action.type for action in plan]
        assert expected_action_sequence == obtained_action_sequence

    def test_robot_cart_same_location_delivery_diff_floor(self):
        state_facts = [('empty_gripper', [('bot', 'frank')])]
        state_fluents = [('robot_at', [('bot', 'frank')], 'PICKUP_LOCATION'),
                         ('load_at', [('load', 'mobidik')], 'PICKUP_LOCATION')]

        floor_facts = [('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR0')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR1')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR2')])]
        floor_fluents = [('robot_floor', [('bot', 'frank')], 'floor0'),
                         ('load_floor', [('load', 'mobidik')], 'floor0'),
                         ('elevator_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('destination_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('location_floor', [('loc', 'PICKUP_LOCATION')], 'floor0'),
                         ('location_floor', [('loc', 'DELIVERY_LOCATION')], 'floor2'),
                         ('location_floor', [('loc', 'ELEVATOR0')], 'floor0'),
                         ('location_floor', [('loc', 'ELEVATOR2')], 'floor2')]

        self.planner_interface.kb_interface.insert_facts(state_facts)
        self.planner_interface.kb_interface.insert_facts(floor_facts)
        self.planner_interface.kb_interface.insert_fluents(state_fluents)
        self.planner_interface.kb_interface.insert_fluents(floor_fluents)

        task_request = TaskRequest()
        task_request.load_id = 'mobidik'
        task_request.delivery_pose.id = 'DELIVERY_LOCATION'

        task_goals = [('load_at', [('load', task_request.load_id),
                                   ('loc', task_request.delivery_pose.id)]),
                      ('empty_gripper', [('bot', 'frank')])]
        plan_found, plan = self.planner_interface.plan(task_request, 'frank', task_goals)

        self.planner_interface.kb_interface.remove_facts(state_facts)
        self.planner_interface.kb_interface.remove_facts(floor_facts)
        self.planner_interface.kb_interface.remove_fluents(state_fluents)
        self.planner_interface.kb_interface.remove_fluents(floor_fluents)

        assert plan_found

        # the expected plan is:
        # 1. DOCK
        # 2. GOTO ELEVATOR0
        # 3. REQUEST_ELEVATOR
        # 4. WAIT_FOR_ELEVATOR
        # 5. ENTER_ELEVATOR
        # 6. WAIT_FOR_ELEVATOR
        # 7. RIDE_ELEVATOR
        # 8. EXIT_ELEVATOR
        # 9. GOTO DELIVERY_LOCATION
        # 10. UNDOCK
        # though RIDE_ELEVATOR can also be followed by WAIT_FOR_ELEVATOR
        assert len(plan) == 10

        allowed_action_sequence1 = ['DOCK', 'GOTO', 'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'ENTER_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'RIDE_ELEVATOR',
                                    'EXIT_ELEVATOR', 'GOTO', 'UNDOCK']
        allowed_action_sequence2 = ['DOCK', 'GOTO', 'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'ENTER_ELEVATOR', 'RIDE_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'EXIT_ELEVATOR', 'GOTO', 'UNDOCK']
        obtained_action_sequence = [action.type for action in plan]
        assert (allowed_action_sequence1 == obtained_action_sequence) or\
               (allowed_action_sequence2 == obtained_action_sequence)

    def test_robot_cart_same_floor(self):
        state_facts = [('empty_gripper', [('bot', 'frank')])]
        state_fluents = [('robot_at', [('bot', 'frank')], 'CHARGING_STATION'),
                         ('load_at', [('load', 'mobidik')], 'PICKUP_LOCATION')]

        floor_facts = [('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR0')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR1')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR2')])]
        floor_fluents = [('robot_floor', [('bot', 'frank')], 'floor0'),
                         ('load_floor', [('load', 'mobidik')], 'floor0'),
                         ('elevator_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('destination_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('location_floor', [('loc', 'CHARGING_STATION')], 'floor0'),
                         ('location_floor', [('loc', 'PICKUP_LOCATION')], 'floor0'),
                         ('location_floor', [('loc', 'DELIVERY_LOCATION')], 'floor0'),
                         ('location_floor', [('loc', 'ELEVATOR0')], 'floor0'),
                         ('location_floor', [('loc', 'ELEVATOR2')], 'floor0')]

        self.planner_interface.kb_interface.insert_facts(state_facts)
        self.planner_interface.kb_interface.insert_facts(floor_facts)
        self.planner_interface.kb_interface.insert_fluents(state_fluents)
        self.planner_interface.kb_interface.insert_fluents(floor_fluents)

        task_request = TaskRequest()
        task_request.load_id = 'mobidik'
        task_request.delivery_pose.id = 'DELIVERY_LOCATION'

        task_goals = [('load_at', [('load', task_request.load_id),
                                   ('loc', task_request.delivery_pose.id)]),
                      ('empty_gripper', [('bot', 'frank')])]
        plan_found, plan = self.planner_interface.plan(task_request, 'frank', task_goals)

        self.planner_interface.kb_interface.remove_facts(state_facts)
        self.planner_interface.kb_interface.remove_facts(floor_facts)
        self.planner_interface.kb_interface.remove_fluents(state_fluents)
        self.planner_interface.kb_interface.remove_fluents(floor_fluents)

        assert plan_found

        # the expected plan is:
        # 1. GOTO PICKUP_LOCATION
        # 2. DOCK
        # 3. GOTO DELIVERY_LOCATION
        # 4. UNDOCK
        assert len(plan) == 4

        expected_action_sequence = ['GOTO', 'DOCK', 'GOTO', 'UNDOCK']
        obtained_action_sequence = [action.type for action in plan]
        assert expected_action_sequence == obtained_action_sequence

    def test_delivery_location_diff_floor(self):
        state_facts = [('empty_gripper', [('bot', 'frank')])]
        state_fluents = [('robot_at', [('bot', 'frank')], 'CHARGING_STATION'),
                         ('load_at', [('load', 'mobidik')], 'PICKUP_LOCATION')]

        floor_facts = [('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR0')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR1')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR2')])]
        floor_fluents = [('robot_floor', [('bot', 'frank')], 'floor0'),
                         ('load_floor', [('load', 'mobidik')], 'floor0'),
                         ('elevator_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('destination_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('location_floor', [('loc', 'CHARGING_STATION')], 'floor0'),
                         ('location_floor', [('loc', 'PICKUP_LOCATION')], 'floor0'),
                         ('location_floor', [('loc', 'DELIVERY_LOCATION')], 'floor2'),
                         ('location_floor', [('loc', 'ELEVATOR0')], 'floor0'),
                         ('location_floor', [('loc', 'ELEVATOR2')], 'floor2')]

        self.planner_interface.kb_interface.insert_facts(state_facts)
        self.planner_interface.kb_interface.insert_facts(floor_facts)
        self.planner_interface.kb_interface.insert_fluents(state_fluents)
        self.planner_interface.kb_interface.insert_fluents(floor_fluents)

        task_request = TaskRequest()
        task_request.load_id = 'mobidik'
        task_request.delivery_pose.id = 'DELIVERY_LOCATION'

        task_goals = [('load_at', [('load', task_request.load_id),
                                   ('loc', task_request.delivery_pose.id)]),
                      ('empty_gripper', [('bot', 'frank')])]
        plan_found, plan = self.planner_interface.plan(task_request, 'frank', task_goals)

        self.planner_interface.kb_interface.remove_facts(state_facts)
        self.planner_interface.kb_interface.remove_facts(floor_facts)
        self.planner_interface.kb_interface.remove_fluents(state_fluents)
        self.planner_interface.kb_interface.remove_fluents(floor_fluents)

        assert plan_found

        # the expected plan is:
        # 1. GOTO PICKUP_LOCATION
        # 2. DOCK
        # 3. GOTO ELEVATOR0
        # 4. REQUEST_ELEVATOR floor0 floor2
        # 5. WAIT_FOR_ELEVATOR
        # 6. ENTER_ELEVATOR
        # 7. WAIT_FOR_ELEVATOR
        # 8. RIDE_ELEVATOR
        # 9. EXIT_ELEVATOR
        # 10. GOTO DELIVERY_LOCATION
        # 11. UNDOCK
        # though RIDE_ELEVATOR can also be followed by WAIT_FOR_ELEVATOR
        assert len(plan) == 11

        allowed_action_sequence1 = ['GOTO', 'DOCK', 'GOTO', 'REQUEST_ELEVATOR',
                                    'WAIT_FOR_ELEVATOR', 'ENTER_ELEVATOR',
                                    'WAIT_FOR_ELEVATOR', 'RIDE_ELEVATOR',
                                    'EXIT_ELEVATOR', 'GOTO', 'UNDOCK']
        allowed_action_sequence2 = ['GOTO', 'DOCK', 'GOTO', 'REQUEST_ELEVATOR',
                                    'WAIT_FOR_ELEVATOR', 'ENTER_ELEVATOR',
                                    'RIDE_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'EXIT_ELEVATOR', 'GOTO', 'UNDOCK']
        obtained_action_sequence = [action.type for action in plan]
        assert (allowed_action_sequence1 == obtained_action_sequence) or\
               (allowed_action_sequence2 == obtained_action_sequence)

    def test_robot_cart_diff_floors(self):
        state_facts = [('empty_gripper', [('bot', 'frank')])]
        state_fluents = [('robot_at', [('bot', 'frank')], 'CHARGING_STATION'),
                         ('load_at', [('load', 'mobidik')], 'PICKUP_LOCATION')]

        floor_facts = [('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR0')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR1')]),
                       ('elevator_at', [('elevator', 'toma_elevator'), ('loc', 'ELEVATOR2')])]
        floor_fluents = [('robot_floor', [('bot', 'frank')], 'floor0'),
                         ('load_floor', [('load', 'mobidik')], 'floor2'),
                         ('elevator_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('destination_floor', [('elevator', 'toma_elevator')], 'unknown'),
                         ('location_floor', [('loc', 'CHARGING_STATION')], 'floor0'),
                         ('location_floor', [('loc', 'PICKUP_LOCATION')], 'floor2'),
                         ('location_floor', [('loc', 'DELIVERY_LOCATION')], 'floor1'),
                         ('location_floor', [('loc', 'ELEVATOR0')], 'floor0'),
                         ('location_floor', [('loc', 'ELEVATOR1')], 'floor1'),
                         ('location_floor', [('loc', 'ELEVATOR2')], 'floor2')]

        self.planner_interface.kb_interface.insert_facts(state_facts)
        self.planner_interface.kb_interface.insert_facts(floor_facts)
        self.planner_interface.kb_interface.insert_fluents(state_fluents)
        self.planner_interface.kb_interface.insert_fluents(floor_fluents)

        task_request = TaskRequest()
        task_request.load_id = 'mobidik'
        task_request.delivery_pose.id = 'DELIVERY_LOCATION'

        task_goals = [('load_at', [('load', task_request.load_id),
                                   ('loc', task_request.delivery_pose.id)]),
                      ('empty_gripper', [('bot', 'frank')])]
        plan_found, plan = self.planner_interface.plan(task_request, 'frank', task_goals)

        self.planner_interface.kb_interface.remove_facts(state_facts)
        self.planner_interface.kb_interface.remove_facts(floor_facts)
        self.planner_interface.kb_interface.remove_fluents(state_fluents)
        self.planner_interface.kb_interface.remove_fluents(floor_fluents)

        assert plan_found

        # the expected plan is:
        # 1. GOTO ELEVATOR0
        # 2. REQUEST_ELEVATOR floor0 floor2
        # 3. WAIT_FOR_ELEVATOR
        # 4. ENTER_ELEVATOR
        # 5. WAIT_FOR_ELEVATOR
        # 6. RIDE_ELEVATOR
        # 7. EXIT_ELEVATOR
        # 8. GOTO PICKUP_LOCATION
        # 9. DOCK
        # 10. GOTO ELEVATOR2
        # 11. REQUEST_ELEVATOR floor2 floor1
        # 12. WAIT_FOR_ELEVATOR
        # 13. ENTER_ELEVATOR
        # 14. WAIT_FOR_ELEVATOR
        # 15. RIDE_ELEVATOR
        # 16. EXIT_ELEVATOR
        # 17. GOTO DELIVERY_LOCATION
        # 18. UNDOCK
        # though RIDE_ELEVATOR can also be followed by WAIT_FOR_ELEVATOR
        assert len(plan) == 18

        allowed_action_sequence1 = ['GOTO', 'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'ENTER_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'RIDE_ELEVATOR',
                                    'EXIT_ELEVATOR', 'GOTO', 'DOCK', 'GOTO',
                                    'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'ENTER_ELEVATOR',
                                    'WAIT_FOR_ELEVATOR', 'RIDE_ELEVATOR', 'EXIT_ELEVATOR',
                                    'GOTO', 'UNDOCK']
        allowed_action_sequence2 = ['GOTO', 'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'ENTER_ELEVATOR', 'RIDE_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'EXIT_ELEVATOR', 'GOTO', 'DOCK', 'GOTO',
                                    'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'ENTER_ELEVATOR',
                                    'WAIT_FOR_ELEVATOR', 'RIDE_ELEVATOR', 'EXIT_ELEVATOR',
                                    'GOTO', 'UNDOCK']
        allowed_action_sequence3 = ['GOTO', 'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'ENTER_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'RIDE_ELEVATOR',
                                    'EXIT_ELEVATOR', 'GOTO', 'DOCK', 'GOTO',
                                    'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'ENTER_ELEVATOR',
                                    'RIDE_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'EXIT_ELEVATOR',
                                    'GOTO', 'UNDOCK']
        allowed_action_sequence4 = ['GOTO', 'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'ENTER_ELEVATOR', 'RIDE_ELEVATOR', 'WAIT_FOR_ELEVATOR',
                                    'EXIT_ELEVATOR', 'GOTO', 'DOCK', 'GOTO',
                                    'REQUEST_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'ENTER_ELEVATOR',
                                    'RIDE_ELEVATOR', 'WAIT_FOR_ELEVATOR', 'EXIT_ELEVATOR',
                                    'GOTO', 'UNDOCK']
        obtained_action_sequence = [action.type for action in plan]
        assert (allowed_action_sequence1 == obtained_action_sequence) or\
               (allowed_action_sequence2 == obtained_action_sequence) or\
               (allowed_action_sequence3 == obtained_action_sequence) or\
               (allowed_action_sequence4 == obtained_action_sequence)

    @classmethod
    def _drop_test_db(self):
        if self.test_kb_name in self.client.list_database_names():
            self.client.drop_database(self.test_kb_name)

    @classmethod
    def _get_db_host_and_port(self):
        host = 'localhost'
        port = 27017
        if 'DB_HOST' in os.environ:
            host = os.environ['DB_HOST']
        if 'DB_PORT' in os.environ:
            port = int(os.environ['DB_PORT'])
        return (host, port)

if __name__ == '__main__':
    unittest.main()
