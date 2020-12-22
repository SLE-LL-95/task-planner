from typing import Tuple


class HDDLKnowledgeUtils(object):
    @staticmethod
    def get_ordered_param_list(params: list, param_order: dict, obj_types: dict) -> Tuple[list, dict]:
        param_list = []
        param_count = 0
        updated_obj_types = dict(obj_types)
        while param_count < len(params):
            for param in params:
                param_name = param_order[param_count][0]
                param_type = param_order[param_count][1]
                if param.name == param_name:
                    param_list.append(param.value)
                    param_count += 1
                    if param_type not in updated_obj_types:
                        updated_obj_types[param_type] = []

                    if param.value not in updated_obj_types[param_type]:
                        updated_obj_types[param_type].append(param.value)
                    break
        return param_list, updated_obj_types


class HDDLPredicateLibrary(object):
    @staticmethod
    def get_assertion_param_list(predicate_name: str, predicate_params: list, obj_types: dict) -> Tuple[list, dict]:
        ordered_param_list, updated_obj_types = getattr(PDDLPredicateLibrary, predicate_name)(predicate_params, obj_types)
        return ordered_param_list, updated_obj_types

    @staticmethod
    def robotName(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Robot', 'Robot')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def objectCategory(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object0', 'Object'), 1: ('Object1','Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def doorAt(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Door', 'Door'), 1: ('Waypoint','Waypoint')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def furnitureAt(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Furniture', 'Furniture'), 1: ('Waypoint','Waypoint')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def planeAt(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Plane', 'Plane'), 1: ('Waypoint','Waypoint')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def doorOpen(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Door', 'Door')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def belongsTo(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Plane', 'Plane'), 1: ('Object','Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def unexplored(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Plane', 'Plane')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def explored(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Plane', 'Plane')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def holding(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Interactor', 'Interactor'), 1:('Object', 'Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def emptyGripper(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Robot', 'Robot')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def known(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Person', 'Person')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    #Why? not known != unknown??
    def unknown(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Person', 'Person')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def canPlaceOn(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object', 'Object'), 1:('Plane', 'Plane')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)

    @staticmethod
    def likelyStoringLocation(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object', 'Object'), 1:('Furniture', 'Furniture')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def hasDoor(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Furniture', 'Furniture')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def above(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object0', 'Object'), 1: ('Object1','Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def below(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object0', 'Object'), 1: ('Object1','Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def onTopOf(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object0', 'Object'), 1: ('Object1','Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def inside(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object0', 'Object'), 1: ('Object1','Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
    @staticmethod
    def toTheLeftOf(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object0', 'Object'), 1: ('Object1','Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    
     @staticmethod
    def toTheRightOf(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('Object0', 'Object'), 1: ('Object1','Object')}
        return HDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)
    

class HDDLFluentLibrary(object):
    @staticmethod
    def get_assertion_param_list(fluent_name: str, fluent_params: list,
                                 fluent_value: str, obj_types: dict) -> Tuple[list, dict]:
        ordered_param_list, obj_types = getattr(HDDLFluentLibrary, fluent_name)(fluent_params,
                                                                                obj_types,
                                                                                fluent_value)
        return ordered_param_list, obj_types

    @staticmethod
    def robotAt(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Robot', 'Robot')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'Waypoint' in updated_obj_types:
            if fluent_value not in updated_obj_types['Waypoint']:
                updated_obj_types['Waypoint'].append(fluent_value)
        else:
            updated_obj_types['Waypoint'] = [fluent_value]
        return ordered_param_list, updated_obj_types
    
    @staticmethod
    def objectAt(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'Waypoint' in updated_obj_types:
            if fluent_value not in updated_obj_types['Waypoint']:
                updated_obj_types['Waypoint'].append(fluent_value)
        else:
            updated_obj_types['Waypoint'] = [fluent_value]
        return ordered_param_list, updated_obj_types
    
    @staticmethod
    def personAt(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Person', 'Person')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'Waypoint' in updated_obj_types:
            if fluent_value not in updated_obj_types['Waypoint']:
                updated_obj_types['Waypoint'].append(fluent_value)
        else:
            updated_obj_types['Waypoint'] = [fluent_value]
        return ordered_param_list, updated_obj_types

    @staticmethod
    def on(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'Plane' in updated_obj_types:
            if fluent_value not in updated_obj_types['Plane']:
                updated_obj_types['Plane'].append(fluent_value)
        else:
            updated_obj_types['Plane'] = [fluent_value]
        return ordered_param_list, updated_obj_types
    
    @staticmethod
    #problem!! 'in' is a keyword!
    def in_(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'Object' in updated_obj_types:
            if fluent_value not in updated_obj_types['Object']:
                updated_obj_types['Object'].append(fluent_value)
        else:
            updated_obj_types['Object'] = [fluent_value]
        return ordered_param_list, updated_obj_types

    @staticmethod
    def defaulStoringLocation(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'Furniture' in updated_obj_types:
            if fluent_value not in updated_obj_types['Furniture']:
                updated_obj_types['Furniture'].append(fluent_value)
        else:
            updated_obj_types['Furniture'] = [fluent_value]
        return ordered_param_list, updated_obj_types
    
    @staticmethod
    def locatedAt(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'Location' in updated_obj_types:
            if fluent_value not in updated_obj_types['Location']:
                updated_obj_types['Location'].append(fluent_value)
        else:
            updated_obj_types['Location'] = [fluent_value]
        return ordered_param_list, updated_obj_types

    @staticmethod
    def isAtNamedPose(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Thing', 'Thing')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'NamedPose' in updated_obj_types:
            if fluent_value not in updated_obj_types['NamedPose']:
                updated_obj_types['NamedPose'].append(fluent_value)
        else:
            updated_obj_types['NamedPose'] = [fluent_value]
        return ordered_param_list, updated_obj_types 

    @staticmethod
    def isAtLocation(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('NamedPose', 'NamedPose')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'Location' in updated_obj_types:
            if fluent_value not in updated_obj_types['Location']:
                updated_obj_types['Location'].append(fluent_value)
        else:
            updated_obj_types['Location'] = [fluent_value]
        return ordered_param_list, updated_obj_types    

    @staticmethod
    def PreferredGraspingStrategy(params: list, obj_types: dict, fluent_value: str) -> Tuple[list, dict]:
        param_order = {0: ('Object', 'Object')}
        ordered_param_list, updated_obj_types = HDDLKnowledgeUtils.get_ordered_param_list(params,
                                                                                          param_order,
                                                                                          obj_types)
        if 'GraspingStrategy' in updated_obj_types:
            if fluent_value not in updated_obj_types['GraspingStrategy']:
                updated_obj_types['GraspingStrategy'].append(fluent_value)
        else:
            updated_obj_types['GraspingStrategy'] = [fluent_value]
        return ordered_param_list, updated_obj_types    

class HDDLNumericFluentLibrary(object):
    @staticmethod
    def get_assertion_param_list(fluent_name: str, fluent_params: list, obj_types: dict) -> Tuple[list, dict]:
        ordered_param_list, obj_types = getattr(PDDLNumericFluentLibrary, fluent_name)(fluent_params, obj_types)
        return ordered_param_list, obj_types
    
    @staticmethod
    def example(params: list, obj_types: dict) -> Tuple[list, dict]:
        param_order = {0: ('VarName', 'Type')}
        return PDDLKnowledgeUtils.get_ordered_param_list(params, param_order, obj_types)