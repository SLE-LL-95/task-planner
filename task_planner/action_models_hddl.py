import uuid
from action.action import Action


class ActionModelLibrary(object):
    @staticmethod
    def get_action_model(action_name: str, action_params: list) -> Action:
        action = Action()
        action.id = str(uuid.uuid4())
        action.type = action_name
        try:
            action = getattr(ActionModelLibrary, action_name)(action, action_params)
        except:
            print(f"Action {action_name} not defined!")
            action = []
        return action
    
    @staticmethod
    def MOVEBASE(action: Action, params: list) -> Action:
        action.parameter_order = ['Robot','Location0','Location1']
        return action

    @staticmethod
    def OPEN(action: Action, params: list) -> Action:
        action.parameter_order = ['Door','Robot','Waypoint']
        return action
    
    @staticmethod
    def N_OPEN(action: Action, params: list) -> Action:
        action.parameter_order = ['Door','Robot','Waypoint']
        return action
    
    @staticmethod
    def PERCEIVEPLANE(action: Action, params: list) -> Action:
        action.parameter_order = ['Plane','Robot','Waypoint']
        return action

    #TODO:  Replace Pick/Place from/on Plane/Furniture by one action with 'Context' parameter
    #       Depending on 'Context', assign correct parameter order
    #       eg: 
    #       if(params[4] == 'pick_from_furniture'): 
    #           action.parameter_order = xxx
    #       elif (params[4] == 'pick_from_plane'):
    #          action.parameter_order = yyy

    @staticmethod
    def PICKFROMFURNITURE(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Furniture','Robot','Waypoint']
        return action
    
    @staticmethod
    def PICKFROMPLANE(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Plane','Robot','Waypoint']
        return action
    
    @staticmethod
    def PLACEONPLANE(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Plane','Robot','Waypoint']
        return action
    
    @staticmethod
    def PLACEONFURNITURE(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Furniture','Robot','Waypoint']
        return action
    #---------------------------------------------------------------------------------------

    @staticmethod
    def THROW(action: Action, params: list) -> Action:
        action.parameter_order = ['Object0','Object1','Robot','Waypoint']
        return action
    
    @staticmethod
    def HANDOVER(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Robot','Person','Waypoint']
        return action
