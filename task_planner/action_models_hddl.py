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
            #call assign_parameters here??
        except:
            print("Action {} not defined!".format(action_name))
            action = []
        return action
    
    # Actions for default domestic HDDL domain
    @staticmethod
    def MOVEBASE(action: Action, params: list) -> Action:
        action.parameter_order = ['Robot','Location0','Location1']
        return action

    @staticmethod
    def OPEN(action: Action, params: list) -> Action:
        action.parameter_order = ['Door','Robot','Waypoint']
        return action
        
    @staticmethod
    def PERCEIVEPLANE(action: Action, params: list) -> Action:
        action.parameter_order = ['Plane','Robot','Waypoint']
        return action

    @staticmethod
    def PICKFROMFURNITURE(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Furniture','Robot','Waypoint']
        action.type = 'PICK'
        action.parameters['Context'] = 'pick_from_container'
        return action
    
    @staticmethod
    def PICKFROMPLANE(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Plane','Robot','Waypoint']
        action.type = 'PICK'
        action.parameters['Context'] = 'pick_from_plane'
        return action
    
    @staticmethod
    def PLACEONPLANE(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Plane','Robot','Waypoint']
        action.type = 'PLACE'
        action.parameters['Context'] = 'place_on_plane'
        return action
    
    @staticmethod
    def PLACEONFURNITURE(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Furniture','Robot','Waypoint']
        action.type = 'PLACE'
        action.parameters['Context'] = 'place_in_container'
        return action

    @staticmethod
    def THROW(action: Action, params: list) -> Action:
        action.parameter_order = ['Object0','Object1','Robot','Waypoint']
        return action
    
    @staticmethod
    def HANDOVER(action: Action, params: list) -> Action:
        action.parameter_order = ['Object','Robot','Person','Waypoint']
        return action

    @staticmethod
    def FINDPEOPLE(action: Action, params: list) -> Action:
        action.parameter_order = ['Person','Robot','Waypoint']
        return action