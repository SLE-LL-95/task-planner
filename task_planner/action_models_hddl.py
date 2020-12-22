import uuid
from ropod.structs.action import Action
from ropod.structs.area import Area


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
    def MOVEBASE(action: Action, params: List) -> Action:
        return action

    @staticmethod
    def OPEN(action: Action, params: List) -> Action:
        return action
    
    @staticmethod
    def PERCEIVEPLANE(action: Action, params: List) -> Action:
        return action

    @staticmethod
    def PICK(action: Action, params: List) -> Action:
        return action
    
    @staticmethod
    def PLACE(action: Action, params: List) -> Action:
        return action

    @staticmethod
    def THROW(action: Action, params: List) -> Action:
        return action
    
    @staticmethod
    def HANDOVER(action: Action, params: List) -> Action:
        return action
