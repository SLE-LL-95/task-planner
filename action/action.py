class Action(object):
    def __init__(self):
        self.id = ''
        self.type = ''
        self.parameter_order = []
        self.parameters = dict()

    def assign_parameters(self, params):
      if(len(self.parameter_order) == len(params)):
          for param_name, param_value in zip(self.parameter_order, params):
              self.parameters[param_name] = param_value

    @staticmethod
    def to_dict(self):
      pass

    @staticmethod
    def from_dict(action_dict):
       pass