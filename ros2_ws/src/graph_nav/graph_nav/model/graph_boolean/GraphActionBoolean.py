from graph_nav.model.Action import Action

class GraphActionBoolean(Action):
    def __init__(self, node):
        Action.__init__(self, str(node))
        self._node = node