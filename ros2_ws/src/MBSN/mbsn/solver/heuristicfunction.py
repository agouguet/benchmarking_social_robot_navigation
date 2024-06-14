import random

def random_function(mdp, state):
    return random.choice(mdp.get_actions(state))

def closest_node_to_goal(mdp, state):
    actions = mdp.get_actions(state)
    return min(actions, key=lambda a: (mdp.astar_dict[a._node][state.goal]))