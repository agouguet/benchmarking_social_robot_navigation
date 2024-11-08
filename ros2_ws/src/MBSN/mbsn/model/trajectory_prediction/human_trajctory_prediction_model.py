import copy
import itertools
import math, time

import numpy as np
from shapely import Point
import torch

from mbsn.model.polygon.State import State
from mbsn.model.polygon.human import Human

from ament_index_python.packages import get_package_share_directory


human_movements = {
    0: 0.8,
    45: 0.05,
    90: 0.035,
    -45: 0.05,
    -90: 0.035,
    180: 0.03,
}

human_movements_02 = {
    0: 0.6,
    45: 0.1,
    90: 0.075,
    -45: 0.1,
    -90: 0.075,
    180: 0.05,
}

human_movements_03 = {
    0: 1/6,
    60: 1/6,
    120: 1/6,
    180: 1/6,
    240: 1/6,
    300: 1/6,
}

probability_function_to_use = human_movements

def simple_trajectory_prediction_for_one_human(human):
    future_pos = []
    for angle, probability in probability_function_to_use.items():
            future_pos.append(Point(future_position(human.position.x, human.position.y, human.orientation + angle)))
    return future_pos

def simple_human_trajectory_prediction(mdp, state, action):

    if len(state.humans) == 0:
        return [(State(action, state.humans), 1.0)]
    
    humans = sorted(state.humans, key=lambda h: mdp.polygons[state.robot][0].centroid.distance(h.position))
    humans = humans[:5]
    # print(humans)


    t = time.time()
    pair_future_state_probabilities = []

    list_of_futures_humans = []
    list_of_probabilities = []

    for human in humans:
        human_states = []
        probabilities = []
        for angle, probability in probability_function_to_use.items():
            future_pos = future_position(human.position.x, human.position.y, human.orientation + angle)
            human_states.append(Human(future_pos, human.orientation + angle, id=human.id))
            probabilities.append(probability)
        list_of_futures_humans.append(human_states)
        list_of_probabilities.append(probabilities)
    
    
    all_possibility = list(itertools.product(*list_of_futures_humans))
    all_probabilities = [np.prod(prob) for prob in list(itertools.product(*list_of_probabilities))] 

    all = [(State(action, all_possibility[i]), all_probabilities[i]) for i in range(len(all_possibility))]
    # print(np.array(all).shape, len(humans))
    return all


    # humans = state.humans.copy()
    for human in state.humans:
        for angle, probability in probability_function_to_use.items():
            new_humans = []
            future_pos = future_position(human.position.x, human.position.y, human.orientation + angle)

            for human2 in state.humans:
                if human2.id == human.id:
                    new_humans.append(Human(future_pos, human.orientation + angle, id=human.id))
                else:
                    new_humans.append(copy.deepcopy(human2))

            pair_future_state_probabilities.append((State(action, new_humans), probability))

    # print("S ", state, action, "    ", pair_future_state_probabilities)

    #all_possibility = list(itertools.product(*list_of_lists))

    return pair_future_state_probabilities

def future_position(x0, y0, theta, v=1, t=1):
    # Si theta est en degrés, le convertir en radians
    theta_rad = math.radians(theta)
    
    # Calculer les deltas
    delta_x = v * t * math.cos(theta_rad)
    delta_y = v * t * math.sin(theta_rad)
    
    # Calculer la nouvelle position
    x1 = x0 + delta_x
    y1 = y0 + delta_y
    
    return x1, y1


def pecnet_human_trajectory_prediction(mdp, state, action):
    if len(state.humans) == 0:
        return [(State(action, state.humans), 1.0)]
    
    pair_future_state_probabilities = []

    humans = state.humans
    new_humans = []
    for h in humans:
        if len(h.future_predicted_position) > 0:
            new_humans.append(Human(h.future_predicted_position[0], h.orientation, h.future_predicted_position[1:], id=h.id))
        else:
            new_humans.append(Human(h.position, h.orientation, id=h.id))

    pair_future_state_probabilities.append((State(action, new_humans), 1.0))

    return pair_future_state_probabilities