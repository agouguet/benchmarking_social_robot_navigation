import copy
import math

import numpy as np
import torch

from mbsn.model.polygon.State import State
from mbsn.model.polygon.human import Human

from mbsn.model.trajectory_prediction.utils.models import PECNet
from ament_index_python.packages import get_package_share_directory


human_movements = {
    0: 0.75,
    45: 0.05,
    90: 0.05,
    -45: 0.05,
    -90: 0.05,
    180: 0.05,
}


def simple_human_trajectory_prediction(state, action):

    if len(state.humans) == 0:
        return [(State(action, state.humans), 1.0)]
    
    pair_future_state_probabilities = []

    # humans = state.humans.copy()
    for human in state.humans:
        for angle, probability in human_movements.items():
            new_humans = []
            future_pos = future_position(human.position.x, human.position.y, human.orientation + angle)

            for human2 in state.humans:
                if human2.id == human.id:
                    new_humans.append(Human(future_pos, human.orientation + angle, id=human.id))
                else:
                    new_humans.append(copy.deepcopy(human2))

            pair_future_state_probabilities.append((State(action, new_humans), probability))

    # print("S ", state, action, "    ", pair_future_state_probabilities)
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


def pecnet_human_trajectory_prediction(state, action):
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