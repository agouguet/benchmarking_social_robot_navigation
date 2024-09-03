import copy
import math
from mbsn.model.polygon.State import State
from mbsn.model.polygon.human import Human
from mbsn.model.trajectory_prediction.trajectory_prediction_model import TrajectoryPredictionModel

human_movements = {
                    0: 0.75,
                    45: 0.05,
                    90: 0.05,
                    -45: 0.05,
                    -90: 0.05,
                    180: 0.05,
                }   

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

class SimpleTrajectoryPrediction(TrajectoryPredictionModel):

    def predict(self, state, action):
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

        return pair_future_state_probabilities