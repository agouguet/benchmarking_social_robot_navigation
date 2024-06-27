import math
import random
from mbsn.solver.multi_armed_bandit.multi_armed_bandit import MultiArmedBandit


class UpperConfidenceBounds(MultiArmedBandit):
    def __init__(self):
        self.total = 0
        # number of times each action has been chosen
        self.times_selected = {}

    def select(self, state, actions, qfunction, c_param=1.41):

        # First execute each action one time
        for action in actions:
            if action not in self.times_selected.keys():
                self.times_selected[action] = 1
                self.total += 1
                return action

        max_actions = []
        max_value = float("-inf")
        for action in actions:
            value = qfunction.get_q_value(state, action) + c_param * math.sqrt(
                (2 * math.log(self.total)) / self.times_selected[action]
            )
            # print("         ", action, value, qfunction.get_q_value(state, action))
            if value > max_value:
                max_actions = [action]
                max_value = value
            elif value == max_value:
                max_actions += [action]

        # if there are multiple actions with the highest value
        # choose one randomly
        result = random.choice(max_actions)
        self.times_selected[result] = self.times_selected[result] + 1
        self.total += 1
        # print("             ", result, self.times_selected[result])
        return result
