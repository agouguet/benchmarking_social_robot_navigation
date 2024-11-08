import itertools
import math
import time
import random
from collections import defaultdict, deque
from mbsn.solver.heuristicfunction import *

class MBSNAgentNode:
    # Record a unique node id to distinguish duplicated states
    next_node_id = 0

    # Records the number of times states have been visited
    visits = defaultdict(lambda: 0)

    def __init__(self, mdp, parent, state, qfunction, bandit, reward=0.0, action=None):
        self.mdp = mdp
        self.parent = parent
        self.state = state
        self.id = MBSNAgentNode.next_node_id
        MBSNAgentNode.next_node_id += 1

        # The Q function used to store state-action values
        self.qfunction = qfunction

        # A multi-armed bandit for this node
        self.bandit = bandit

        # The immediate reward received for reaching this state, used for backpropagation
        self.reward = reward

        # The action that generated this node
        self.action = action

        # A dictionary from actions to a set of node-probability pairs
        self.children = defaultdict(list)

    """ Return true if and only if all child actions have been expanded """
    def is_fully_expanded(self):
        valid_actions = self.mdp.get_actions(self.state)
        if len(valid_actions) == len(self.children):
            return True
        else:
            return False

    """ Select a node that is not fully expanded """
    def select(self):
        if not self.is_fully_expanded() or self.mdp.is_terminal(self.state):
            return self
        else:
            actions = list(self.children.keys())
            action = self.bandit.select(self.state, actions, self.qfunction)
            # print("     ", self.state, "  ", actions, "  ", action)
            return self.get_outcome_child(action).select()


    """ Expand a node if it is not a terminal node """
    def expand(self, heuristic_function=None):
        if not self.mdp.is_terminal(self.state):
            actions = self.mdp.get_actions(self.state) - self.children.keys()
            # action = heuristic_function(self.mdp, self.state, prev_actions=[self.action], actions = actions)             
            action = random.choice(list(actions))

            return self.get_outcome_child(action)
        return self

    """ Backpropogate the reward back to the parent node """
    def back_propagate(self, reward, child):
        action = child.action

        MBSNAgentNode.visits[self.state] = MBSNAgentNode.visits[self.state] + 1
        MBSNAgentNode.visits[(self.state, action)] = MBSNAgentNode.visits[(self.state, action)] + 1

        delta = (1 / (MBSNAgentNode.visits[(self.state, action)])) * (
            reward - self.qfunction.get_q_value(self.state, action)
        )
        
        self.qfunction.update(self.state, action, delta)

        if self.parent != None:
            self.parent.back_propagate(self.reward + reward, self)

    """ Simulate the outcome of an action, and return the child node """
    def get_outcome_child(self, action):
        # Choose one outcome based on transition probabilities
        (next_state, reward) = self.mdp.execute(self.state, action)

        # Find the corresponding state and return if this already exists
        for (child, _) in self.children[action]:
            if next_state == child.state:
                # print("state already exists:", self.state, "CHILD:", child.state)
                return child

        # This outcome has not occured from this state-action pair previously
        new_child = MBSNAgentNode(
            self.mdp, self, next_state, self.qfunction, self.bandit, reward, action
        )

        # Find the probability of this outcome (only possible for model-based) for visualising tree
        probability = 0.0
        for (outcome, probability) in self.mdp.get_transitions(self.state, action):
            if outcome == next_state:
                self.children[action].append((new_child, probability))
                # print("NEW CHILD : ", new_child.state)
                return new_child
        

    """ Return the value of this node """
    def get_value(self):
        (arg_max_q, max_q_value) = self.qfunction.get_max_q(
            self.state, self.mdp.get_actions(self.state)
        )
        return (arg_max_q, max_q_value)

    """ Get the number of visits to this state """
    def get_visits(self):
        return MBSNAgentNode.visits[self.state]

    def reset_visits():
        MBSNAgentNode.next_node_id = 0
        MBSNAgentNode.visits = defaultdict(lambda: 0)


class MBSNAgentMCTS:
    def __init__(self, mdp, qfunction, bandit, heuristic_function=DEFAULT_HEURISTIC_FUNCTION):
        self.mdp = mdp
        self.qfunction = qfunction
        self.bandit = bandit
        self._heuristic_function = heuristic_function

    """
    Execute the MCTS algorithm from the initial state given, with timeout in seconds
    """
    def mcts(self, state, timeout=1, sleep=True):
        self.first=True
        root_node = MBSNAgentNode(self.mdp, None, state, self.qfunction, self.bandit)

        start_time = time.time()
        current_time = time.time()
        num_rollouts = 0
        while current_time < start_time + timeout:
            # print('-----')
            # print("{:.2f}".format(current_time-start_time))
            # Find a state node to expand
            selected_node = root_node.select()
            # if self.first:
            
            # print("{:.2f}".format(current_time-start_time))
            if not self.mdp.is_terminal(selected_node.state):
                child = selected_node.expand(self._heuristic_function)
                # print("{:.2f}".format(current_time-start_time))
                reward = self.simulate(selected_node, child)
                # print("{:.2f}".format(current_time-start_time))
                selected_node.back_propagate(reward, child)
                # print("{:.2f}".format(current_time-start_time))
                num_rollouts += 1

            current_time = time.time()
            if sleep:
                time.sleep(0.000001)

            # print("\n\nSELECTION: ", selected_node.state, MBSNAgentNode.visits[selected_node.state], self.mdp.get_actions(selected_node.state), len(selected_node.children)) #[sp[0][0].state for id, sp in selected_node.children.items()]

        return root_node, num_rollouts

    """ Choose a random action. Heustics can be used here to improve simulations. """
    def choose(self, state, prev_actions):
        # print(" -- MCTS choose")
        action_choosen = self._heuristic_function(self.mdp, state, debug=False)
        # action_choosen = self._heuristic_function(self.mdp, state, prev_actions=prev_actions) #random.choice(self.mdp.get_actions(state))
        # if self.first:
        #     print(state, action_choosen)
        #     self.first=False
        # print("List prev actions:", prev_actions)
        # print("MCTS:", state, self.mdp.get_state_from_continuous_position(self.mdp.goal), prev_actions, action_choosen)

        # action_choosen = random.choice(self.mdp.get_actions(state))
        return action_choosen

    """ Simulate until a terminal state """
    def simulate(self, parent_node, child_node):
        parent_state = parent_node.state
        state = child_node.state
        # if self.first:
        # cumulative_reward = 0.0
        cumulative_reward = self.mdp.get_reward(parent_state, child_node.action, state)

        list_actions=deque(maxlen=3)
        list_actions.append(parent_node.action)
        list_actions.append(child_node.action)

        depth = 0
        while not self.mdp.is_terminal(state):# and depth < 20:
            
            # Choose an action to execute
            action_deque_slice = deque(itertools.islice(list_actions, 0, len(list_actions)-1))
            t = time.time()
            action = self.choose(state, action_deque_slice)
            # print("TTTTTT ", time.time() - t)
            list_actions.append(action)
            # print("A = ", action)

            # if depth == 0:
            

            # Execute the action
            (next_state, reward) = self.mdp.execute(state, action)

            # Discount the reward
            cumulative_reward += pow(self.mdp.get_discount_factor(), depth) * reward

            # print("         - ", depth, state, action, next_state, reward, cumulative_reward)
            depth += 1

            state = next_state

        # print("     REWARD:", cumulative_reward)
        return cumulative_reward