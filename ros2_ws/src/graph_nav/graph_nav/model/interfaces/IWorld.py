from abc import ABC, abstractmethod
import numpy as np

class IWorld(ABC):

    def init(self, debug_mode = False):
        if debug_mode:
            print("Start construct states...", end="")
        self.states = self.get_state_model()
        if debug_mode:
            print("Finish. Size of States :", np.array(self.states).shape, "\n")
            print("Start construct actions...", end="")
        self.actions = self.get_action_model()
        if debug_mode:
            print("Finish.")
            print("Start construct states_prim...", end="")
        self.states_prim = self.get_state_prim_model()
        if debug_mode:
            print("Finish.")
            print("Start construct rewards...", end="")
        self.rewards = self.get_reward_function()
        if debug_mode:
            print("Finish.")
            print("Start construct transitions...", end="")
        self.transitions = self.get_transition_model()
        if debug_mode:
            print("Finish.")

    @abstractmethod
    def get_state_model(self):
        """State model"""

    @abstractmethod
    def get_action_model(self):
        """Action model"""

    @abstractmethod
    def get_state_prim_model(self):
        """Action model"""

    @abstractmethod
    def get_reward_function(self):
        """Reward model"""

    @abstractmethod
    def get_transition_model(self):
        """Transition model"""

    def reward(self, state, action, state_prim):
        return self.rewards[state][action][state_prim]

    def transition(self, state, action, state_prim):
        return self.transitions[state][action][state_prim]


    def __str__action__(self):
        str_ = "\n"
        for s in self.actions:
            str_ += str(s) + ":\n"
            for a in self.actions[s]:
                str_ += "   " + str(a) + "\n"
        return str_

    def __str__state_prim__(self):
        str_ = "\n"
        for s in self.states_prim:
            for a in self.states_prim[s]:            
                str_ += "   [" + str(s) + ", " + str(a) + "] -> " + str(self.states_prim[s][a]) + "\n"
        return str_

    def __str__reward__(self):
        str_ = "\n"
        for s in self.rewards:
            str_ += str(s) + ":\n"
            for a in self.rewards[s]:
                str_ += "   " + str(a) + ":\n"
                for s_prim in self.rewards[s][a]:
                    str_ += "       " + str(s_prim) + "--->" + str(self.rewards[s][a][s_prim]) + "r\n"

        return str_

    def __str__transition__(self):
        str_ = "\n"
        for s in self.transitions:
            str_ += str(s) + ":\n"
            for a in self.transitions[s]:
                str_ += "   " + str(a) + ":\n"
                for s_prim in self.transitions[s][a]:
                    str_ += "       " + str(s_prim) + "--->" + str(self.transitions[s][a][s_prim]*100) + "%\n"

        return str_

    def __str__(self):
        str_ = ""
        str_ += "--- States: "      + str(self.states)          + "\n"
        str_ += "--- Actions: "     + self.__str__action__()    + "\n"
        str_ += "--- States Prim:"  + self.__str__state_prim__()+ "\n"
        str_ += "--- Rewards: "     + self.__str__reward__()    + "\n"
        str_ += "--- Transitions: " + self.__str__transition__()+ "\n"

        return str_

    def __repr__(self):
        return self.__str__()   