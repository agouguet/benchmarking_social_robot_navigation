
class QFunction:

    """ Update the Q-value of (state, action) by delta """

    def update(self, state, action, delta):
        abstract

    """ Get a Q value for a given state-action pair """

    def get_q_value(self, state, action):
        abstract

    """ Save a policy to a specified filename """
    def save_policy(self, filename):
        abstract

    """ Load a policy from a specified filename """
    def load_policy(self, filename):
        abstract

    """ Return a pair containing the action and Q-value, where the
        action has the maximum Q-value in state
    """

    def get_max_q(self, state, actions):
        arg_max_q = None
        max_q = float("-inf")
        for action in actions:
            value = self.get_q_value(state, action)
            if max_q < value:
                arg_max_q = action
                max_q = value
        return (arg_max_q, max_q)