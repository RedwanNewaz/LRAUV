def greedyPolicy(state):
    while not state.isTerminal():
        try:
            best_reward = -10000000
            best_action = None
            for action in state.getPossibleActions():
                newState = state.takeAction(action)
                reward = newState.getReward()
                if reward > best_reward:
                    best_reward = reward
                    best_action = action
        except IndexError:
            raise Exception("Non-terminal state has no possible actions: " + str(state))
        state = state.takeAction(best_action)
    return state.getReward()
