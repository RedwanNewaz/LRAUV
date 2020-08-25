from __future__ import division
from mcts import mcts
from EnvMCTS import Simulator
from copy import deepcopy
from queue import Queue
from time import time
def timeWrap(func):
    def runtime(*args, **kwargs):
        tic = time()
        result = func(*args, **kwargs)
        toc = time()
        print('Elapsed time {:.3f}s'.format(toc - tic))
        return result
    return runtime

@timeWrap
def main(initialState):
    epoch, step = 0, 0
    initial_pos = deepcopy(initialState.pos)
    SEARCH_HORIZON = 200
    while not initialState.isTerminal():
        if epoch < SEARCH_HORIZON:
            action = mcts.search(initialState=initialState)
            initialState = initialState.takeAction(action)
            print(epoch, action, initialState)
            epoch += 1
        else:
            epoch = 0
            initialState.pos = deepcopy(initial_pos)
            print('restarting search ', '.' * 10)
        step += 1
        break
    print("TOTAL STEP TAKEN ", step)
    print('*' * 100)


def show_tree(mcts):
    def DFS(node, action = None):
        if not node: return
        print(node.state, action)
        for action, child in node.children.items():
            DFS(child, action)

    def BFS(node):
        q = Queue()
        q.put(node)
        while (not q.empty()):
            node = q.get()
            print(node.state)
            for action, child in node.children.items():
                if child:
                    q.put(child)

    return BFS(mcts.root)


if __name__ == '__main__':
    initialState = Simulator()
    mcts = mcts(timeLimit=1000, explorationConstant=0.9)
    main(initialState)
    show_tree(mcts)
