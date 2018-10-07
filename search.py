# -*- coding:utf-8 -*-
# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state
        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state
        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take
        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def DFS(state, problem, visited, ans_direct):
    """

    :rtype: object
    """
    # print"state:",state

    # print ans_direct
    check = state in visited.keys()
    # print "checking",check

    if problem.isGoalState(state):
        # print"First"
        # print ans_direct
        return True


    elif (check == False):
        # print"Second"
        visited[state] = 1;
        succ = problem.getSuccessors(state)
        for i in (range(0, len(succ))):
            # print "Current Him",succ[i]
            ans_direct.append(succ[i][1])
            ans = DFS(succ[i][0], problem, visited, ans_direct)
            if ans == True:
                return True
            ans_direct.pop()

    elif (check == True):
        # print"Third"
        if not ans_direct:
            ans_direct.pop()
        return False


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    visited = {}
    ans_direct = []
    start_state = problem.getStartState()
    z = DFS(start_state, problem, visited, ans_direct)

    '''for i in (range(0,len(ans_direct))):
        if ans_direct[i]=='South':
            final_ans[i]='s'
         if ans_direct[i]=='North':
            final_ans[i]='n'
        if ans_direct[i]=='West':
            final_ans[i]='w'
        if ans_direct[i]=='East':
            final_ans[i]='e'''

    return ans_direct

    # util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    start_state = problem.getStartState()
    # print start_state
    q = util.Queue()
    ans_direct = []
    visited = {}
    list_parent = {}
    map_succ = {}
    list_state = []
    q.push(start_state)
    goal_state = start_state
    while not (q.isEmpty()):
        current_state = q.pop()
        # print "Current",current_state

        if (problem.isGoalState(current_state)):
            goal_state = current_state
            # print"goal_state",goal_state

            break
        elif (current_state in visited.keys()):
            continue
        else:
            visited[current_state] = 1;
            succ = problem.getSuccessors(current_state)
            map_succ[current_state] = succ
            # print "succ_list",succ
            for i in (range(0, len(succ))):
                if not (succ[i][0] in visited.keys()):
                    # print"push",succ[i][0]
                    q.push(succ[i][0])

                if not (succ[i][0] in list_parent.keys()):
                    # print "parent",current_state,succ[i][0]
                    list_parent[succ[i][0]] = current_state

    current = goal_state
    # print "List-succ",map_succ[start_state]
    while not (current == start_state):
        # print "curr",current,list_parent[current]
        list_state.append(current)
        current = list_parent[current]
    # print current
    list_state.append(start_state)
    # print "List",list_state

    i = len(list_state)
    i = i - 1
    while (i >= 1):
        cc = list_state[i]
        # print "cc",list_state[i]
        dd = map_succ[cc]
        for j in range(0, len(dd)):
            if (dd[j][0] == list_state[i - 1]):
                ans_direct.append(dd[j][1])

        i = i - 1

    # print ans_direct

    return ans_direct


def uniformCostSearch(problem):
    # author junruitian
    """Search the node of least total cost first."""
    """ Dijkstra shorest path with null heuristic"""
    # 扩展的是路径消耗g(n)最小的节点n,用优先队列来实现，对解的路径步数不关心，只关心路径总代价。
    # 即使找到目标节点也不会结束，而是再检查新路径是不是要比老路径好，确实好，则丢弃老路径。

    start_state = problem.getStartState()
    # print start_state
    q = util.PriorityQueue()
    ans_direct = []
    visited = {}
    list_parent = {}
    map_succ = {}
    list_state = []
    cost = {}
    q.update(start_state, 0)
    cost[start_state] = 0

    while not (q.isEmpty()):
        current_state = q.pop()
        # print "Current",current_state

        if (problem.isGoalState(current_state)):
            goal_state = current_state
            # print"goal_state",goal_state
            break
        elif (current_state in visited.keys()):
            continue
        else:
            visited[current_state] = 1;
            succ = problem.getSuccessors(current_state)
            map_succ[current_state] = succ
            # print "succ_list",succ
            for i in (range(0, len(succ))):
                # if not(succ[i][0] in visited.keys()):
                # print"push",succ[i][0]
                if (succ[i][0] in cost.keys()):
                    if (succ[i][2] + cost[current_state] < cost[succ[i][0]]):
                        cost[succ[i][0]] = succ[i][2] + cost[current_state]
                        list_parent[succ[i][0]] = current_state
                else:
                    cost[succ[i][0]] = succ[i][2] + cost[current_state]
                    list_parent[succ[i][0]] = current_state
                # print succ[i][0],cost[succ[i][0]]
                q.update(succ[i][0], cost[succ[i][0]])

            # if not (succ[i][0] in list_parent.keys()):
            # print "parent",current_state,succ[i][0]
            # list_parent[succ[i][0]]=current_state

    current = goal_state
    # print "List-succ",map_succ[start_state]
    while not (current == start_state):
        # print "curr",current,list_parent[current]
        list_state.append(current)
        current = list_parent[current]
    # print current
    list_state.append(start_state)
    # print "List",list_state

    i = len(list_state)
    i = i - 1
    while (i >= 1):
        cc = list_state[i]
        # print "cc",list_state[i]
        dd = map_succ[cc]
        for j in range(0, len(dd)):
            if (dd[j][0] == list_state[i - 1]):
                ans_direct.append(dd[j][1])

        i = i - 1

    # print ans_direct

    return ans_direct


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    start_state = problem.getStartState()
    # print start_state
    q = util.PriorityQueue()
    ans_direct = []
    visited = {}
    list_parent = {}
    map_succ = {}
    list_state = []
    cost = {}
    q.update(start_state, 0)
    cost[start_state] = 0

    while not (q.isEmpty()):
        current_state = q.pop()
        # print "Current",current_state

        if (problem.isGoalState(current_state)):
            goal_state = current_state
            # print"goal_state",goal_state
            break
        elif (current_state in visited.keys()):
            continue
        else:
            visited[current_state] = 1;
            succ = problem.getSuccessors(current_state)
            map_succ[current_state] = succ
            # print "succ_list",succ
            for i in (range(0, len(succ))):
                # if not(succ[i][0] in visited.keys()):
                # print"push",succ[i][0]
                h = heuristic(succ[i][0], problem)
                if (succ[i][0] in cost.keys()):
                    if (succ[i][2] + cost[current_state] + h < cost[succ[i][0]]):
                        cost[succ[i][0]] = succ[i][2] + cost[current_state]
                        list_parent[succ[i][0]] = current_state
                else:
                    cost[succ[i][0]] = succ[i][2] + cost[current_state]
                    list_parent[succ[i][0]] = current_state
                # print succ[i][0],cost[succ[i][0]]
                q.update(succ[i][0], cost[succ[i][0]] + h)

            # if not (succ[i][0] in list_parent.keys()):
            # print "parent",current_state,succ[i][0]
            # list_parent[succ[i][0]]=current_state

    current = goal_state
    # print "List-succ",map_succ[start_state]
    while not (current == start_state):
        # print "curr",current,list_parent[current]
        list_state.append(current)
        current = list_parent[current]
    # print current
    list_state.append(start_state)
    # print "List",list_state

    i = len(list_state)
    i = i - 1
    while (i >= 1):
        cc = list_state[i]
        # print "cc",list_state[i]
        dd = map_succ[cc]
        for j in range(0, len(dd)):
            if (dd[j][0] == list_state[i - 1]):
                ans_direct.append(dd[j][1])

        i = i - 1
    # print ans_direct

    return ans_direct


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
