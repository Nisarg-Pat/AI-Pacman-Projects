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


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    STATE = "state"
    PATH = "path"
    initial_node = {STATE: problem.getStartState(), PATH: list()}
    frontier = util.Stack()
    explored = set()

    frontier.push(initial_node)

    while not frontier.isEmpty():
        current_node = frontier.pop()
        explored.add(current_node[STATE])
        if problem.isGoalState(current_node[STATE]):
            return current_node[PATH]

        for next_state, action, cost in problem.getSuccessors(current_node[STATE]):
            if next_state not in explored:
                frontier.push({STATE: next_state, PATH: current_node[PATH] + [action]})

    return []

    # P-1 Search - Question 1 Non-Code Questions:
    # 1) The exploration order is as expected because a path from a single adjacent state is fully explored
    # before path from another adjacent state is explored.
    # 2) Pacman does not go through all the explored squares as some explored states lead to a loop or a dead end
    # and the PATH value will not have directions for such cases.
    # 3) DFS does not guarantee the least cost solution.
    # 4) DFS will return a path that it finds out first by fully exploring a single state.
    # If a longer path is found before the optimal, than that path will be returned.


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    STATE = "state"
    PATH = "path"
    initial_node = {STATE: problem.getStartState(), PATH: list()}

    frontier = util.Queue()
    frontier_set = set()
    explored = set()

    frontier.push(initial_node)
    frontier_set.add(initial_node[STATE])

    while not frontier.isEmpty():
        current_node = frontier.pop()
        frontier_set.remove(current_node[STATE])
        explored.add(current_node[STATE])

        if problem.isGoalState(current_node[STATE]):
            return current_node[PATH]

        for next_state, action, cost in problem.getSuccessors(current_node[STATE]):
            if next_state not in explored and next_state not in frontier_set:
                frontier.push({STATE: next_state, PATH: current_node.get(PATH) + [action]})
                frontier_set.add(next_state)
    return []

    # P-1 Search - Question 2 Non-Code Questions:
    # BFS returns the least cost solution as the states closest to startState are explored first.

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    PATH = "path"
    COST = "cost"
    frontier = util.PriorityQueue()
    frontier_map = {}
    explored = set()

    initial_state = problem.getStartState()
    frontier.push(initial_state, 0)
    frontier_map[initial_state] = {PATH: list(), COST: 0}

    while not frontier.isEmpty():
        current_state = frontier.pop()
        current_path = frontier_map[current_state][PATH]
        current_cost = frontier_map[current_state][COST]
        frontier_map.pop(current_state)
        explored.add(current_state)

        if problem.isGoalState(current_state):
            return current_path

        for next_state, action, cost in problem.getSuccessors(current_state):
            if next_state not in explored:
                state_path = current_path + [action]
                state_cost = current_cost + cost
                if next_state not in frontier_map:
                    frontier.push(next_state, state_cost)
                    frontier_map[next_state] = {PATH: state_path, COST: state_cost}
                elif frontier_map[next_state][COST] > state_cost:
                    frontier.update(next_state, state_cost)
                    frontier_map[next_state] = {PATH: state_path, COST: state_cost}
    return []

    # P-1 Search - Question 3 Non-Code Questions:
    # StayEastSearchAgent - cost = 1
    # StayWestSearchAgent - cost = 68719479864


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    PATH = "path"
    COST = "cost"
    frontier = util.PriorityQueue()
    frontier_map = {}
    explored = set()

    initial_state = problem.getStartState()
    frontier.push(initial_state, 0 + heuristic(initial_state, problem))
    frontier_map[initial_state] = {PATH: list(), COST: 0}

    while not frontier.isEmpty():
        current_state = frontier.pop()
        current_path = frontier_map[current_state][PATH]
        current_cost = frontier_map[current_state][COST]
        frontier_map.pop(current_state)
        explored.add(current_state)

        if problem.isGoalState(current_state):
            return current_path

        for next_state, action, cost in problem.getSuccessors(current_state):
            if next_state not in explored:
                state_path = current_path + [action]
                state_cost = current_cost + cost
                if next_state not in frontier_map:
                    frontier.push(next_state, state_cost + heuristic(next_state, problem))
                    frontier_map[next_state] = {PATH: state_path, COST: state_cost}
                elif frontier_map[next_state][COST] > state_cost + heuristic(next_state, problem):
                    frontier.update(next_state, state_cost + heuristic(next_state, problem))
                    frontier_map[next_state] = {PATH: state_path, COST: state_cost}
    return []

    # P-1 Search - Question 4 Non-Code Questions:
    # |    openMaze    | Path Cost | Nodes Expanded | Score |
    # |     dfs        |    298    |       806      |  212  |
    # |     bfs        |     54    |       682      |  456  |
    # |     ucs        |     54    |       682      |  456  |
    # |astar(manhattan)|     54    |       535      |  456  |
    # dfs does not take the optimal path and pacman traverses long rows which can be skipped.
    # bfs and ucs works the same as cost function is constant between adjacent states.
    # astar with manhattan heuristic expands the least number of nodes.


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
