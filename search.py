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
    return  [s, s, w, s, w, w, s, w]

def DFS_append(fringe, node, parent):
    fringe.push(node)

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
    from util import Stack
    fringe = Stack()
    return Graph_Search_Path(problem, BFS_DFS_UCS_strategy, DFS_append, fringe)
    #util.raiseNotDefined()

def BFS_DFS_UCS_strategy(fringe):
    return fringe.pop()

def BFS_append(fringe, node, parent):
    fringe.push(node)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    fringe = Queue()
    return Graph_Search_Path(problem, BFS_DFS_UCS_strategy, BFS_append, fringe)

def UCS_append(fringe, node, parent):
    print 'node cost'
    print node
    print 'parent cost'
    print 'parent'
    print ''
    fringe.push(node, node[2] + parent[2])

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    fringe = PriorityQueue()
    return Graph_Search_Path(problem, BFS_DFS_UCS_strategy, UCS_append, fringe)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def Graph_Search_Path(problem, strategy, append, fringe):
    # only be coordinates
    #print 'Start of GSP'
    visited = []
    visited.append(problem.getStartState())
    
    if problem.isGoalState(problem.getStartState()):
        return []
    backPointers = {problem.getStartState() : None}
    for childNode in problem.getSuccessors(problem.getStartState()):
                backPointers[childNode] = problem.getStartState()
                append(fringe, childNode, (0,0,0))


    goalNode = None
    while not fringe.isEmpty():
        node = strategy(fringe)
        #print node
        if problem.isGoalState(node[0]):
            goalNode = node
            break
        if node[0] not in visited:
            visited.append(node[0])
            for childNode in problem.getSuccessors(node[0]):
                backPointers[childNode] = node
                append(fringe, childNode, node)
    #print '...'
    if(goalNode == None):
        print 'Path Not Found'
        return
    path = []
    path.insert(0, goalNode[1])
    curNode = backPointers[goalNode]
    print goalNode
    while backPointers[curNode] != None :
        #print curNode
        path.insert(0, curNode[1])
        curNode = backPointers[curNode]
    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
