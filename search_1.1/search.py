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

def depthFirstSearch(problem):
	#problem es de la clase SearchProblem definido en este archivo
	#problem es de una clase generica que puede ser FoodSearchProblem, PositionSearchProblem y CornersProblem
	#definidos en searchAgents.py

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
    print "Start:", problem.getStartState()
    #Se esta utilizando como referencia el codigo de las diapositivas "Unidad 2" del curso "Aplicaciones de Ciencias de la Computacion(Inteligencia Artificial)"
    #Del profesor Edwin Villanueva Talavera
    nodeState = problem.getStartState()
    nodeParent = None
    nodeAction = None
    tNode = (nodeState,nodeParent,nodeAction)

    frontierNode = util.Stack()
    frontierState = util.Stack()
    frontierNode.push(tNode)
    frontierState.push(nodeState)

    explored = []
    
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    #print "Start's successors:", problem.getSuccessors(problem.getStartState())

    while True:
    	if (frontierState.isEmpty()):
    		return None
    	tNode = frontierNode.pop()
    	if(problem.isGoalState(tNode[0])):
    		return solution(tNode)

    	explored.append(tNode[0])
    	
        for action in problem.getSuccessors(tNode[0]) :
    		childState = action[0]
    		childParent = tNode
    		childAction = action[1]
    		childNode = (childState,childParent,childAction)
    		if (childState not in explored) and (childState not in frontierState.list):
    			frontierNode.push(childNode)
    			frontierState.push(childState)

    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    print "Start:", problem.getStartState()
    #Se esta utilizando como referencia el codigo de las diapositivas "Unidad 2" del curso "Aplicaciones de Ciencias de la Computacion(Inteligencia Artificial)"
    #Del profesor Edwin Villanueva Talavera
    nodeState = problem.getStartState()
    nodeParent = None
    nodeAction = None
    tNode = (nodeState,nodeParent,nodeAction)

    frontierNode = util.Queue()
    frontierState = util.Queue()
    frontierNode.push(tNode)
    frontierState.push(nodeState)

    explored = []
    
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    #print "Start's successors:", problem.getSuccessors(problem.getStartState())

    while True:
        if (frontierState.isEmpty()):
            return None
        tNode = frontierNode.pop()
        if(problem.isGoalState(tNode[0])):
            return solution(tNode)

        explored.append(tNode[0])
        
        for action in problem.getSuccessors(tNode[0]) :
            childState = action[0]
            childParent = tNode
            childAction = action[1]
            childNode = (childState,childParent,childAction)
            if (childState not in explored) and (childState not in frontierState.list):
                frontierNode.push(childNode)
                frontierState.push(childState)

def solution(node):
    result = []
    tNode = node
    while tNode[1] is not None:
        result.insert(0,tNode[2]) #se coloca de forma invertida
        tNode = tNode[1]

    return result

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    explored = []
    frontier = util.PriorityQueue()
    tNode = (problem.getStartState(),[]) #el nodo se compone de estado y acciones realizadas hasta ahora
    frontier.push( tNode , 0) #se inserta en la cola de prioridad con un costo 0

    while(True):
    	if(frontier.isEmpty()):
    		return None

    	tNode = frontier.pop()
    	if(problem.isGoalState(tNode[0])):
    		return tNode[1] #se retorna la lista de acciones #probar cambios

    	if tNode[0] not in explored:
    		successors = problem.getSuccessors(tNode[0])
    		for action in successors:
    			nextState = action[0]
    			
    			if nextState not in explored:
    				newAction = action[1]
    				newActionList = tNode[1] + [newAction] #se anade la accion realizada a lista de acciones del nodo
    				newNode = (nextState,newActionList)
    				frontier.push(newNode,problem.getCostOfActions(newActionList))

    	explored.append(tNode[0]) #se anade el estado revisado a la lista de explorados

    return None



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


def recursiveDLS(node,problem,limit,explored):
    #node es un triple que se compone de estado, padre y accion
    #print "Nodo: ",node[0]

    #adicional
    #explored.append(node[0])
    #fin adicional

    if problem.isGoalState(node[0]):
        return solution(node)
    elif node[0] in explored:
    	#print "explorado"
    	return "exploredNode"
    elif limit==0:
        return "cutoff"
    else:
    	#print "hola"
    	explored.append(node[0])
        cutoff_ocurred = False
        for action in problem.getSuccessors(node[0]):
            childState = action[0]
            childParent = node
            childAction = action[1]
            childNode = (childState,childParent,childAction)

            result = recursiveDLS(childNode,problem,limit-1,explored)
            if result == "cutoff":
                cutoff_ocurred=True
            #adicional
            elif result == "exploredNode":
            	continue
            #fin adicional
            elif result != "failure":
                return result
        if cutoff_ocurred:
            return "cutoff"
        else:
            return "failure"

def depthLimitedSearch(problem,limit):
    nodeState = problem.getStartState()
    nodeParent = None
    nodeAction = None
    node = (nodeState,nodeParent,nodeAction)
    explored = []
    return recursiveDLS(node,problem,limit,explored)

def iDeepeningSearch(problem):
    depth = 190
    while True:
        result = depthLimitedSearch(problem,depth)
        depth += 1
        if result != "cutoff":
        	print depth-1
        	return result


def bidirectionalSearch(problem):
    nodeState = problem.getStartState()
    nodeParent = None
    nodeAction = None
    tNode = (nodeState,nodeParent,nodeAction)

    frontierNode = util.Queue()
    frontierState = util.Queue()
    frontierNode.push(tNode)
    frontierState.push(nodeState)

    explored = []
    
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    #print "Start's successors:", problem.getSuccessors(problem.getStartState())

    while True:
        if (frontierState.isEmpty()):
            return None
        tNode = frontierNode.pop()
        if(problem.isGoalState(tNode[0])):
            return solution(tNode)

        explored.append(tNode[0])
        
        for action in problem.getSuccessors(tNode[0]) :
            childState = action[0]
            childParent = tNode
            childAction = action[1]
            childNode = (childState,childParent,childAction)
            if (childState not in explored) and (childState not in frontierState.list):
                frontierNode.push(childNode)
                frontierState.push(childState)
    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iDeepeningSearch
bs = bidirectionalSearch