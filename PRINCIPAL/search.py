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
            if (childState not in explored) or (childState not in frontierState.list):
                frontierNode.push(childNode)
                frontierState.push(childState)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    node = (problem.getStartState(),[])
    frontierNode = util.Queue()
    explored = []
    frontierNode.push(node)

    while(True):
        if frontierNode.isEmpty():
        	return None

    	node = frontierNode.pop()

        if(problem.isGoalState(node[0])):
        	return node[1]

    	explored.append(node[0])

    	for action in problem.getSuccessors(node[0]):

    		stateList = [x[0] for x in frontierNode.list]
    		if(action[0] not in explored) and (action[0] not in stateList):
    			newActionList = node[1] + [action[1]]
    			frontierNode.push((action[0],newActionList))

    return None

def solution(node):
    result = []
    tNode = node
    while tNode[1] is not None:
        result.insert(0,tNode[2]) #se coloca de forma invertida
        tNode = tNode[1]
    #print result
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
    node = (problem.getStartState(),[])
    frontier = util.PriorityQueue()
    frontier.push(node,0)
    explored = []
    consistente = True
    while True:
        if(frontier.isEmpty()):
            return None

        node = frontier.pop()

        if problem.isGoalState(node[0]):
            if(consistente):
                print "La heuristica usada es consistente y por ende admisible"
            return node[1]

        if node[0] not in explored:

            for action in problem.getSuccessors(node[0]):
                if action[0] not in explored:
                    newActionList = node[1] + [action[1]]
                    newNode = (action[0],newActionList)

                    respuesta = heuristic(newNode[0],problem)
                    costo =problem.getCostOfActions(newActionList)
                    #respuestaAnterior = heuristic(new)

                    if (newActionList):
                    	ultimoMov = newActionList.pop()
                    	newActionList.append(ultimoMov)
                    	if(ultimoMov == "North"):
                    		posAnterior = (newNode[0][0][0],newNode[0][0][1]-1)
                    	elif(ultimoMov == "Sur"):
                    		posAnterior = (newNode[0][0][0],newNode[0][0][1]+1)
                    	elif (ultimoMov == "East"):
                    		posAnterior = (newNode[0][0][0]-1,newNode[0][0][1])
                    	else:
                    		posAnterior = (newNode[0][0][0]+1,newNode[0][0][1])

                    	lista=[]
                    	if (newNode[0][0] in problem.corners):
                    		for i in newNode[0][1]:
                    			lista.append(i)
                    		lista.append(newNode[0][0])
                    	respuestaAnterior= heuristic([posAnterior,lista],problem)
                    else:
                    	respuestaAnterior=0

                    if (respuestaAnterior > costo + respuesta ) :
                    	print " No es admisible"
                        consistente = False

                    funcionOrden = costo + respuesta
                    frontier.push(newNode,funcionOrden)

        explored.append(node[0])

    util.raiseNotDefined()


def recursiveDLS(node,problem,limit,explored):

    if problem.isGoalState(node[0]):
        return node[1]
    elif node[0] in explored:
    	return "exploredNode"
    elif limit==0:
        return "cutoff"
    else:
    	explored.append(node[0])
        cutoff_ocurred = False
        for action in problem.getSuccessors(node[0]):
            newActionList = node[1] + [action[1]]
            newNode = (action[0],newActionList)

            result = recursiveDLS(newNode,problem,limit-1,explored)
            if result == "cutoff":
                cutoff_ocurred=True

            elif result == "exploredNode":
            	continue

            elif result != "failure":
                return result
        if cutoff_ocurred:
            return "cutoff"
        else:
            return "failure"

def depthLimitedSearch(problem,limit):
    node = (problem.getStartState(),[])
    explored = []
    return recursiveDLS(node,problem,limit,explored)

def iDeepeningSearch(problem):
    depth = 0
    while True:
        result = depthLimitedSearch(problem,depth)
        depth += 1
        if result != "cutoff":
        	print depth-1
        	return result


def bidirectionalSearch(problem):
	#Se esta utilizando como referencial el pseudocodigo de Bidirectional Search
	#de la pagina web http://planning.cs.uiuc.edu/node50.html visitado el dia jueves 3 de mayo
    nodeStateInitial = problem.getStartState()
    tNodeInitial = (nodeStateInitial,[])

    nodeStateGoal = problem.getStartStateInv()
    tNodeGoal = (nodeStateGoal,[])

    frontierNodeInitial = util.Queue()
    frontierNodeGoal = util.Queue()

    frontierNodeInitial.push(tNodeInitial)
    frontierNodeGoal.push(tNodeGoal)

    explored = []

    node1 = (problem.getStartState(),[])
    node2 = (problem.getStartStateInv(),[])

    while not(frontierNodeInitial.isEmpty()) and not(frontierNodeGoal.isEmpty()):

        if not(frontierNodeInitial.isEmpty()):
        	node1 = frontierNodeInitial.pop()

        	sucesores = problem.getSuccessors(node1[0])

        	for sucesor in sucesores:
        		listaPrimerasComponentes = [x[0] for x in frontierNodeGoal.list]
        		if(sucesor[0] in listaPrimerasComponentes):
        			indAcciones = listaPrimerasComponentes.index(sucesor[0])
        			accionesResult = frontierNodeGoal.list[indAcciones]
        			accionesResult[1].reverse()
        			invertirDireccionesListaAcciones(accionesResult[1])
        			return node1[1] + [sucesor[1]] + accionesResult[1]

        	if problem.isGoalState(node1[0]) or (node1[0] in [x[0] for x in frontierNodeGoal.list]): #verifica si llego al objetivo o si ya hay interseccion entre las fronteras
        		node2[1].reverse()
        		invertirDireccionesListaAcciones(node2[1])

        		return node1[1]

        	for action in sucesores:
        		node2 = (action[0], node1[1] + [action[1]])
        		if node2[0] not in explored:
        			explored.append(node2[0])
        			frontierNodeInitial.push(node2)

        if not(frontierNodeGoal.isEmpty()):
        	node2 = frontierNodeGoal.pop()

        	sucesores = problem.getSuccessorsInv(node2[0])

        	for sucesor in sucesores:
        		listaPrimerasComponentes = [x[0] for x in frontierNodeInitial.list]

        		if(sucesor[0] in listaPrimerasComponentes):
        			indAcciones = listaPrimerasComponentes.index(sucesor[0])
        			accionesResult = frontierNodeInitial.list[indAcciones]

        			accionesResult[1].reverse()
        			invertirDireccionesListaAcciones(accionesResult[1])
        			return node1[1] + accionesResult[1]

        	if problem.isGoalStateInv(node2[0]) or (node2[0] in [x[0] for x in frontierNodeInitial.list]):
        		invertirDireccionesListaAcciones(node2[1])
        		return node2[1]

        	for action in sucesores:
        		node1 = (action[0], node2[1] + [action[1]])#completar la lista de acciones
        		if node1[0] not in explored:
        			explored.append(node1[0])
        			frontierNodeGoal.push(node1)
    return None


def invertirDireccionesListaAcciones(lista):
	for i in range(len(lista)):
		if(lista[i] == 'North'):
			lista[i] = 'South'
		elif(lista[i] == 'South'):
			lista[i] = 'North'
		elif(lista[i] == 'East'):
			lista[i] = 'West'
		else:
			lista[i] = 'East'

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iDeepeningSearch
bs = bidirectionalSearch
