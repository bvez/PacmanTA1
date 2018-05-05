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
    		if (childState not in explored) and (childState not in frontierState.list):
    			frontierNode.push(childNode)
    			frontierState.push(childState)

    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    #Se esta utilizando como referencia el codigo de las diapositivas "Unidad 2" del curso "Aplicaciones de Ciencias de la Computacion(Inteligencia Artificial)"
    #Del profesor Edwin Villanueva Talavera
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
    print result
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
        """if (frontierNodeInitial.isEmpty() and frontierNodeGoal.isEmpty()):
			print "sale"
			return None
        
        """
        if not(frontierNodeInitial.isEmpty()):
        	node1 = frontierNodeInitial.pop()

        	#for p in frontierNodeGoal.list :

        	#print "1 fronteraGoal",node1[0],[x[0] for x in frontierNodeGoal.list]
        	#print "se revisa el estado ",node1[0],"en la lista ",[x[0] for x in frontierNodeGoal.list]
        	
        	sucesores = problem.getSuccessors(node1[0])

        	#if (node1[0])
        	for sucesor in sucesores:
        		print sucesor

        		#sucesor es (estado,accion,costo)

        		print "1 se busca", sucesor[0], "en", [x[0] for x in frontierNodeGoal.list]
        		listaPrimerasComponentes = [x[0] for x in frontierNodeGoal.list]
        		if(sucesor[0] in listaPrimerasComponentes):
        			#node2[1].reverse()
        			#invertirDireccionesListaAcciones(node2[1])
        			indAcciones = listaPrimerasComponentes.index(sucesor[0])
        			accionesResult = frontierNodeGoal.list[indAcciones]
        			accionesResult[1].reverse()
        			invertirDireccionesListaAcciones(accionesResult[1])
        			print accionesResult[1]
        			#print node1[1]
        			#print node2[1]

        			#return node1[1]+node2[1]
        			return node1[1]+ [sucesor[1]] + accionesResult[1]

        	if problem.isGoalState(node1[0]) or (node1[0] in [x[0] for x in frontierNodeGoal.list]): #verifica si llego al objetivo o si ya hay interseccion entre las fronteras
        		print "resolvio1"
        		return node1[1]+node2[1]
        	
        	#print "sucesores1",node1[0],[x[0] for x in sucesores]
        	#print "1 Sucesores del estado ",node1[0],sucesores
        	for action in sucesores:
        		node2 = (action[0], node1[1] + [action[1]]) #completar la lista de acciones
        		if node2[0] not in explored:
        			explored.append(node2[0])
        			frontierNodeInitial.push(node2)
        			#print "frontera1 ",frontierNodeInitial.list
        		#else:
        			#resolve duplicate node2
		#print frontierNodeGoal.list
        if not(frontierNodeGoal.isEmpty()):
        	node2 = frontierNodeGoal.pop()
        	#print "2 fronteraInitial",node2[0],[x[0] for x in frontierNodeInitial.list]
        	#print "se revisa el estado ",node2[0],"en la lista ",[x[0] for x in frontierNodeInitial.list]
        	
        	sucesores = problem.getSuccessorsInv(node2[0])

        	for sucesor in sucesores:
        		print "2 se busca", sucesor[0], "en", [x[0] for x in frontierNodeInitial.list]
        		listaPrimerasComponentes = [x[0] for x in frontierNodeInitial.list]

        		if(sucesor[0] in listaPrimerasComponentes):
        			#node2[1].reverse()
        			#invertirDireccionesListaAcciones(node2[1])
        			indAcciones = listaPrimerasComponentes.index(sucesor[0])
        			accionesResult = frontierNodeInitial.list[indAcciones]
        			print accionesResult
        			#print node1[1]
        			#print node2[1]

        			#return node1[1]+node2[1]
        			return accionesResult[1]

        	if problem.isGoalStateInv(node2[0]) or (node2[0] in [x[0] for x in frontierNodeInitial.list]):
        		print "resolvio2"
        		#print node1
        		#print node2
        		return node1[1]+node2[1]
        	
        	#print "sucesores2",node2[0],[x[0] for x in sucesores]
        	#print "2 Sucesores del estado ",node2[0],sucesores
        	for action in sucesores:
        		node1 = (action[0], node2[1] + [action[1]])#completar la lista de acciones
        		if node1[0] not in explored:
        			explored.append(node1[0])
        			frontierNodeGoal.push(node1)
        			#print "frontera2 ",frontierNodeGoal.list
        		#else:
        			#resolve duplicate node1
	#if()
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