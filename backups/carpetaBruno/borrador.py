print "Start:", problem.getStartState()
    #Se esta utilizando como referencia el codigo de las diapositivas "Unidad 2" del curso "Aplicaciones de Ciencias de la Computacion(Inteligencia Artificial)"
    #Del profesor Edwin Villanueva Talavera
    nodeState = problem.getStartState()
    nodeParent = None
    nodeAction = None
    tNode = (nodeState,nodeParent,nodeAction)

    if problem.isGoalState(nodeState):
        result = []
        while tNode[1] is not None:
            result.insert(0,tNode[2]) #se coloca de forma invertida
            tNode = tNode[1]

        return result

    frontierNode = util.Queue()
    frontierState = util.Queue()
    frontierNode.push(tNode)
    frontierState.push(nodeState)

    explored = []
    
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    while True:
        if (frontierState.isEmpty()):
            return None
        tNode = frontierNode.pop()

        explored.append(tNode[0])
        for action in problem.getSuccessors(tNode[0]) :
            childState = action[0]
            childParent = tNode
            childAction = action[1]
            childNode = (childState,childParent,childAction)
            if (childState not in explored) or (childState not in frontierState.list):
                if(problem.isGoalState(tNode[0])):
                    #se retorna el resultado
                    result = []
                    while tNode[1] is not None:
                        result.insert(0,tNode[2]) #se coloca de forma invertida
                        tNode = tNode[1]

                    return result
                frontierNode.push(childNode)
                frontierState.push(childState)


##################################################################


    print "Start:", problem.getStartState()
    #Se esta utilizando como referencia el codigo de las diapositivas "Unidad 2" del curso "Aplicaciones de Ciencias de la Computacion(Inteligencia Artificial)"
    #Del profesor Edwin Villanueva Talavera
    nodeState = problem.getStartState()
    nodeParent = None
    nodeAction = None
    tNode = (nodeState,nodeParent,nodeAction)
    
    if(problem.isGoalState(nodeState)):
            return []

    frontierNode = util.Queue()
    frontierState = util.Queue()
    frontierNode.push(tNode)
    frontierState.push(nodeState)

    explored = [nodeState]


    
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    while True:
        if (frontierState.isEmpty()):
            return None
        tNode = frontierNode.pop()
        if(problem.isGoalState(tNode[0])):
            break

        explored.append(tNode[0])
        
        for action in problem.getSuccessors(tNode[0]) :
            childState = action[0]
            childParent = tNode
            childAction = action[1]
            childNode = (childState,childParent,childAction)
            
            if (childState not in explored) or (childState not in frontierState.list):
                frontierNode.push(childNode)
                frontierState.push(childState)


    result = []
    while tNode[1] is not None:
        result.insert(0,tNode[2]) #se coloca de forma invertida
        tNode = tNode[1]

    return result




##################################################################

    q = util.Queue()
    mark = []

    v = (problem.getStartState(), '', 0)
    if problem.isGoalState(v[0]):
        return []

    mark.append(v[0])

    q.push(v)
  
    nos = {str(v[0]): (None, None,)}

    while q :
        t = q.pop()
        print t
        if problem.isGoalState(t[0]) :
            v = t
            break
        for edge in problem.getSuccessors(t[0]):
            if not edge[0] in mark :
                mark.append(edge[0])
                nos[str(edge[0])] = (edge, t,)

                q.push(edge)

    _s = v
    _q = []
    while _s :
        if nos[str(_s[0])][1] :
            _q.append(_s[1])
        _s = nos[str(_s[0])][1]

    return _q[::-1]







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
    
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    while True:
        if (frontierState.isEmpty()):
            return None
        tNode = frontierNode.pop()
        print tNode[0],tNode[2]
        if(problem.isGoalState(tNode[0])):
            break

        explored.append(tNode[0])
        for action in problem.getSuccessors(tNode[0]) :
            childState = action[0]
            childParent = tNode
            childAction = action[1]
            childNode = (childState,childParent,childAction)
            if (childState not in explored) or (childState not in frontierState.list):
                frontierNode.push(childNode)
                frontierState.push(childState)


    result = []
    while tNode[1] is not None:
        result.insert(0,tNode[2]) #se coloca de forma invertida
        tNode = tNode[1]

    return result





    print "Start:", problem.getStartState()
    #Se esta utilizando como referencia el codigo de las diapositivas "Unidad 2" del curso "Aplicaciones de Ciencias de la Computacion(Inteligencia Artificial)"
    #Del profesor Edwin Villanueva Talavera
    nodeState = problem.getStartState()
    nodeParent = None
    nodeAction = None
    tNode = (nodeState,nodeParent,nodeAction)
    
    if(problem.isGoalState(nodeState)):
        return []

    frontierNode = util.Queue()
    frontierState = util.Queue()
    frontierNode.push(tNode)
    frontierState.push(nodeState)

    explored = [nodeState]


    
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    while True:
        if (frontierState.isEmpty()):
            return None
        tNode = frontierNode.pop()
        print tNode[0],tNode[2]
        if(problem.isGoalState(tNode[0])):
            break

        explored.append(tNode[0])
        for action in problem.getSuccessors(tNode[0]) :
            childState = action[0]
            childParent = tNode
            childAction = action[1]
            childNode = (childState,childParent,childAction)
            if (childState not in explored) or (childState not in frontierState.list):
                frontierNode.push(childNode)
                frontierState.push(childState)


    result = []
    while tNode[1] is not None:
        result.insert(0,tNode[2]) #se coloca de forma invertida
        tNode = tNode[1]

    return result










    nodeState = problem.getStartState()
    nodeParent = None
    nodeAction = None
    tNode = (nodeState,nodeParent,nodeAction)

    if problem.isGoalState(tNode[0]):
        return solution(tNode)

    frontierState = util.Queue()
    frontierState.push(nodeState)
    frontierNode = util.Queue()
    frontierNode.push(tNode)

    explored=[]

    while(True):
        if frontierState.isEmpty():
            return None
        tNode = frontierNode.pop()
        
        explored.append(tNode[0])      

        for action in problem.getSuccessors(tNode[0]):
            childState,childAction = action[0],action[1]
            childParent = tNode
            childNode = (childState,childParent,childAction)

            if (childState not in frontierState.list) or (childState not in explored):
                if problem.isGoalState(childState):
                    return solution(childNode)
                frontierState.push(childState)
                frontierNode.push(childNode)