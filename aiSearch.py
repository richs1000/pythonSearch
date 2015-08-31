#
# aiSearch.py
#
# Rich Simpson
# December 11, 2014
#
# This code is used in my CSCI 355 course. It implements the
# following search algorithms:
# * Breadth-First Search
# * Uniform Cost Search
# * Greedy Search
# * A* Search
#
# The state graph and search tree are implemented using adjacency lists:
#
#    adjacencyList = {fromNodeID:[edgeObj, edgeObj, edgeObj], fromNodeID:[edgeObj, edgeObj]}


__author__ = 'rsimpson'


from Queue import PriorityQueue


class GraphNode():
    def __init__(self):
        """
        A graph node has a list of neighbors and a flag that indicates
        if the node has already been expanded
        """
        # set the node's neighbors to an empty list
        self.neighbors = []
        # the visited flag is used in graph search so that each
        # state is only considered once
        self.expanded = False

    def __eq__(self, _other):
        """
        Test the equality of two state space graph nodes. This is
        used by the closed list.
        Should be reimplemented in the derived class.
        """
        raise NotImplementedError()

    def prettyPrint(self):
        """
        Pretty-prints the contents of a state space graph node.
        Should be reimplemented in the derived class.
        """
        raise NotImplementedError()


class Edge:
    def __init__(self, _fromNodeID, _toNodeID, _edgeCost):
        """
        An edge has a start node, an end node and a cost
        :return:
        """
        # set starting node for edge
        self.fromNodeID = _fromNodeID
        # set ending node for edge
        self.toNodeID = _toNodeID
        # set edge cost
        self.edgeCost = _edgeCost


class StateSpaceGraph:
    def __init__(self, _undirectedGraph):
        """
        A state space graph consists of a list of nodes and an
        adjacency list (implemented as a dictionary of dictionaries)
        of edges
        """
        # the node list starts out empty
        self.nodeList = []
        # the adjacency list starts out empty
        self.adjacencyList = {}
        # a flag that is true when the graph is undirected and false when
        # it is directed (i.e., edges between nodes have a direction)
        self.undirected = _undirectedGraph

    def equivalentNodeExists(self, _node):
        """
        Compares the node passed as an argument to all the nodes that are
        already in the node dictionary. If there is an equivalent node then
        it returns true
        """
        # loop through all the nodes in the node list
        for n in self.nodeList:
            # compare the nodes (uses the __eq__ function)
            if n == _node:
                # if we find a match then return true
                return True
        # no match was found
        return False

    def addNode(self, _node):
        """
        adds a node to the list of nodes
        """
        # if the nodeID is already taken, don't do anything
        if (self.equivalentNodeExists(_node)):
            return -1
        # add the node to the node dictionary
        self.nodeList.append(_node)
        # return the ID of the new node
        return len(self.nodeList) - 1

    def findNode(self, _node):
        # make sure a node with fromNodeID exists
        if not self.equivalentNodeExists(_node):
            return -1
        # return a pointer to the node
        return self.nodeList.index(_node)

    def addEdgeHelper(self, _fromNodeID, _toNodeID, _edgeCost):
        """
        Creates an edge and adds it to the adjacency list
        :return:
        """
        # make sure a node with fromNodeID exists
        if _fromNodeID >= len(self.nodeList):
            return
        # make sure a node with fromNodeID exists
        if _toNodeID >= len(self.nodeList):
            return
        # make sure cost is within bounds
        if _edgeCost < 0:
            return
        # make sure edge doesn't already exist
        if _fromNodeID in self.adjacencyList:                # is the fromNodeID a key in the dict?
            for edgeObj in self.adjacencyList[_fromNodeID]:  # loop through list associated with fromNodeID
                if _toNodeID == edgeObj.toNodeID:            # is there an edge with the same toNodeID?
                    # print "\t---edge already exists---"
                    return                                  # then this edge already exists
        # create the new edge
        edgeObjNew = Edge(_fromNodeID, _toNodeID, _edgeCost)
        # insert edge into adjacency list
        if _fromNodeID in self.adjacencyList:                    # does a list already exist with this key?
            self.adjacencyList[_fromNodeID].append(edgeObjNew)   # then append this edge to that list
        else:                                                   # otherwise
            self.adjacencyList[_fromNodeID] = [edgeObjNew]       # create a new list and attache to fromNodeID as key
        # update node with new neighbor
        # self.nodeDict is a dictionary
        # self.nodeDict[fromNodeID] is a GraphNode object
        # self.nodeDict[fromNodeID].neighbors is a list
        self.nodeList[_fromNodeID].neighbors.append(_toNodeID)

    def addEdge(self, _fromNodeID, _toNodeID, _edgeCost):
        """
        This function adds an edge between two nodes. If the graph is directed
        then we only add one edge. If the graph is undirected, we add two edges
        (one in each direction). This isn't the most space-efficient way to do
        this.
        """
        # add the edge
        self.addEdgeHelper(_fromNodeID, _toNodeID, _edgeCost)
        # if this graph is undirected
        if self.undirected:
            # add a second edge in the opposite direction
            self.addEdgeHelper(_toNodeID, _fromNodeID, _edgeCost)

    def dumpGraph(self):
        """
        This function prints out the graph
        :return:
        """
        # print out nodes
        for nodeID, nodeObj in self.nodeList:
            print "nodeID = " + nodeObj.nodeID
            # for neighbor in nodeObj.neighbors:
            #     print "neighbor: " + neighbor
        # print out edges
        for edgeID, edgeList in self.adjacencyList.iteritems():
            for edgeObj in edgeList:
                print "edge from " + str(edgeObj.fromNodeID) + " to " + str(edgeObj.toNodeID) + " with cost " + str(edgeObj.edgeCost)


class FringeNode():
    def __init__(self, _treeNodeID, _priority):
        """
        I use this class to put values into the priority queue
        :return:
        """
        # which tree node does this point to?
        self.treeNodeID = _treeNodeID
        # set the priority value
        self.priority = _priority

    def __cmp__(self, other):
        """
        Compare two nodes based on cost
        :param other:
        :return:
        """
        return cmp(self.priority, other.priority)


class TreeNode():
    def __init__(self):
        """
        A tree node represents a path through the graph. Each tree node has a list of children,
        a single parent, a cost (g), and a heuristic (h) value,
        """
        # set the path represented by tree node to an empty list
        self.path = []
        # set the index of the node in the tree node list
        self.nodeID = 0
        # set the children to an empty list
        self.children = []
        # set the parent to an empty value
        self.parent = None
        # set the cost to zero
        self.cost = 0
        # set the heuristic value to zero
        self.heuristic = 0


class SearchTree:
    def __init__(self):
        """
        A search tree consists of a dictionary of nodes and an
        adjacency list (implemented as a dictionary) of edges
        """
        # the node list starts out empty
        self.nodeList = []
        # the adjacency list starts out empty
        self.adjacencyList = {}

    def addNode(self, _parentID, _endOfPath, _cost, _hValue):
        """
        Creates a node and adds it to the end of the
        list of nodes in the tree. It also creates an edge between
        the new node and its parent, and then calculates the
        total cost of the path to the node (the g value)
        :param nodeID:
        :return: newNode
        """
        # make sure cost is within bounds
        if _cost < 0:
            return None
        # create a new node
        newNode = TreeNode()
        # set the heuristic (h) value
        newNode.heuristic = _hValue
        # if the node has no parent, and the root hasn't already
        # been specified, then this is the root. The parent
        # defaults to None, the cost defaults to zero and
        # children defaults to an empty list.
        # otherwise, this is not the root node, so fill in the
        # right values
        if (len(self.nodeList) > 0):
            # set its parent
            newNode.parent = _parentID
            # calculate the cost to reach this node:
            # cost of its parent + cost from parent to this node
            newNode.cost = _cost + self.nodeList[_parentID].cost
            # set its path equal to its parent's path
            newNode.path = list(self.nodeList[_parentID].path)
        # add last node on path
        newNode.path.append(_endOfPath)
        # add the node to the end of the node list
        self.nodeList.append(newNode)
        # get the index of the node
        newNodeID = len(self.nodeList) - 1
        # set the ID of the node
        newNode.nodeID = newNodeID
        # if this isn't the root node, then we need to create
        # an edge from the parent to this node
        if (_parentID is not None):
            # create the new edge
            # newEdge = Edge(parentID, nodeID, cost)
            # insert edge into adjacency list
            # self.adjacencyList[parentID] = newEdge
            # create the new edge
            edgeObjNew = Edge(_parentID, newNodeID, _cost)
            # insert edge into adjacency list
            if _parentID in self.adjacencyList:                    # does a list already exist with this key?
                self.adjacencyList[_parentID].append(edgeObjNew)   # then append this edge to that list
            else:                                                  # otherwise
                self.adjacencyList[_parentID] = [edgeObjNew]       # create a new list and attache to fromNodeID as key
            # update parent node with new child
            # self.nodeDict is a dictionary
            # self.nodeDict[fromNodeID] is a GraphNode object
            # self.nodeDict[fromNodeID].children is a list
            self.nodeList[_parentID].children.append(newNodeID)
        return newNode

    def dumpTree(self):
        """
        This function prints out the search tree
        :return:
        """
        # print out nodes
        for nodeID, nodeObj in self.nodeList:
            print "nodeID = " + nodeObj.nodeID + " parent = " + str(nodeObj.parent) + " g = " + str(nodeObj.cost) + " h = " + str(nodeObj.heuristic)
            # for child in nodeObj.children:
            #     print "child: " + child
        # print out edges
        # for edgeID, edgeObj in self.adjacencyList.iteritems():
        #     print "edge from " + str(edgeObj.fromNodeID) + " to " + str(edgeObj.toNodeID) + " with cost " + str(edgeObj.edgeCost)
        for edgeID, edgeList in self.adjacencyList.iteritems():
            for edgeObj in edgeList:
                print "edge from " + str(edgeObj.fromNodeID) + " to " + str(edgeObj.toNodeID) + " with cost " + str(edgeObj.edgeCost)

    def tracePath(self, _leafNodeID):
        """
        This function returns a list with the path from a leaf node
        back to the root node
        """
        # start with empty list for path
        nodePath = []
        # initialize the node id used to loop through tree to start at the leaf
        nodeID = _leafNodeID
        # loop through the nodes until you get to the root
        while self.nodeList[nodeID].parent is not None:
            # add node to path
            nodePath.append(nodeID)
            # get the parent
            nodeID = self.nodeDict[nodeID].parent
        # add last node to path
        nodePath.append(nodeID)
        # return the list
        return nodePath


class SearchAlgorithm():
    def __init__(self, _graphSearch, _undirectedGraph):
        # flag for whether we use a closed list or not
        #   * if we use a closed list, then this is a graph search
        #   * if we don't use a closed list, then this is a tree
        #     search and the same state space graph node
        #     will get considered multiple times
        self.graphSearch = _graphSearch
        # create the closed list
        self.closedList = []
        # create the state space graph
        self.ssg = StateSpaceGraph(_undirectedGraph)
        # create the search tree
        self.st = SearchTree()

    def printPath(self, nodeIDs):
        """
        Given a list of node ID's, print out the state represented by each one
        """
        for nodeID in nodeIDs:
            # get a pointer to the corresponding graph node
            graphNode = self.ssg.nodeList[nodeID]
            # print the state represented by this node
            graphNode.prettyPrint()

    def search(self, _startNode):
        """
        Implements a tree search strategy. The strategy is dictated by how items are ordered in
        the fringe:
        * Breadth-First Search - items ordered by the order in which they are added to the fringe
        * Uniform Cost Search - items are ordered by total path cost
        * Greedy Search - items are ordered by distance to the goal
        * A* Search - items are ordered by the sum of total path cost and distance to the goal
        startNode is a node in the graph
        """
        loopCount = 0
        # add the start node to the state space graph
        self.ssg.addNode(_startNode)
        # make the start node the root of the tree: parent = none, endOfPath = 0, cost = 0, heuristic
        newTreeNode = self.st.addNode(None, 0, 0, self.heuristic(_startNode))
        # create an object to push onto front of priority queue
        newFringeNode = FringeNode(0, self.priority(newTreeNode))
        # create the priority queue
        fringe = PriorityQueue()
        # put start node on fringe
        fringe.put(newFringeNode)
        # keep looping until we run out of fringe or reach our goal
        while not fringe.empty():
            # pull next node off fringe - remember it's a queue so we
            # remove from the front
            nextFringeNode = fringe.get()
            # get a pointer to the corresponding tree node
            treeNode = self.st.nodeList[nextFringeNode.treeNodeID]
            # get the index of the graph node at the end of the tree node's path
            graphNodeID = treeNode.path[len(treeNode.path) - 1]
            # get a pointer to the corresponding graph node
            graphNode = self.ssg.nodeList[graphNodeID]
            # add a pointer to the node to the closed list
            self.closedList.append(graphNode)
            # if we found the goal than return the answer
            if (self.goalTest(graphNode)):
                print "---victoia---"
                # get the list of nodes from goal back to start
                pathList = list(treeNode.path)
                # print the path
                self.printPath(pathList)
                # return the solution path
                return pathList
            # expand the node to get all the potential next nodes
            nextStates = self.expand(graphNode)
            # loop through the potential next nodes
            for childState in nextStates:
                # if we are doing graph search...
                if self.graphSearch:
                    # and this child state is already in the closed list
                    if childState in self.closedList:
                        # then skip the rest of this loop
                        continue
                # add the new node to the state space graph
                newNodeID = self.ssg.addNode(childState)
                # if we didn't create a new node then skip the rest
                if (newNodeID == -1):
                    continue
                # add edge to state space graph - either one edge or two will be added, depending
                # on whether the graph is directed or undirected
                self.ssg.addEdge(graphNodeID, newNodeID, self.cost(graphNode, childState))
                # add the new node to the search tree: parent = none, endOfPath = 0, cost = 0, heuristic
                childTreeNode = self.st.addNode(graphNodeID, newNodeID, self.cost(graphNode, childState), self.heuristic(childState))
                # create an object to push onto priority queue
                childFringeNode = FringeNode(childTreeNode.nodeID, self.priority(childTreeNode))
                # put new node on fringe
                fringe.put(childFringeNode)
            # we've gone through the whole search algorithm one more time
            loopCount += 1
            # update the user on the number of times we've gone through the search algorithm
            if loopCount % 10000 == 0:
                print "loopCount = " + str(loopCount)

    def goalTest(self, graphNode):
        """
        Returns true if we reached a goal state
        Should be reimplemented in the derived class.
        """
        raise NotImplementedError()

    def expand(self, graphNode):
        """
        This function takes a state as input and returns a list of possible
        next states as output. We change states by "swapping" the blank tile
        with the tiles in any of the four cardinal directions.
        Should be reimplemented in the derived class.
        """
        raise NotImplementedError()

    def heuristic(self, graphNode):
        """
        This function calculates a heuristic value for
        a state which estimates the distance from the
        state to the goal
        Should be reimplemented in the derived class.
        """
        raise NotImplementedError()

    def cost(self, graphNode1, graphNode2):
        """
        This function returns the cost of an edge between two graph nodes
        Should be reimplemented in the derived class.
        """
        raise NotImplementedError()

    def priority(self, treeNode):
        """
        The priority determines the order of the fringe, which dictates the search algorithm:
        * Breadh-First Search - items ordered by the order in which they are added to the fringe
        * Uniform Cost Search - items are ordered by total path cost
        * Greedy Search - items are ordered by distance to the goal
        * A* Search - items are ordered by the sum of total path cost and distance to the goal
        Should be reimplemented in the derived class.
        """
        raise NotImplementedError()


class AStarSearch(SearchAlgorithm):
    def __init__(self, graphSearch, undirectedGraph):
        SearchAlgorithm.__init__(self, graphSearch, undirectedGraph)

    def priority(self, treeNode):
        """
        The priority determines the order of the fringe, which dictates the search algorithm:
        * A* Search - items are ordered by the sum of total path cost and distance to the goal
        """
        return treeNode.cost + treeNode.heuristic


class GreedySearch(SearchAlgorithm):
    def __init__(self, graphSearch, undirectedGraph):
        SearchAlgorithm.__init__(self, graphSearch, undirectedGraph)

    def priority(self, treeNode):
        """
        The priority determines the order of the fringe, which dictates the search algorithm:
        * Greedy Search - items are ordered by distance to the goal
        """
        return treeNode.heuristic


class UniformCostSearch(SearchAlgorithm):
    def __init__(self, graphSearch, undirectedGraph):
        SearchAlgorithm.__init__(self, graphSearch, undirectedGraph)

    def priority(self, treeNode):
        """
        The priority determines the order of the fringe, which dictates the search algorithm:
        * Uniform Cost Search - items are ordered by total path cost
        """
        return treeNode.cost



class BreadthFirstSearch(SearchAlgorithm):
    def __init__(self, graphSearch, undirectedGraph):
        SearchAlgorithm.__init__(self, graphSearch, undirectedGraph)

    def priority(self, treeNode):
        """
        The priority determines the order of the fringe, which dictates the search algorithm:
        * Breadth-First Search - items ordered by the order in which they are added to the fringe
        """
        # This is a little sneaky, because the items on the queue are ordered by when they are added
        # to the queue. Each item has a higher priority than any nodes created before it
        return treeNode.nodeID


class DepthFirstSearch(SearchAlgorithm):
    def __init__(self, graphSearch, undirectedGraph):
        SearchAlgorithm.__init__(self, graphSearch, undirectedGraph)

    def priority(self, treeNode):
        """
        The priority determines the order of the fringe, which dictates the search algorithm:
        * Breadth-First Search - items ordered by the order in which they are added to the fringe
        """
        # This is a little sneaky, because the items on the queue are ordered by when they are added
        # to the queue. Each item has a lower priority than any nodes created before it
        return -treeNode.nodeID
