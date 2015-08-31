__author__ = 'rsimpson'

from aiSearch import *


class FGWCGraphNode(GraphNode):
    def __init__(self, farmer, goat, wolf, cabbage):
        """
        Stores a single arrangement of the farmer, goat, wolf and cabbage.
        """
        # call the parent constructor
        GraphNode.__init__(self)
        # where is the farmer
        self.farmer = farmer
        # where is the goat
        self.goat = goat
        # where is the wolf
        self.wolf = wolf
        # where is the cabbage
        self.cabbage = cabbage

    def prettyPrint(self):
        """
        Pretty-prints the puzzle.
        """
        print " --- "
        print "farmer = " + str(self.farmer)
        print "goat = " + str(self.goat)
        print "wolf = " + str(self.wolf)
        print "cabbage = " + str(self.cabbage)
        print " --- "
        print "\n"

    def __eq__(self, other):
        """
        Two graph nodes are equal if the contents of their tiles array
        are equal
        """
        return self.farmer == other.farmer and self.wolf == other.wolf and self.cabbage == other.cabbage and self.goat == other.goat


class FGWCSearch(GreedySearch):
    def __init__(self, graphSearch):
        # call the parent initializer, set undirected graph to true
        GreedySearch.__init__(self, graphSearch, True)
        # save the goal state
        self.goalState = FGWCGraphNode('west', 'west', 'west', 'west')

    def oppositeShore(self, shore):
        """
        This function returns the opposite shore from what is passed in:
        east -> west
        west -> east
        :param shore:
        :return:
        """
        if shore == "east":
            return "west"
        return "east"

    def stateValid(self, graphNode):
        """
        This function takes in a state string and returns a boolean value of
        true if the state is valid and false if the state is not
        :param stateStr:
        :return:
        """
        # if the farmer is on one side and the goat and wolf are on the other
        if (graphNode.farmer != graphNode.goat) and (graphNode.farmer != graphNode.wolf):
            return False
        # if the farmer is on one side and the goat and cabbage are on the other
        elif (graphNode.farmer != graphNode.goat) and (graphNode.farmer != graphNode.cabbage):
            return False
        # otherwise, the state is valid
        return True

    def expand(self, graphNode):
        """
        This function takes a state as input and returns a list of possible
        next states as output
        """
        # start with empty list of next possible states
        nextStateList = []
        # farmer goes to opposite shore alone
        newState = FGWCGraphNode(self.oppositeShore(graphNode.farmer), graphNode.goat, graphNode.wolf, graphNode.cabbage)
        if self.stateValid(newState):
            nextStateList.append(newState)
        # farmer goes to opposite shore with goat
        if (graphNode.farmer == graphNode.goat):
            newState = FGWCGraphNode(self.oppositeShore(graphNode.farmer), self.oppositeShore(graphNode.goat), graphNode.wolf, graphNode.cabbage)
            if self.stateValid(newState):
                nextStateList.append(newState)
        # farmer goes to opposite shore with wolf
        if (graphNode.farmer == graphNode.wolf):
            newState = FGWCGraphNode(self.oppositeShore(graphNode.farmer), graphNode.goat, self.oppositeShore(graphNode.wolf), graphNode.cabbage)
            if self.stateValid(newState):
                nextStateList.append(newState)
        # farmer goes to opposite shore with cabbage
        if (graphNode.farmer == graphNode.cabbage):
            newState = FGWCGraphNode(self.oppositeShore(graphNode.farmer), graphNode.goat, graphNode.wolf, self.oppositeShore(graphNode.cabbage))
            if self.stateValid(newState):
                nextStateList.append(newState)
        # return list of next states
        return nextStateList

    def goalTest(self, graphNode):
        """
        Returns true if we reached a goal state
        :return:
        """
        return graphNode == self.goalState

    def heuristic(self, graphNode):
        """
        This function uses the number of tiles out of place
        as its heuristic
        """
        # initialize h value
        hVal = 0
        # how many things are out of place?
        if (graphNode.farmer != self.goalState.farmer):
            hVal += 1
        if (graphNode.goat != self.goalState.goat):
            hVal += 1
        if (graphNode.wolf != self.goalState.wolf):
            hVal += 1
        if (graphNode.cabbage != self.goalState.cabbage):
            hVal += 1
        # return heuristic value
        return hVal

    def cost(self, graphNode1, graphNode2):
        """
        This function always returns 1 for the cost of an edge.
        """
        return 1



# create the search object - graph search = true, undirected graph = true
fgwc = FGWCSearch(True)
# create the start state
startNode = FGWCGraphNode('east', 'east', 'east', 'east')
# start the search
pathList = fgwc.search(startNode)
