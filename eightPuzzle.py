__author__ = 'rsimpson'

from aiSearch import *


# This is a 2D array containing the Manhattan distance between
# each two pairs of tiles in the puzzle
# the key is the "from" tile and the list contains the distance
# to all the other tiles from that tile
manhattanDistances = {
    0:[0, 1, 2, 1, 2, 3, 2, 3, 4],
    1:[1, 0, 1, 2, 1, 2, 3, 2, 3],
    2:[2, 1, 0, 3, 2, 1, 4, 3, 2],
    3:[1, 2, 3, 0, 1, 2, 1, 2, 3],
    4:[2, 1, 2, 1, 0, 1, 2, 1, 2],
    5:[3, 2, 1, 2, 1, 0, 3, 2, 1],
    6:[2, 3, 4, 1, 2, 3, 0, 1, 2],
    7:[3, 2, 3, 2, 1, 2, 1, 0, 1],
    8:[4, 3, 2, 3, 2, 1, 2, 1, 0]
}

class EightPuzzleGraphNode(GraphNode):
    def __init__(self, tiles):
        """
        Stores a single arrangement of the 8-Puzzle in the state
        space graph.
        The state is represented as a string with 8 numbers and the
        letter B. The letter B indicates where the unoccupied square
        is located.
        """
        # call the parent constructor
        GraphNode.__init__(self)
        # store the arrangement of the tiles as a string
        # using the [:] construct makes a copy of the array, rather
        # than just storing a pointer to the tiles array
        self.tiles = tiles[:]

    def prettyPrint(self):
        """
        Pretty-prints the puzzle.
        """
        print " --- "
        print "|" + self.tiles[0:3] + "|"
        print "|" + self.tiles[3:6] + "|"
        print "|" + self.tiles[6:9] + "|"
        print " --- "
        print "\n"

    def __eq__(self, other):
        """
        Two graph nodes are equal if the contents of their tiles array
        are equal
        """
        if self.tiles == other.tiles:
            return True
        return False


class EightPuzzleSearch(AStarSearch):
    def __init__(self, graphSearch):
        # call parent initializer; True means use an undirected graph
        AStarSearch.__init__(self, graphSearch, True)
        # save the goal state
        self.goalState = EightPuzzleGraphNode("12345678B")

    def blankOnEdge(self, node, edge):
        """
        Returns true if the blank tile is on a given edge of the puzzle.
        node can be a state space graph node or a tree node.
        """
        if (edge == 'top') and (node.tiles[0] == 'B' or node.tiles[1] == 'B' or node.tiles[2] == 'B'):
            return True
        elif (edge == 'right') and (node.tiles[2] == 'B' or node.tiles[5] == 'B' or node.tiles[8] == 'B'):
            return True
        elif (edge == 'left') and (node.tiles[0] == 'B' or node.tiles[3] == 'B' or node.tiles[6] == 'B'):
            return True
        elif (edge == 'bottom') and (node.tiles[6] == 'B' or node.tiles[7] == 'B' or node.tiles[8] == 'B'):
            return True
        return False

    def movementOffset(self, direction):
        """
        Given a movement direction, return the offset from the current tile position to the new tile position
        """
        # find the tile above it - which is 3 back in the state string
        if (direction == 'up'): return -3
        # find the tile to the left of it - which is 1 back in the state string
        elif (direction == 'left'): return -1
        # find the tile to the right of it - which is 1 ahead in the state string
        elif (direction == 'right'): return +1
        # find the tile to the right of it - which is 1 ahead in the state string
        elif (direction == 'down'): return +3
        # uh oh, we got a bad direction...
        else:
            print "problem in move blank - bad direction given"
            exit()

    def moveBlank(self, node, direction):
        """
        This function returns a string state with the blank tile moved in the given direction:
        * up one row - i.e., swapped with the tile above it.
        * left one column - i.e., swapped with the tile next to it.
        * right one column - i.e., swapped with the tile next to it.
        * down one row - i.e., swapped with the tile below it.
        node can be a state space graph node or a tree node.
        """
        # create a copy of the node to work with
        newNode = EightPuzzleGraphNode(node.tiles)
        # get the string representing the position of all the tiles
        stateStr = newNode.tiles
        # find the position of the blank tile in the state str
        blankTileLoc = stateStr.index("B")
        # find the position of the tile that will be swapped with the blank tile
        swapTileLoc = blankTileLoc + self.movementOffset(direction)
        # save the tile to swap
        swapTile = stateStr[swapTileLoc]
        # swap the tiles
        newState = ''
        # put the string back together in the correct order with the tiles swapped
        if (direction == 'up' or direction == 'left'):
            newState = stateStr[:swapTileLoc] + "B" + stateStr[swapTileLoc+1:blankTileLoc] + str(swapTile) + stateStr[blankTileLoc+1:]
        elif (direction == 'down' or direction == 'right'):
            newState = stateStr[:blankTileLoc] + str(swapTile) + stateStr[blankTileLoc+1:swapTileLoc] + "B" + stateStr[swapTileLoc+1:]
        # replace the old state string with the new one
        newNode.tiles = newState
        # return the new state node
        return newNode

    def expand(self, graphNode):
        """
        This function takes a state as input and returns a list of possible
        next states as output. We change states by "swapping" the blank tile
        with the tiles in any of the four cardinal directions.
        node can be a state space graph node or a tree node.
        """
        # start with empty list of next possible states
        nextStateList = []
        # store edge:movement pairs
        edgeMovementDict = {'top':'up', 'bottom':'down', 'left':'left', 'right':'right'}
        # loop over the edge, movement pairs
        for edge, movement in edgeMovementDict.items():
            # if the tile isn't on the wrong edge
            if (not self.blankOnEdge(graphNode, edge)):
                # move the tile in the specified direction
                newNode = self.moveBlank(graphNode, movement)
                # add the new state to the list
                nextStateList.append(newNode)
        # return list of next states
        return nextStateList

    def goalTest(self, graphNode):
        """
        Returns true if we reached a goal state
        :return:
        """
        if (graphNode == self.goalState):
            return True
        return False

    def heuristic(self, graphNode):
        """
        This function uses the total manhattan distance as the heuristic
        """
        global manhattanDistances
        # initialize h value
        hVal = 0
        # sum of manhattan distances
        # loop through each location in the puzzle
        for fromTile in range(9):
            # what's in that location now?
            whatsThere = graphNode.tiles[fromTile]
            # where should that tile be?
            whereItBelongs = self.goalState.tiles.index(whatsThere)
            # what's the manhattan distance between where the tile is and where it should be?
            manDist = manhattanDistances[fromTile][whereItBelongs]
            # add that to the heuristic value
            hVal += manDist
        # return heuristic value
        return hVal

    def cost(self, graphNode1, graphNode2):
        """
        This function always returns 1 for the cost of an edge.
        """
        return 1


# create the search object - true means use graph search
eightPuzzle = EightPuzzleSearch(True)
# create the start node
startNode = EightPuzzleGraphNode("B87654321")
# start the search
eightPuzzle.search(startNode)
