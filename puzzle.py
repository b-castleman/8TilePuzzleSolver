from __future__ import division
from __future__ import print_function

import sys
import math
import time
import resource
import queue as Q

class Node(object):
    def __init__(self, priority):
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority


#### SKELETON CODE ####
## The Class that Represents the Puzzle
class PuzzleState(object):
    """
        The PuzzleState stores a board configuration and implements
        movement instructions to generate valid children.
    """
    def __init__(self, config, n, parent=None, action="Initial", cost=0):
        """
        :param config->List : Represents the n*n board, for e.g. [0,1,2,3,4,5,6,7,8] represents the goal state.
        :param n->int : Size of the board
        :param parent->PuzzleState
        :param action->string
        :param cost->int
        """
        if n*n != len(config) or n < 2:
            raise Exception("The length of config is not correct!")
        if set(config) != set(range(n*n)):
            raise Exception("Config contains invalid/duplicate entries : ", config)

        self.n        = n
        self.cost     = cost
        self.parent   = parent
        self.action   = action
        self.config   = config
        self.children = []

        # Get the index and (row, col) of empty block
        self.blank_index = self.config.index(0)

    def display(self):
        """ Display this Puzzle state as a n*n board """
        for i in range(self.n):
            print(self.config[self.n*i : self.n*(i+1)])

    def move_up(self):
        # See if it is a valid operation
        r = self.blank_index // self.n
        if(r == 0):
            return None

        # Copy old list
        newConfig = self.config.copy()

        # Move up is a valid operation
        newPos = self.blank_index - self.n

        # Swap positions
        self.swap(newConfig, self.blank_index, newPos)

        return PuzzleState(newConfig,self.n,self,"Up",1+self.cost)

      
    def move_down(self):
        # See if it is a valid operation
        r = self.blank_index // self.n
        if(r == self.n-1):
            return None

        # Copy old list
        newConfig = self.config.copy()

        # Move up is a valid operation
        newPos = self.blank_index + self.n

        # Swap positions
        self.swap(newConfig, self.blank_index, newPos)

        return PuzzleState(newConfig,self.n,self,"Down",1+self.cost)
      
    def move_left(self):
        # See if it is a valid operation
        c = self.blank_index % self.n
        if(c == 0):
            return None

        # Copy old list
        newConfig = self.config.copy()

        # Move up is a valid operation
        newPos = self.blank_index - 1

        # Swap positions
        self.swap(newConfig, self.blank_index, newPos)

        return PuzzleState(newConfig,self.n,self,"Left",1+self.cost)

    def move_right(self):
        # See if it is a valid operation
        c = self.blank_index % self.n
        if(c == self.n-1):
            return None

        # Copy old list
        newConfig = self.config.copy()

        # Move up is a valid operation
        newPos = self.blank_index + 1

        # Swap positions
        self.swap(newConfig,self.blank_index,newPos)

        return PuzzleState(newConfig,self.n,self,"Right",1+self.cost)

    @staticmethod
    def swap(curConfig,p1,p2):
        tmp = curConfig[p1]
        curConfig[p1] = curConfig[p2]
        curConfig[p2] = tmp


    def expand(self):
        """ Generate the child nodes of this node """
        
        # Node has already been expanded
        if len(self.children) != 0:
            return self.children
        
        # Add child nodes in order of UDLR
        children = [
            self.move_up(),
            self.move_down(),
            self.move_left(),
            self.move_right()]

        # Compose self.children of all non-None children states
        self.children = [state for state in children if state is not None]

        # Give all children self as a parent
        for child in self.children:
            child.parent = self

        return self.children

# Function that Writes to output.txt

### Students need to change the method to have the corresponding parameters
def writeOutput(finalState,nodesExpanded,maxDepth,maxResourcesUsed,netTime):
    actionsMade = []
    state = finalState
    while(state.action != "Initial"):
        actionsMade.append(state.action)
        state = state.parent
    actionsMade.reverse()
    fh = open("output.txt", "w")

    fh.write('path_to_goal: ')
    fh.write("[")
    for i in range(len(actionsMade)-1):
        fh.write("'" + actionsMade[i] + "', ")
    fh.write("'" + actionsMade[-1] + "']")
    fh.write('\ncost_of_path: ')
    fh.write(str(finalState.cost))
    fh.write('\nnodes_expanded: ')
    fh.write(str(nodesExpanded))
    fh.write('\nsearch_depth: ')
    fh.write(str(finalState.cost))
    fh.write('\nmax_search_depth: ')
    fh.write(str(maxDepth))
    fh.write('\nrunning_time: ')
    fh.write(str(round(netTime,8)))
    fh.write('\nmax_ram_usage: ')
    fh.write(str(round(maxResourcesUsed,8)))
    fh.close()


def bfs_search(initial_state):
    """BFS search"""
    explored = set() # set keeps track of configurations we've visited before

    frontier = [] # queue keeps track of the frontier to visit
    frontier.append(initial_state)

    nodesExpanded = 0
    maxDepth = 0
    maxResourcesUsed = 0
    startTime = time.time()
    dfs_start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

    while len(frontier) != 0:
        # Poll out the state (pop(0))
        state = frontier.pop(0)


        # See if this state has been cached yet, otherwise cache it
        tupleConfig = tuple(state.config)
        if tupleConfig in explored:
            continue
        explored.add(tupleConfig)

        # See if the end condition has been met
        if(test_goal(state.config,initial_state.n)):
            writeOutput(state,nodesExpanded,maxDepth,maxResourcesUsed,time.time()-startTime)
            return


        # Expand & Add to Frontier
        state.expand()
        nodesExpanded+=1
        for child in state.children:
            # Add to frontier
            frontier.append(child)

            # Update maximum depth for output
            maxDepth = max(maxDepth, child.cost)

        # Check Resources Used
        maxResourcesUsed = max((resource.getrusage(resource.RUSAGE_SELF).ru_maxrss-dfs_start_ram)/(2**20), maxResourcesUsed)

    return # Solution never found, no file to create

def dfs_search(initial_state):
    """DFS search"""
    explored = set()  # set keeps track of configurations we've visited before

    frontier = []  # stack keeps track of the node to visit
    frontier.append(initial_state)
    inFrontier = set()
    inFrontier.add(tuple(initial_state.config))

    nodesExpanded = 0
    maxDepth = 0
    maxResourcesUsed = 0
    startTime = time.time()
    dfs_start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

    while len(frontier) != 0:
        # Pop out the state
        state = frontier.pop()

        # See if this state has been cached yet, otherwise cache it
        tupleConfig = tuple(state.config)
        inFrontier.remove(tupleConfig)
        if tupleConfig in explored:
            continue
        explored.add(tupleConfig)

        # See if the end condition has been met
        if test_goal(state.config, initial_state.n):
            writeOutput(state, nodesExpanded, maxDepth, maxResourcesUsed, time.time() - startTime)
            return

        # Expand & Add to Frontier
        state.expand()
        nodesExpanded += 1

        # Reverse expanded order since stack is LIFO
        state.children.reverse()

        for child in state.children:
            # Set of stuff already in the frontier
            inFrontierCheck = tuple(child.config)
            if inFrontierCheck in inFrontier:
                continue
            # Add to frontier
            frontier.append(child)
            inFrontier.add(inFrontierCheck)

            # Update maximum depth for output
            maxDepth = max(maxDepth, child.cost)

        # Check Resources Used
        maxResourcesUsed = max((resource.getrusage(resource.RUSAGE_SELF).ru_maxrss-dfs_start_ram)/(2**20), maxResourcesUsed)

    return  # Solution never found, no file to create

def A_star_search(initial_state):
    """A * search"""
    """BFS search"""
    explored = set()  # set keeps track of configurations we've visited before

    frontier = Q.PriorityQueue()  # heap keeps track of the node to visit
    frontier.put((Node(calculate_total_cost(initial_state)),initial_state))

    nodesExpanded = 0
    maxDepth = 0
    maxResourcesUsed = 0
    startTime = time.time()
    dfs_start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss


    while frontier.qsize() != 0:
        # Pop out the state
        state = frontier.get()[1]

        # See if this state has been cached yet, otherwise cache it
        tupleConfig = tuple(state.config)
        if tupleConfig in explored:
            continue
        explored.add(tupleConfig)

        # See if the end condition has been met
        if (test_goal(state.config, initial_state.n)):
            writeOutput(state, nodesExpanded, maxDepth, maxResourcesUsed, time.time() - startTime)
            return

        # Expand & Add to Frontier
        state.expand()
        nodesExpanded += 1
        for child in state.children:
            # Add to frontier
            item = tuple([Node(calculate_total_cost(child)),child])
            frontier.put(item)

            # Update maximum depth for output
            maxDepth = max(maxDepth, child.cost)

        # Check Resources Used
        maxResourcesUsed = max((resource.getrusage(resource.RUSAGE_SELF).ru_maxrss - dfs_start_ram) / (2 ** 20),
                               maxResourcesUsed)

    return  # Solution never found, no file to create

def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""
    # Add heuristic first
    netSum = 0
    for i in range(state.n**2):
        if state.config[i] == 0:
            continue
        netSum += calculate_manhattan_dist(i,state.config[i],state.n)

    # Add current cost
    netSum += state.cost

    return netSum

def calculate_manhattan_dist(idx, value, n): # idx == current index, value == where it should be/what it is, n == size
    """calculate the manhattan distance of a tile"""
    desiredR = value // n
    desiredC = value % n

    curR = idx // n
    curC = idx % n

    return abs(desiredC-curC) + abs(desiredR-curR)

def test_goal(puzzle_state,n):
    """test the state is the goal state or not"""
    ### STUDENT CODE GOES HERE ###
    endGoal = list(range(n ** 2))  # end goal to aim for
    return puzzle_state == endGoal

# Main Function that reads in Input and Runs corresponding Algorithm
def main():
    search_mode = sys.argv[1].lower()
    begin_state = sys.argv[2].split(",")
    begin_state = list(map(int, begin_state))
    board_size  = int(math.sqrt(len(begin_state)))
    hard_state  = PuzzleState(begin_state, board_size)
    start_time  = time.time()
    
    if   search_mode == "bfs": bfs_search(hard_state)
    elif search_mode == "dfs": dfs_search(hard_state)
    elif search_mode == "ast": A_star_search(hard_state)
    else: 
        print("Enter valid command arguments !")
        
    end_time = time.time()
    print("Program completed in %.3f second(s)"%(end_time-start_time))

if __name__ == '__main__':
    main()
