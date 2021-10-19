from __future__ import division
from __future__ import print_function

import sys
import math
import time
import queue as Q
import copy
import heapq
import resource

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
            print(self.config[3*i : 3*(i+1)])

    def move_up(self):
        """ 
        Moves the blank tile one row up.
        :return a PuzzleState with the new configuration
        """
        c = list(self.config)
        idx = c.index(0)
        if idx > self.n-1:
            temp = self.config[idx]
            c[idx] = self.config[idx-self.n]
            c[idx-self.n] = temp
            new_state = PuzzleState(c, self.n, self, "UP", self.cost+1)
            return new_state
      
    def move_down(self):
        """
        Moves the blank tile one row down.
        :return a PuzzleState with the new configuration
        """
        c = list(self.config)
        idx = c.index(0)
        if idx < self.n * (self.n - 1):
            temp = self.config[idx]
            c[idx] = self.config[idx+self.n]
            c[idx+self.n] = temp
            new_state = PuzzleState(c, self.n, self, "DOWN", self.cost+1)
            return new_state
      
    def move_left(self):
        """
        Moves the blank tile one column to the left.
        :return a PuzzleState with the new configuration
        """
        c = list(self.config)
        idx = c.index(0)
        if idx%self.n != 0:
            temp = self.config[idx]
            c[idx] = self.config[idx-1]
            c[idx-1] = temp
            new_state = PuzzleState(c, self.n, self, "LEFT", self.cost+1)
            return new_state

    def move_right(self):
        """
        Moves the blank tile one column to the right.
        :return a PuzzleState with the new configuration
        """
        c = list(self.config)
        idx = c.index(0)
        if idx%self.n < self.n-1:
            temp = self.config[idx]
            c[idx] = self.config[idx+1]
            c[idx+1] = temp
            new_state = PuzzleState(c, self.n, self, "RIGHT", self.cost+1)
            return new_state

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
        return self.children

class MyHeap(object):
    def __init__(self, initial):
        self.cost = calculate_total_cost(initial)
        self.data = [(self.cost, -1, 0, initial)]
        self.index = 0
        heapq.heapify(self.data)

    def push(self, cost_state):
        cost, i, idx, state = cost_state
        heapq.heappush(self.data, (cost, i, self.index, state))
        self.index += 1

    def pop(self):
        return heapq.heappop(self.data)[-1]
    
    def decreasekey(self, cost, i, idx, state):
        for i, m in enumerate(self.data):
            if state.config == m[-1].config and cost <= m[0]:
                self.data[i] = (cost, i, self.index, state)
                heapq.heapify(self.data)
        self.index += 1

# Function that Writes to output.txt
def writeOutput(goal_path, cost, nodes_expanded, search_depth, max_depth, runtime):
    f = open('output.txt', 'w')
    f.write("path to goal: " + goal_path + '\n')
    f.write("cost of path: " + cost + '\n')
    f.write("nodes expanded: " + nodes_expanded + '\n')
    f.write("search depth: " + search_depth + '\n')
    f.write("max search depth: " + max_depth + '\n')
    f.write("running time: " + runtime + '\n')
    s = str(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/(8 * pow(10, 6)))
    f.write("max ram usage: " + s + '\n')
    f.close()
    

def bfs_search(initial_state):
    """BFS search"""
    start_time = time.time()
    visited = set()
    q = Q.Queue(maxsize=0)
    q.put(initial_state)
    nodes_expanded = 0
    max_depth = 0
    while q:
        state = q.get()
        visited.add(tuple(state.config))
        if test_goal(state):
            cost = search_depth = state.cost
            goal_path = [state.action]
            temp = state
            while temp.parent.action != "Initial":
                goal_path.insert(0, temp.parent.action)
                temp = temp.parent
            runtime = "{:.8f}".format(time.time()-start_time)
            return writeOutput(str(goal_path), str(cost), str(nodes_expanded), str(search_depth), str(max_depth), runtime)
        nodes_expanded += 1
        children = state.expand()
        for child in children:
            if tuple(child.config) not in visited:
                q.put(child)
                visited.add(tuple(child.config))
                if child.cost > max_depth:
                    max_depth += 1

def dfs_search(initial_state):
    """DFS search"""
    visited = set(tuple(initial_state.config))
    start_time = time.time()
    max_depth = 0
    nodes_expanded = 0
    st = [initial_state]
    heap_set = set()
    while st:
        state = st.pop()
        visited.add(tuple(state.config))
        if test_goal(state):
            cost = search_depth = state.cost
            goal_path = [state.action]
            temp = state
            while temp.parent.action != "Initial":
                goal_path.insert(0, temp.parent.action)
                temp = temp.parent
            runtime = "{:.8f}".format(time.time()-start_time)
            return writeOutput(str(goal_path), str(cost), str(nodes_expanded), str(search_depth), str(max_depth), runtime)
        children = reversed(state.expand())
        nodes_expanded += 1
        for child in children:
            if tuple(child.config) not in visited:
                st.append(child)
                visited.add(tuple(child.config))
                if child.cost > max_depth:
                    max_depth += 1

def A_star_search(initial_state):
    """A * search"""
    minheap = MyHeap(initial_state)
    visited = set()
    heap_set = set(tuple(initial_state.config))
    start_time = time.time()
    max_depth = 0
    nodes_expanded = 0
    while minheap.data:
        state = minheap.pop()
        visited.add(tuple(state.config))
        if test_goal(state):
            cost = search_depth = state.cost
            goal_path = [state.action]
            temp = state
            while temp.parent.action != "Initial":
                goal_path.insert(0, temp.parent.action)
                temp = temp.parent
            runtime = "{:.8f}".format(time.time()-start_time)
            return writeOutput(str(goal_path), str(cost), str(nodes_expanded), str(search_depth), str(max_depth), runtime)
        children = state.expand()
        nodes_expanded += 1
        for i, child in enumerate(children):
            f = calculate_total_cost(child)
            if tuple(child.config) not in set(list(visited) + list(heap_set)):
                minheap.push((f, i, minheap.index, child))
                heap_set.add(tuple(child.config))
                if child.cost > max_depth:
                    max_depth += 1
            elif tuple(child.config) in heap_set:
                minheap.decreasekey(f, i, minheap.index, child)

def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""
    sum_dists = 0
    for i in range(0, len(state.config)):
        if state.config[i] != 0:
            sum_dists += calculate_manhattan_dist(i, state.config[i], state.n)
    total_cost = state.cost + sum_dists
    return total_cost

def calculate_manhattan_dist(idx, value, n):
    """calculate the manhattan distance of a tile"""
    x = idx%n
    y = idx//n
    goal_state = [[0,1,2],[3,4,5],[6,7,8]]
    dist = 0
    for i in range(n):
        for j in range(n):
            if goal_state[i][j] == value:
                return abs(j-x) + abs(i-y)

def test_goal(puzzle_state):
    """test the state is the goal state or not"""
    if puzzle_state.config == [i for i in range(9)]:
        return puzzle_state.config

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
