from __future__ import print_function
from heapq import * #Hint: Use heappop and heappush

# the actions are move right, down, left, up
ACTIONS = [(0,1),(1,0),(0,-1),(-1,0)]

class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        # Initialization of algorithms goes here
        if self.type == "dfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "bfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "ucs":
            self.frontier = []
            heappush(self.frontier, (0, self.grid.start))
            self.explored = []
        elif self.type == "astar":
            self.frontier = []
            
            goal = self.grid.goal
            start = self.grid.start
            
            # manhattan distance
            f = abs(goal[0] - start[0]) + abs(goal[1] - start[1])

            heappush(self.frontier, (f, 0, self.grid.start))
            self.explored = []
            

    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            self.grid.nodes[current].color_in_path = True #This turns the color of the node to red
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    #DFS: BUGGY, fix it first
    def dfs_step(self):
        if not self.frontier: # if-statement executes if frontier is empty
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop()

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        # the children of current are located up, down, left, and right of current
        # the order we look at is right, down, left, and up. Coordinates are (row, col)
        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]

        # update coloring of current node
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        # for each child of current that isn't in the explore nor frontier set,
        # and isn't a puddle, add it to the frontier
        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    if (n not in self.explored) and (n not in self.frontier):
                        self.previous[n] = current
                        self.frontier.append(n)
                        self.grid.nodes[n].color_frontier = True

        # move current node from the frontier to the explored set
        self.explored.append(current)

    #Implement BFS here (Don't forget to implement initialization at line 23)
    def bfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop(0)

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        # update coloring of current node
        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]

        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        # for each child of current that isn't in the explore nor frontier set,
        # and isn't a puddle, add it to the frontier
        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    if (n not in self.explored) and (n not in self.frontier):
                        self.previous[n] = current
                        self.frontier.append(n)
                        self.grid.nodes[n].color_frontier = True

        # move current node from the frontier to the explored set
        self.explored.append(current)



    #Implement UCS here (Don't forget to implement initialization at line 23)
    def ucs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        curr_g, current = heappop(self.frontier)

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        
        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]

        # update coloring of current node
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        # dictionary where key is node in frontier & value is index of node in frontier
        frontier_nodes = {}
        i = 0
        for (g, node) in self.frontier:
            frontier_nodes[node] = i
            i += 1


        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                # child is not a puddle
                if not self.grid.nodes[n].puddle:
                    # compute path cost
                    g_cost = curr_g + self.grid.nodes[n].cost()

                    # child is not in explored or frontier
                    if (n not in self.explored) and (n not in frontier_nodes):
                        self.previous[n] = current
                        heappush(self.frontier, (g_cost, n))
                        self.grid.nodes[n].color_frontier = True
                    # child is in frontier
                    elif (n in frontier_nodes):
                        index_corresp_to_n = frontier_nodes[n]
                        n_g_cost = self.frontier[index_corresp_to_n][0]

                        # child is in frontier with a high path cost
                        if n_g_cost > g_cost:
                            self.previous[n] = current
                            self.frontier[index_corresp_to_n] = (g_cost, n)
                            heapify(self.frontier)

        # move current node from the frontier to the explored set
        self.explored.append(current)
    
    #Implement Astar here (Don't forget to implement initialization at line 23)
    def astar_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return

        curr_f, curr_g, current = heappop(self.frontier)

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        # store children of current in a list
        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]

        # update coloring of current node
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        # dictionary where key is node in frontier & value is index of node in frontier list
        frontier_nodes = {}
        i = 0
        for (f, g, node) in self.frontier:
            frontier_nodes[node] = i
            i += 1


        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                # child is not a puddle
                if not self.grid.nodes[n].puddle:
                    # compute total cost
                    g_cost  = curr_g + self.grid.nodes[n].cost()
                    heuristic = abs(self.grid.goal[0] - n[0]) + abs(self.grid.goal[1] - n[1])
                    f_cost =  g_cost + heuristic

                    # child is not in explored or frontier
                    if (n not in self.explored) and (n not in frontier_nodes):
                        self.previous[n] = current
                        heappush(self.frontier, (f_cost, g_cost, n))
                        self.grid.nodes[n].color_frontier = True
                    # child is in frontier
                    elif (n in frontier_nodes):
                        index_corresp_to_n = frontier_nodes[n]
                        n_f_cost = self.frontier[index_corresp_to_n][0]

                        # child is in frontier with a high path cost
                        if n_f_cost > f_cost:
                            self.previous[n] = current
                            self.frontier[index_corresp_to_n] = (f_cost, g_cost, n)
                            heapify(self.frontier)

        # move current node from the frontier to the explored set
        self.explored.append(current)
