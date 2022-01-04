import math
from collections import deque
from queue import PriorityQueue
import PySimpleGUI as sg
import numpy as np

class Point:
    def __init__(self, x, y, parent=None,   cost=0.0): # point constructor
        self.state = (x, y) # state to compare between points
        self.x = x # coordinate X
        self.y = y # coordinate y
        self.parent = parent # appending parent node
        if (parent): # cost calculation
            self.cost = parent.cost + cost
        else:
            self.cost = cost

    def __str__(self): # to print on screen
        return "({}, {})".format(self.x, self.y)

    def __repr__(self): # to print on screen
        return "({}, {})".format(self.x, self.y)

    def __lt__(self, node): # used in priroity queue
        return self.state <= node.state

    def path(self): # to construct path between target and start position
        node, path = self, []
        while node:
            path.append(node)
            node = node.parent
        return list(reversed(path))

    def isFree(self, x, y, map): # to check available positions to move
        if ((x >= 0) and (x < map.shape[0]) and (y >= 0) and (y < map.shape[1]) and (map[x][y] != 1) and (map[x][y] != -1)):
            return True
        else:
            return False

    def manhattanDistance(self, targetX, targetY): # manhattan distance used in vertical and horizontal moves
        return math.fabs(self.x - targetX + 1) + math.fabs(self.y - targetY + 1)

    def euclideanDistanc(self, targetX, targetY): # euclidean distance used in when diagonals are included with cost 1.5
        return math.sqrt((self.x - targetX + 1) ** 2 + (self.y - targetY + 1) ** 2)

    def chebyshevDistance(self, targetX, targetY): # chebyshevDistance distance used in when diagonals are included with cost 1
        return max(self.x - targetX + 1, self.y - targetY + 1)

    def expand(self, map, diagonalMoves, cost=1.0): # generate children nodes
        ls = []

        if self.isFree(self.x + 1, self.y, map):
            next_point = Point(self.x + 1, self.y, parent=self,  cost=1)
            ls.append(next_point)
        if self.isFree(self.x, self.y + 1, map):
            next_point = Point(self.x, self.y + 1, parent=self,  cost=1)
            ls.append(next_point)
        if self.isFree(self.x - 1, self.y, map):
            next_point = Point(self.x - 1, self.y, parent=self,  cost=1)
            ls.append(next_point)
        if self.isFree(self.x, self.y - 1, map):
            next_point = Point(self.x, self.y - 1, parent=self,  cost=1)
            ls.append(next_point)
        if self.isFree(self.x + 1, self.y - 1, map) and diagonalMoves == True:
            next_point = Point(self.x + 1, self.y - 1, parent=self,  cost=cost)
            ls.append(next_point)
        if self.isFree(self.x - 1, self.y - 1, map) and diagonalMoves == True:
            next_point = Point(self.x - 1, self.y - 1, parent=self,  cost=cost)
            ls.append(next_point)
        if self.isFree(self.x + 1, self.y + 1, map) and diagonalMoves == True:
            next_point = Point(self.x + 1, self.y + 1, parent=self, cost=cost)
            ls.append(next_point)
        if self.isFree(self.x - 1, self.y + 1, map) and diagonalMoves == True:
            next_point = Point(self.x - 1, self.y + 1, parent=self,  cost=cost)
            ls.append(next_point)

        map[self.x][self.y] = -1
        return ls


class GUI:
    def __init__(self, n):
        self.n = n # side size
        self.row = n # row size
        self.column = n # column size
        self.cellCount = n * n # number of cell
        self.gridSize = 550 # grid size
        self.cellSize = 550 / n # cell size
        self.windows = False # windows object holder variable
        self.poisitionPlay = False # initial position of pointer
        self.poistionTarget = False # final position of pointer
        self.map = np.array([]) # map holder variable
        self.userCount = False # user placed on map
        self.targetCount = False # target placed on map
        self.mapCreated = False # map created folder
        self.diagonalMoves = 1 # flag to allow diagonal moves
        self.expandedNodes = 0 # counter of expanded nodes
        self.mazeID = [] # mazeID to hold GUI elements id for manipulation
        self.graph = sg.Graph((self.gridSize, self.gridSize), graph_bottom_left=(0, 0), graph_top_right=(self.gridSize, self.gridSize), key='-GRAPH-', background_color='white', drag_submits=True, enable_events=True) # graph used to hold graph canvas object

    def BFSPath(self, node, map, diagonalMoves, cost): #Breadth first search
        if (map[node.x][node.y] == 5): # return node if it is target node
            return node
        self.expandedNodes = 0 # set expanded node counter to zero
        expandedList = set() # track expanded nodes
        expandedList.add(Point(node.x, node.y))
        frontier = deque([Point(node.x, node.y)])
        while frontier:
            target = frontier.popleft()
            ls = target.expand(map, diagonalMoves, cost)
            for x in ls: # checking for target and expanded nodes
                if (map[x.x][x.y] == 5):
                    return target
                self.expandedNodes += 1
                if x not in expandedList:
                    frontier.append(x)
                    expandedList.add(x)
            print('1 -> Wall || -1 -> Visited || 5 -> Target || 2 -> Start position || 0 -> Free to move') # print statistics
            print(map)
            print('Number of expanded nodes -> ', self.expandedNodes)
        return None

    def DFSPath(self, node, map, diagonalMoves, cost):
        self.expandedNodes = 0 # set expanded node counter to zero
        if (map[node.x][node.y] == 5):
            return node # return node if it is target node
        expandedList = set() # track expanded nodes
        expandedList.add(Point(node.x, node.y))
        frontier = deque([Point(node.x, node.y)])
        while frontier:
            target = frontier.pop()
            ls = target.expand(map, diagonalMoves, cost)
            for x in ls:# checking for target and expanded nodes
                if (map[x.x][x.y] == 5):
                    return target
                self.expandedNodes += 1
                if x not in expandedList:
                    frontier.append(x)
                    expandedList.add(x)

            print('1 -> Wall || -1 -> Visited || 5 -> Target || 2 -> Start position || 0 -> Free to move')
            print(map)
            print('Number of expanded nodes -> ', self.expandedNodes)
        return None

    def recurs_DFS(self, node, map, diagonalMoves, cost):
        if (map[node.x][node.y] == 5):
            return node
        else:
            for successor in node.expand(map):
                result = self.recurs_DFS(successor, map)
                if result is not None:
                    return result
        return None

    def iterative_Deepening_Search(self, node, map, diagonalMoves, cost):
        depth = 500
        for limit in range(depth):
            mapLocal = np.copy(map)
            result = self.recursive_DLS(node, mapLocal, limit, diagonalMoves, cost)
            if result != 'cutoff':
                return result

    def recursive_DLS(self, node, map, limit, diagonalMoves, cost):
        cutoff_occured = False
        if map[node.x][node.y] == 5:
            return node
        if limit == node.cost:
            return 'cutoff'
        else:
            for x in node.expand(map, diagonalMoves, 1):
                result = self.recursive_DLS(x, map, limit, diagonalMoves, cost)
                if result == 'cutoff':
                    cutoff_occured = True
                elif result is not None:
                    return result
            if cutoff_occured:
                return 'cutoff'
            return None

    def UCS(self, node, map, diagonalMoves, cost):
        self.expandedNodes = 0
        expandedList = set()
        frontier = PriorityQueue()
        expandedList.add(Point(node.x, node.y))
        frontier.put((node.cost, Point(node.x, node.y)))
        while frontier:
            point = (frontier.get())[1] # getting point from tuple
            if (map[point.x][point.y] == 5):
                return point
            ls = point.expand(map, diagonalMoves, 1.5) # generate child nodes
            for x in ls:
                self.expandedNodes += 1
                if x not in expandedList:
                    frontier.put((x.cost, x)) # adding to priority queue with cost
                    expandedList.add(x)
            print('1 -> Wall || -1 -> Visited || 5 -> Target || 2 -> Start position || 0 -> Free to move')
            print(map)
            print('Number of expanded nodes -> ', self.expandedNodes)
        return None

    def greedy(self, node, map, diagonalMoves, cost): # heuristics depends on case
        self.expandedNodes = 0
        i, j = np.where(map == 5) # finds where target is
        expandedList = set()
        frontier = PriorityQueue()
        expandedList.add(Point(node.x, node.y))
        if cost == 1.5 and diagonalMoves == True: # case where cost of diagonal movement is 1.5
            frontier.put((node.euclideanDistanc(i, j), Point(node.x, node.y)))
        if cost == 1 and diagonalMoves == True: # case where cost of diagonal movement is 1
            frontier.put((node.chebyshevDistance(i, j), Point(node.x, node.y)))
        if diagonalMoves != True: # case where diagonal moves are not allowed
            frontier.put((node.manhattanDistance(i, j), Point(node.x, node.y)))
        while frontier:
            point = (frontier.get())[1]
            if (map[point.x][point.y] == 5):
                return point
            ls = point.expand(map, diagonalMoves, cost)
            for x in ls:
                self.expandedNodes += 1
                if x not in expandedList:
                    if cost == 1.5 and diagonalMoves == True:
                        frontier.put((x.euclideanDistanc(i, j), x))
                        expandedList.add(x)
                    if cost == 1 and diagonalMoves == True:
                        frontier.put((x.chebyshevDistance(i, j), x))
                        expandedList.add(x)
                    if diagonalMoves != True:
                        frontier.put((x.manhattanDistance(i, j), x))
                        expandedList.add(x)
            print('1 -> Wall || -1 -> Visited || 5 -> Target || 2 -> Start position || 0 -> Free to move')
            print(map)
            print('Number of expanded nodes -> ', self.expandedNodes)
        return None

    def aStar(self, node, map, diagonalMoves, cost): # heuristics depends on case h + c
        self.expandedNodes = 0
        i, j = np.where(map == 5)
        expandedList = set()
        frontier = PriorityQueue()
        expandedList.add(Point(node.x, node.y))
        if cost == 1.5 and diagonalMoves == True:
            frontier.put((node.euclideanDistanc(i, j) + node.cost, Point(node.x, node.y)))
        if cost == 1 and diagonalMoves == True:
            frontier.put((node.chebyshevDistance(i, j) + node.cost, Point(node.x, node.y)))
        if diagonalMoves != True:
            frontier.put((node.manhattanDistance(i, j) + node.cost, Point(node.x, node.y)))
        while frontier:
            point = (frontier.get())[1]

            if (map[point.x][point.y] == 5):
                return point
            ls = point.expand(map, diagonalMoves, cost)
            for x in ls:
                self.expandedNodes += 1
                if x not in expandedList:
                    if cost == 1.5 and diagonalMoves == True:
                        frontier.put((x.euclideanDistanc(i, j) + x.cost, x))
                        expandedList.add(x)
                    if cost == 1 and diagonalMoves == True:
                        frontier.put((x.chebyshevDistance(i, j) + x.cost, x))
                        expandedList.add(x)
                    if diagonalMoves != True:
                        frontier.put((x.manhattanDistance(i, j) + x.cost, x))
                        expandedList.add(x)
            print('1 -> Wall || -1 -> Visited || 5 -> Target || 2 -> Start position || 0 -> Free to move')
            print(map)
            print('Number of expanded nodes -> ', self.expandedNodes)
        return None

    def drawGrid(self): # draws grids with specific interval horizontally and vertically
        cell = self.cellCount
        for x in range(cell):
            self.graph.draw_line((self.cellSize * x, 0), (self.cellSize * x, self.gridSize), color='BLACK', width=1)
            self.graph.draw_line((0, self.cellSize * x), (self.gridSize, self.cellSize * x), color='BLACK', width=1)

    def drawCell(self, x, y, color='grey'): # draws cells
        self.mazeID.append(self.graph.draw_rectangle((x, y), (x + self.cellSize, y + self.cellSize), fill_color=color, line_width=1, line_color='BLACK'))

    def placeCell(self): # maze draw
        for x in range(self.row):
            for y in range(self.column):
                self.drawCell(x * self.cellSize, y * self.cellSize)

    def converterMatrixToMaze(self, algorithm, diagonalFlag, costFlag): #convert matrix to maze to draw
        i, j = np.where(self.map == 2) #finds where current poistion is
        node1 = Point(i[0], j[0]) # creating start node
        mapTempe = np.copy(self.map) # copying original map
        locations = algorithm(node1, mapTempe, diagonalFlag, costFlag) #algorithmic calculation
        if locations == None: # result where it might be none or path
            sg.popup('Algorithm could not find path!!!') # pop-ups  result
            print('Algorithm could not find path!!!')
        else:
            print('Path Cost -> ', locations.cost) # another statistic
            locations = locations.path() #  get path
            apTempe = np.copy(self.map) # another copy for final draw
            for x in locations: # marking path from starting to end
                if (apTempe[x.x][x.y] == 2):
                    continue
                if (apTempe[x.x][x.y] == 5):
                    continue
                else:
                    apTempe[x.x][x.y] = -1
            apTempe = np.rot90(apTempe, -1) # rotate once
            apTempe = apTempe.reshape(-1, self.map.shape[0] * self.map.shape[1])[0] # and reshape to fit in canvas
            for x in range(self.map.shape[0] * self.map.shape[1]): # mark color on canvas with id
                if (apTempe[x] == -1):
                    self.graph.Widget.itemconfig(self.mazeID[x], fill='yellow')
                if (apTempe[x] == 0):
                    self.graph.Widget.itemconfig(self.mazeID[x], fill='grey')
                if (apTempe[x] == 1):
                    self.graph.Widget.itemconfig(self.mazeID[x], fill='black')

    def rendring(self): # drawing function itself
        layout11 = [[self.graph]] # left half of  window
        layout12 = [ # right half of window
            [sg.Text('Design:', font='Lucida', justification='left', size=(30, 1))],
            [sg.Checkbox("Block", key='-Block-')],
            [sg.Checkbox("Target", key='-Target-')],
            [sg.Checkbox("User", key='-User-')],
            [sg.Text('Algorithms:', font='Lucida', justification='left', size=(30, 1))],
            [sg.Radio('BFS', "Algorithms", default=True, key="-BFS-", size=(30, 1))],
            [sg.Radio('DFS', "Algorithms", default=False, key="-DFS-", size=(30, 1))],
            [sg.Radio('Uniform Cost', "Algorithms", default=False, key="-Uniform-", size=(30, 1))],
            [sg.Radio('Iterative Deepening ', "Algorithms", default=False, key="-Iterative-", size=(30, 1))],
            [sg.Radio('Greedy', "Algorithms", default=False, key="-Greedy-", size=(30, 1))],
            [sg.Radio('A*', "Algorithms", default=False, key="-A*-", size=(30, 1))],
            [sg.Text('Dioganal')], [sg.Checkbox("Dioganal", key='-Diagonal-')],
            [sg.Text('Cost')], [sg.Radio('Cost -> 1', "Cost", default=True, key="-Cost1-", )], [sg.Radio('Cost -> 1.5', "Cost", default=False, key="-Cost1.5-", )], [sg.Button('Start', size=(30, 1))],
            [sg.Button('Reset', size=(30, 1))], [sg.Button('Exit', size=(30, 1))]
        ]

        col1 = [sg.Column(layout11), sg.Column(layout12)] #combinign into one window
        self.windows = sg.Window('Maze', [col1], finalize=True, size=(750, 600)) # creating window
        # self.graph = self.windows.Element('-GRAPH-') # getting and
        self.drawGrid() # draw grids
        self.placeCell() # places cells

        while True: # main loop
            event, values = self.windows.read()
            if event in (None, 'Exit'): # if exit is pressed then break
                break

            if values['-Target-'] == True and values['-User-'] == False and values['-Block-'] == False and self.targetCount == False: # to place target position
                if self.graph.Widget.itemcget(self.graph.get_figures_at_location((values['-GRAPH-'][0], values['-GRAPH-'][1]))[0], 'fill') == 'grey':
                    self.graph.Widget.itemconfig(self.graph.get_figures_at_location((values['-GRAPH-'][0], values['-GRAPH-'][1]))[0], fill='red')
                    self.targetCount = True

            if values['-Target-'] == False and values['-User-'] == True and values['-Block-'] == False and self.userCount == False: # to place user position
                if self.graph.Widget.itemcget(self.graph.get_figures_at_location((values['-GRAPH-'][0], values['-GRAPH-'][1]))[0], 'fill') == 'grey':
                    self.graph.Widget.itemconfig(self.graph.get_figures_at_location((values['-GRAPH-'][0], values['-GRAPH-'][1]))[0], fill='green')
                    self.userCount = True

            if values['-Target-'] == False and values['-User-'] == False and values['-Block-'] == True: # to place block position
                if self.graph.Widget.itemcget(self.graph.get_figures_at_location((values['-GRAPH-'][0], values['-GRAPH-'][1]))[0], 'fill') == 'grey':
                    self.graph.Widget.itemconfig(self.graph.get_figures_at_location((values['-GRAPH-'][0], values['-GRAPH-'][1]))[0], fill='black')
                    self.mapCreated = False

            if values['-Target-'] == False and values['-User-'] == False and values['-Block-'] == False and self.userCount == True and self.targetCount == True and self.mapCreated == False:
                mapMaze = [] # to create map from canvas
                for x in self.mazeID:
                    if self.graph.Widget.itemcget(x, 'fill') == 'black':
                        mapMaze.append(1)
                    elif self.graph.Widget.itemcget(x, 'fill') == 'red':
                        mapMaze.append(5)
                    elif self.graph.Widget.itemcget(x, 'fill') == 'green':
                        mapMaze.append(2)
                    elif self.graph.Widget.itemcget(x, 'fill') == 'grey':
                        mapMaze.append(0)

                mapMaze = np.rot90((np.array(mapMaze).reshape(self.row, self.column)), 1)
                self.map = mapMaze

                self.mapCreated = True
                # algorithm implementation
            if values['-BFS-'] == True and event == 'Start' and self.mapCreated == True:
                if (values['-Cost1-'] == True):
                    self.diagonalMoves = 1
                else:
                    self.diagonalMoves = 1.5
                self.converterMatrixToMaze(self.BFSPath, values['-Diagonal-'], self.diagonalMoves)

            if values['-DFS-'] == True and event == 'Start' and self.mapCreated == True:
                if (values['-Cost1-'] == True):
                    self.diagonalMoves = 1
                else:
                    self.diagonalMoves = 1.5
                self.converterMatrixToMaze(self.DFSPath, values['-Diagonal-'], self.diagonalMoves)

            if values['-Uniform-'] == True and event == 'Start' and self.mapCreated == True:
                if (values['-Cost1-'] == True):
                    self.diagonalMoves = 1
                else:
                    self.diagonalMoves = 1.5
                self.converterMatrixToMaze(self.UCS, values['-Diagonal-'], self.diagonalMoves)

            if values['-Greedy-'] == True and event == 'Start' and self.mapCreated == True:
                if (values['-Cost1-'] == True):
                    self.diagonalMoves = 1
                else:
                    self.diagonalMoves = 1.5
                self.converterMatrixToMaze(self.greedy, values['-Diagonal-'], self.diagonalMoves)

            if values['-Iterative-'] == True and event == 'Start' and self.mapCreated == True:
                self.diagonalMoves = 1
                self.converterMatrixToMaze(self.iterative_Deepening_Search, values['-Diagonal-'], self.diagonalMoves)

            if values['-A*-'] == True and event == 'Start' and self.mapCreated == True:
                if (values['-Cost1-'] == True):
                    self.diagonalMoves = 1
                else:
                    self.diagonalMoves = 1.5
                self.converterMatrixToMaze(self.aStar, values['-Diagonal-'], self.diagonalMoves)
             # Reset button
            if (event == 'Reset'):
                self.positionPlay = False
                self.positionTarget = False
                self.map = False
                self.userCount = False
                self.targetCount = False
                self.mapCreated = False
                for x in self.mazeID:
                    self.graph.Widget.itemconfig(x, fill='grey')

        self.windows.close()

# initial menu to choose size of maze
layout1 = [
    [sg.Text('Choose the grid size: ')],
    [sg.Combo([4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], default_value=5, key="-Combo-", size=(40, 5))],
    [sg.Button('OK', size=(37, 3))]
]

win = sg.Window("Initial Windows", layout1, resizable=True)
event, values = win.read()
gui = GUI(values['-Combo-'])  # create instance of GUI class and pass maze size
win.close() # close first menu
gui.rendring() # start main application
