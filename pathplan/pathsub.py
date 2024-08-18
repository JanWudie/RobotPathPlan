import heapq
import serial #for sending stuff to odometry
import time

class PriorityQueue:
    """
    Priority Queue built off of heap datatype
    """

    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority): #adds item to priority queue w/
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self): #removes item from priority queue
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        """
        If item already in priority queue with higher priority, update its priority and rebuild the heap.
        If item already in priority queue with equal or lower priority, do nothing.
        If item not in priority queue, do the same thing as self.push.
        """
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)
            
class Search():
    """
    Things to pass on in this:
    - LIDAR map(grid)
    - goal position on grid
        
    Things to get from it:
    List of actions which is a list of integers:
        'x x.x x.xxx'
        First one is either 0,1,2 (0- Forward, 1- Right, 2-Left)
        x.x is speed 0-1 (can be negative for a reverse motion)
        x.xxx is angle (only used for left and right)
    """

    def __init__(self, goal, grid, start): #goal is [int, int], grid is 2D Lidar map
        #declares attributes of search object
        self.goal = goal
        self.grid = grid
        self.start = start

    def getStart(self):
        return self.start
    

    def getSuccessors(self, position):
        #successors is list of potential positions to move to, along with the direction moved
        successors = []
        y = position[0]
        x = position[1]
        step = 2
        yhi = [[y + step, x], ["AA 99 00", 1], "y", 1.0, [y, x]]
        ylow = [[y - step, x], ["AA 99 00", -1], "y", 1.0, [y, x]]
        xhi = [[y, x + step], ["AA 99 00", 1], "x", 1.0, [y, x]]
        xlow = [[y, x - step], ["AA 99 00", -1], "x", 1.0, [y, x]]
        if y + step <= len(grid): #prevents exceeding range
            successors.append(yhi)
        if y - step >= 0:
            successors.append(ylow)
        if x + step <= len(grid[1]):
            successors.append(xhi)
        if x - step >= 0:
            successors.append(xlow)
        return successors

    
    def checkGoal(self, position):    #checks if goal state has been reached
        if position == goal:
            return True
        return False
    
    def computeTurn(self, lastAxis, actions, action): #makes the Robit TUUUUUUURRRRRRN##
        #making this very simple for now
        """
        if lastAxis == "y":
            return [1, 1, 90] #turn right if going to +x
        return [2, 1, 90]#turn left if going to +y
        """
        # trying to make it not simple, but not implementing 180° turns yet
        lastAction = actions[-1]
        #print(actions)
        
        if lastAction[1] != action[1] and lastAxis == "y": #turning from +y to -x or -y to +x
            return "CC 99 50"
        elif lastAction[1] != action[1] and lastAxis == "x": #turning +x to -y or -x to +y
            return "BB 99 50"
        elif lastAxis == "y": #turning +y to +x or -y to -x
            return "BB 99 50"
        return "CC 99 50"   #turning +x to +y or -x to -y
        
                    

    def calcDist(self): 
        #will be implemented when encoders are added
        util.raiseNotDefined()

    def calcManhattan(self, position): #manhattan position/heuristsic
        pos = position
        goal = self.goal
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    def searchA(self): #visited is 2D array of visited coordinates
        """cost is cost of current path
        ACTUAL SEARCH ALGORITHM
        returns series of directions, currently speed 1 should move 1 imaginary unit on the grid
        """
        #if self.checkGoal(self.start):
        #    return
        visited = []
        queue = PriorityQueue()
        lastAxis = "y"
        position = self.start
        node = [position, [["DD 00 00", 1]], lastAxis, 0.0, [position]] #makes node for starting position, starting movement is a stop
        queue.push(node, 0.0)    #add start node to queue
        while not queue.isEmpty():
            position, actions, lastAxis, cost, coord = queue.pop() #removes top of queue
            if position not in visited and grid[position[0]][position[1]] < 99 : #if node is visited, move onto next in queue
                visited.append(position)
                if self.checkGoal(position):
                    #print(position) #for demo purposes, prints goal node when reached
                    #print()
                    coord += [position]
                    #print(coord)#prints coordinates ordered in path
                    #actions.pop()
                    actions = actions + [["DD 00 00", 1]]
                    actions[0] = coord[2]
                    return actions
                else:
                    """n's in successors are different from nodes, only one action is passed through a successor.
                    Nodes in the queue have a list of actions leading up to their position from start 
                    """
                    for n in self.getSuccessors(position): #goes through each possible successor
                        if lastAxis != n[2]:
                            #print(n[2], lastAxis)
                            tmp = actions + [[self.computeTurn(lastAxis, actions, n[1]), n[1][1]]] + [n[1]]
                        else:
                            tmp = actions + [n[1]]
                        tmp2 = coord + [n[4]]
                        h = self.calcManhattan(n[0])
                        #print(coord[len(coord)-1][0])
                        #print(self.grid[1][2])
                        cost += 1 + self.grid[coord[len(coord)-1][0]][coord[len(coord)-1][1]] #gets weight of node, and updates path cost
                        node = (n[0], tmp, n[2], cost, tmp2) #new node w/ actions and cost
                        queue.push(node, cost + h) #pushes new node onto stack
                        #print(actions)
        return actions  #returns list of actions



goal = [13, 7] #goal is +y direction from start, same x value as start
grid = [
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 99, 99, 99, 99, 99, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]
start = [9,7]
robot = Search(goal, grid, start)
dirList = robot.searchA()
print (dirList)
print(start)
print(dirList[0])
#ser = serial.Serial('/dev/cu.usbmodem14201', 9600, timeout=0.1)
#print(ser.name)
#ser.reset_input_buffer()
time.sleep(2) #skips commands w/out this delay
while(dirList[0] != goal):
    #time.sleep(2) #skips commands w/out this delay
    print(dirList[0])
    #ser.write(dirList[1][0].encode("utf-8"))
    #time.sleep(2) #skips commands w/out this delay
    #print(ser.readline().decode('utf-8').rstrip())
    #ser.reset_input_buffer()
    robot = Search(goal, grid, dirList[0])
    dirList = robot.searchA()


"""#checking values as a list
for item in dirList[1:]:    #first value in every path isn't used
    print(item)
    ser.write(item[0].encode("utf-8"))
    time.sleep(1.65)    #needs this amount of time between commands or else it skips
    line = ser.readline().decode('utf-8').rstrip()
    
    #tried doing a while loop to wait for finished command and that just didn't fucking work 
    #while ser.in_waiting:  # Or: while ser.inWaiting():
    #    print(ser.readline().decode('utf-8').rstrip())
    
    #print(line)
    #print(item)

"""


"""
for item in dirList[1:]:    #first value in every path isn't used
    print(item)
    ser.write(item[0].encode("utf-8"))
    time.sleep(1.65)    #needs this amount of time between commands or else it skips
    line = ser.readline().decode('utf-8').rstrip()
    
    #tried doing a while loop to wait for finished command and that just didn't fucking work 
    #while ser.in_waiting:  # Or: while ser.inWaiting():
    #    print(ser.readline().decode('utf-8').rstrip())
    
    #print(line)
    #print(item)

robot = Search(goal,grid,dirList[0])

for item in dirList[1:]:    #first value in every path isn't used
    print(item)
    ser.write(item[0].encode("utf-8"))
    time.sleep(1.65)    #needs this amount of time between commands or else it skips
    line = ser.readline().decode('utf-8').rstrip()
    
    #tried doing a while loop to wait for finished command and that just didn't fucking work 
    #while ser.in_waiting:  # Or: while ser.inWaiting():
    #    print(ser.readline().decode('utf-8').rstrip())
    
    #print(line)
    #print(item)
"""
#ser.close()





