# ----------
# User Instructions:
#
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space


grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']


###################################################################################################
#hahahahahahahahahaha   调通了居然   hahahahahahahahahaha
'''
def search(grid,init,goal,cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    closed = [[0 for raw in range(len(grid[0]))]for col in range(len(grid))]
    closed[init[0]][init[1]] = 1
    x = init[0]
    y = init[1]
    g = 0
    open = [[g,x,y]]
    open_goal=[0,0,0]
    fail = False
    while not fail  and (open_goal[1] != goal[0] or open_goal[2] != goal[1]):
        index1 = 0
        open.sort()
        for i in range(len(open)):
            index2 = 0
            for j in range(len(delta)):
                new_x = open[i][1]+delta[j][0]
                if new_x < 0 or new_x >= len(grid):
                    index2 += 1
                    continue
                new_y = open[i][2]+delta[j][1]
                if new_y < 0 or new_y >= len(grid[0]):
                    index2 += 1
                    continue
                if closed[new_x][new_y] == 1:
                    index2 += 1
                    continue
                new_g = open[i][0]+cost
                if grid[new_x][new_y] != 1:
                    open.append([new_g,new_x,new_y])
                    closed[new_x][new_y] = 1
                    if new_x == goal[0] and new_y == goal[1]:
                        open_goal[0] = new_g
                        open_goal[1] = new_x
                        open_goal[2] = new_y
                else:
                    index2 += 1
                    continue
            if index2 == len(delta):
                index1 += 1
        if index1 == len(open):
            fail = True
    if  fail:
        path = 'fail'
    else:
        path = open_goal
    return path

path = search(grid, init, goal, cost)
print(path)
'''
##################################################################################################
# Modify the the search function so that it returns
# a shortest path as follows:
#
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left,
# up, and down motions. Note that the 'v' should be
# lowercase. '*' should mark the goal cell.
####网课答案######################################################################################
#########aaaaaaaaaaa令人绝望的简洁度！！！！！！！！！！！！！！！！！！！！！！！aaaaaaaaa
def answer_search(grid,init,goal,cost):

    closed = [[0 for raw in range(len(grid[0]))]for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for raw in range(len(grid[0]))]for col in range(len(grid))]
    expand[init[0]][init[1]] = 0
    ####################寻路  answer
    action = [[-1 for raw in range(len(grid[0]))] for col in range(len(grid))]

    '''################寻路  自己的
    path_long = [[-1 for raw in range(len(grid[0]))]for col in range(len(grid))]
    path_long[init[0]][init[1]] = 0
    path = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    '''
    x = init[0]
    y = init[1]
    g = 0

    open = [[g,x,y]]

    found = False    #flag that is set when search complete
    resign = False   #flag set if we can't find expand
    count = 0

    while found is False and resign is False:
        #check if we still have elements on the  open list
        if len(open) == 0:
            resign = True
            print('fail')
        else:
            #remove node from list
            open.sort()
            open.reverse()
            next = open.pop()  #pop 从最后取数，取出 g 小的node  从g最小的元素开始展开
            x = next[1]
            y = next[2]
            g = next[0]
            expand[x][y] = count
            count += 1
            #check if we are done
            if x == goal[0] and y == goal[1]:
                found = True
                print(next)

            else:
                #expend winning element and add to new open list
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >=0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open.append([g2,x2,y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i   #################！！绝了！！！！！！！！！！！！！

                            #path_long[x2][y2] = g2     ############寻路  自己的
    '''##############寻路  自己的
    if found == True:
        distance = path_long[goal[0]][goal[1]]
        path[goal[0]][goal[1]] = '*'
        step_x = goal[0]
        step_y = goal[1]
        while distance != 0:
            distance -= 1
            find_last_step = False
            for i in range(len(delta)):
                last_step_x = step_x + delta[i][0]
                last_step_y = step_y + delta[i][1]
                if last_step_x >= 0 and last_step_x < len(grid) and last_step_y >= 0 and last_step_y < len(grid[0]) and grid[last_step_x][last_step_y] != 1:
                    if path_long[last_step_x][last_step_y] == distance:
                        path[last_step_x][last_step_y] = delta_name[(i+2)%len(delta)]
                        # up left down right  ['^', '<', 'v', '>']  0 2 换  1 3 换
                        find_last_step = True
                        break
            if find_last_step :
                step_x = last_step_x
                step_y = last_step_y
        if step_x == init[0] and step_y == init[1]:
            print(path)
    '''

    #print(action)
    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    x = goal[0]
    y = goal[1]
    policy[x][y] = '*'
    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2
    for i in range(len(policy)):
        print(policy[i])

    return expand

answer_search(grid, init, goal, cost)
#print(expand_times)
#################################################################################################
'''
# -----------
# User Instructions:
#
# Modify the the search function so that it becomes
# an A* search algorithm as defined in the previous
# lectures.
#
# Your function should return the expanded grid
# which shows, for each element, the count when
# it was expanded or -1 if the element was never expanded.
# 
# If there is no path from init to goal,
# the function should return the string 'fail'
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]
             
heuristic_old = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost,heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]
    f = g + h

    open = [[f, g, h, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[3]
            y = next[4]
            g = next[1]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            h2 = heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1

    return expand


'''

