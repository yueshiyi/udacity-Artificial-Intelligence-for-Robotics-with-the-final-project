# ----------
# User Instructions:
#
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal.
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1  # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


##########self     wrong     !!!!!

def compute_value(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------

    # make sure your function returns a grid of values as
    # demonstrated in the previous video.
    #####################################self
    value = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    value[goal[0]][goal[1]] = 0

    last_value = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    for x in range(len(grid)):
        for y in range(len(grid[0])):
           last_value[x][y] = value[x][y]
    #####判断value是否有更改，若无则停止循环（1，完成所有   2，有阻断）
    while True:

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if grid[x][y] == 1:
                    value[x][y] = 99
                elif value[x][y] != -1:
                    break
                else:
                    mini = 99
                    for i in range(len(delta)):
                         x2 = x + delta[i][0]
                         y2 = y + delta[i][1]
                         if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                              if  value[x2][y2] != -1 and value[x2][y2] < mini:
                                  value[x][y] = value[x2][y2] + cost
                                  mini = value[x2][y2]

        c = 0
        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if value[x][y] == last_value[x][y]:
                    c = c + 1
        if  c ==  len(grid)*len(grid[0]):
            for x in range(len(grid)):
                for y in range(len(grid[0])):
                    if value[x][y] == -1:
                        value[x][y] = 99  ##阻断时，把多余的缺省值-1 置为99
            break
        else :
            for x in range(len(grid)):
                for y in range(len(grid[0])):
                    last_value[x][y] = value[x][y]
    return value
value = compute_value(grid, goal, cost)
print(value)

def answer_compute_value(grid, goal, cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    value = [[99 for col in range(len(grid[0]))] for row in range(len(grid))]
    change = True
    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        change = True

                elif grid[x][y] == 0:
                    for a in range(len(delta)):
                         x2 = x + delta[a][0]
                         y2 = y + delta[a][1]

                         if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                              v2 = value[x2][y2] + cost
                              if v2 < value[x][y]:
                                  change = True
                                  value[x][y] = v2
    # make sure your function returns a grid of values as
    # demonstrated in the previous video.
    return value
value = answer_compute_value(grid, goal, cost)
print(value)

def optimum_policy(grid,goal,cost):
    # ----------------------------------------
    # modify code below
    # ----------------------------------------
    ##################policy  answer
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    change = True

    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y] = '*'
                        change = True

                elif grid[x][y] == 0:
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]

                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                            v2 = value[x2][y2] + cost

                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[a]

    ##########################################self   policy
    '''
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    policy[goal[0]][goal[1]] = '*'
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            point_value = value[x][y]
            value2 = point_value - cost
            if v2 < 0:
                break
            for i in range(len(delta)):
                x2 = x + delta[i][0]
                y2 = y + delta[i][1]
                if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                    if value[x2][y2] == value2:
                        policy[x][y] = delta_name[i]
    '''
    return policy
policy = optimum_policy(grid, goal, cost)
print(policy)
