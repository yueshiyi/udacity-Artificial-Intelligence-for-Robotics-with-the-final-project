# ----------
# User Instructions:
#
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's
# optimal path to the position specified in goal;
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a
# right turn.

forward = [[-1, 0],  # go up
           [0, -1],  # go left
           [1, 0],  # go down
           [0, 1]]  # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0]  # given in the form [row,col,direction]
# direction = 0: up
#             1: left
#             2: down
#             3: right

goal = [2, 0]  # given in the form [row,col]

cost = [2, 1, 20]  # cost has 3 values, corresponding to making


# a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------

def optimum_policy2D(grid, init, goal, cost):
    value = [[[999 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[999 for row in range(len(grid[0]))] for col in range(len(grid))]]

    policy = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
              [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
              [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
              [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]
    change = True

    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for d in range(len(forward)):  ####directions 0 up  1 left  2 down  3 right
                    if goal[0] == x and goal[1] == y:
                        if value[d][x][y] > 0:
                            value[d][x][y] = 0
                            policy[d][x][y] = '*'
                            change = True

                    elif grid[x][y] == 0:
                        for a in range(len(action)): #### [-1, 0, 1] 0 right turn 1 no turn 2 left turn
                            x2 = x + forward[(d+action[a])%len(forward)][0]
                            y2 = y + forward[(d+action[a])%len(forward)][1]
                            d2 = (d+action[a])%len(forward)

                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                                v2 = value[d2][x2][y2] + cost[a]

                                if v2 < value[d][x][y]:
                                    change = True
                                    value[d][x][y] = v2
                                    policy[d][x][y] = action_name[a]
    ###################   应该一步一步选出来policy，因为方向在变，而不是选与初始方向一致的那个图
    policy2D = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    move = policy[init[2]][init[0]][init[1]]
    policy2D[init[0]][init[1]] = move
    x = init[0]
    y = init[1]
    d = init[2]
    not_reach = True
    while not_reach:
        if goal[0] == x and goal[1] == y:
           policy2D[x][y] = '*'
           not_reach = False
        elif grid[x][y] == 0:
            for a in range(len(action)):
                if action_name[a] == move:
                    x2 = x + forward[(d + action[a]) % len(forward)][0]
                    y2 = y + forward[(d + action[a]) % len(forward)][1]
                    d2 = (d + action[a]) % len(forward)
                    m2 = policy[d2][x2][y2]
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                        policy2D[x2][y2] = policy[d2][x2][y2]
                        not_reach = True
                        x = x2
                        y = y2
                        d = d2
                        move = m2
        else :
            not_reach = False
            print('can not find a way from'+str([x, y])+'to'+str([goal[0], goal[1]]))

    return policy2D

policy = optimum_policy2D(grid, init, goal, cost)
for i in range(len(policy)):
    print(policy[i])


#################################################网课答案   policy2D
def answer_optimum_policy2D(grid, init, goal, cost):
    value = [[[999 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[999 for row in range(len(grid[0]))] for col in range(len(grid))],
             [[999 for row in range(len(grid[0]))] for col in range(len(grid))]]

    policy = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
              [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
              [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
              [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]

    policy2D = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    change = True

    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                for orientation in range(4):  ####directions 0 up  1 left  2 down  3 right
                    if goal[0] == x and goal[1] == y:
                        if value[orientation][x][y] > 0:
                            value[orientation][x][y] = 0
                            policy[orientation][x][y] = '*'
                            change = True

                    elif grid[x][y] == 0:
                        for i in range(3): #### [-1, 0, 1] 0 right turn 1 no turn 2 left turn
                            o2 = (orientation + action[i])%4
                            x2 = x + forward[o2][0]
                            y2 = y + forward[o2][1]

                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                                v2 = value[o2][x2][y2] + cost[i]
                                if v2 < value[orientation][x][y]:
                                    change = True
                                    value[orientation][x][y] = v2
                                    policy[orientation][x][y] = action_name[i]

    x = init[0]
    y = init[1]
    orientation = init[2]

    policy2D[x][y] = policy[orientation][x][y]
    while policy[orientation][x][y] != '*':
        if policy[orientation][x][y] == '#':
            o2 = orientation
        if policy[orientation][x][y] == 'R':
            o2 = (orientation - 1) % 4
        if policy[orientation][x][y] == 'L':
            o2 = (orientation + 1) % 4
        x = x + forward[o2][0]
        y = y + forward[o2][1]
        orientation = o2
        policy2D[x][y] = policy[orientation][x][y]

    return policy2D

policy = answer_optimum_policy2D(grid, init, goal, cost)
for i in range(len(policy)):
    print(policy[i])

