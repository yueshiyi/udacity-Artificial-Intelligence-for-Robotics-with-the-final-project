p = [0.2, 0.2, 0.2, 0.2, 0.2]
world = ['green', 'green', 'red', 'green', 'red']
measurements = ['red', 'red']
motions = [1, 0]
pHit = 0.9
pMiss = 0.1
def sense(p, Z):
    q = [ ]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit)*pMiss))
    s = sum(q)
    for i in range (len(p)):
        q[i] = q[i]/s

    return q


###非循环世界 撞墙不动，motion exact
def move(p, U):
    q= []
    for i in range(len(p)):
        if i == 0: q.append(0.0)
        elif i<len(p)-1: q.append(p[(i-U)])
        else: q.append(p[(i-U)]+p[i])
    return q

for k in range(len(measurements)):
    p = sense(p, measurements[k])
    p = move(p, motions[k])
print(p)
