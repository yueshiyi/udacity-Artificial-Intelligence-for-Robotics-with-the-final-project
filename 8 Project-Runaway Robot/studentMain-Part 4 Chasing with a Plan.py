# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot.
# But this time, your speed will be about the same as the runaway bot.
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

########################?????
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):

    F = matrix([[1., 1., 0.],
                [0., 1., 0.],
                [0., 0., 1.]])
    H = matrix([[1., 0., 0.],
                [0., 0., 1.]])
    R = matrix([[measurement_noise, 0.],
                [0., measurement_noise]])
    I = matrix([[1., 0., 0.],
                [0., 1., 0.],
                [0., 0., 1.]])

    if not OTHER:
        OTHER = {}
        OTHER["initialState"] = matrix([[0.],
                                        [0.],
                                        [0.]])
        OTHER["initialUncertainty"] = matrix([[1000., 0., 0.],
                                              [0., 1000., 0.],
                                              [0., 0., 1000.]])
        OTHER["Measuremnt"] = []
    OTHER["Measuremnt"].append(target_measurement)
    measurements = OTHER["Measuremnt"]

    heading = []
    distance = []
    turningPredict = 0
    headingPredict = 0

    if len(measurements) == 1:
        x0 = measurements[0][0]
        y0 = measurements[0][1]
        xy_estimate = (x0, y0)

    elif len(measurements) == 2:
        distance = [0]
        heading = [0]

    else:
        for i in range(len(measurements) - 1):
            Initial = (measurements[i][0], measurements[i][1])
            Next = (measurements[i + 1][0], measurements[i + 1][1])
            distance.append(distance_between(Next, Initial))
            heading.append((atan2(measurements[i + 1][1] - measurements[i][1],
                                  measurements[i + 1][0] - measurements[i][0])) % (2 * pi))

    if len(measurements) != 1:
        x = OTHER["initialState"]
        P = OTHER["initialUncertainty"]
        PreviousHeading = x.value[0][0]
        #PreviousDistance = x.value[2][0]

        turningPredict = heading[len(heading) - 1] - PreviousHeading

        if (turningPredict + (PreviousHeading // (2 * pi)) * (2 * pi) + pi) < 0:
            turningPredict = (PreviousHeading // (2 * pi)) * (2 * pi) + (2 * pi)
        elif (turningPredict + (PreviousHeading // (2 * pi)) * (2 * pi) - pi) < 0:
            turningPredict = (PreviousHeading // (2 * pi)) * (2 * pi)

        headingPredict = heading[len(heading) - 1] + turningPredict

        distancePredict = sum(distance) / len(distance)

        # measure
        z = matrix([[headingPredict],
                    [distancePredict]])
        y = z - H * x
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - K * H) * P
        # predict
        x = F * x
        P = F * P * F.transpose()

        OTHER["initialState"] = x
        OTHER["initialUncertainty"] = P

        # compute the estimated x and y
        xPredict = target_measurement[0] + x.value[2][0] * cos(x.value[0][0])
        yPredict = target_measurement[1] + x.value[2][0] * sin(x.value[0][0])
        xy_estimate = (xPredict, yPredict)

    target = xy_estimate

    distancePredict = distance_between(hunter_position, target)
    i = 1
    x = OTHER["initialState"]
    P = OTHER["initialUncertainty"]

    while (distancePredict > i * max_distance):
        i = i + 1
        # predict
        x = F * x
        P = F * P * F.transpose()
        # compute the estimated x and y
        target = (target[0] + x.value[2][0] * cos(x.value[0][0]), target[1] + x.value[2][0] * sin(x.value[0][0]))
        distancePredict = distance_between(hunter_position, target)

    if distancePredict > max_distance:
        distancePredict = max_distance
    turning = angle_trunc(get_heading(hunter_position, target) - hunter_heading)

    return turning, distancePredict, OTHER


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance  # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance  # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print("You got it right! It took you ", ctr, " steps to catch the target.")
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance,
                                                 OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER  # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target
    distance = max_distance  # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print(demo_grading(hunter, target, next_move))


def visualization_demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print("You got it right! It took you ", ctr, " steps to catch the target.")
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught

#visualization_demo_grading(hunter, target, next_move)



####################################dose not work
'''
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if not OTHER:
        #OTHER=[[], [], [], [], []]
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        #             x,   y,   dist, theta, dtheta
        x = matrix([[0.], [0.], [0.], [0.], [0.]])
        P = matrix([[1000, 0, 0, 0, 0],
                    [0, 1000, 0, 0, 0],
                    [0, 0, 1000, 0, 0],
                    [0, 0, 0, 1000, 0],
                    [0, 0, 0, 0, 1000]])

        OTHER = [measurements, hunter_positions, hunter_headings, x, P]
    else:
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, x, P = OTHER  # now I can always refer to these variables


    x_now = target_measurement[0]
    y_now = target_measurement[1]

    dt = 1  # one step
    #u = matrix([[0.], [0.], [0.], [0.], [0.]])

    H = matrix([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0]])
    R = matrix([[measurement_noise, 0],
                [0, measurement_noise]])
    I = matrix([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1]])

    # measurement update
    Z = matrix([[x_now, y_now]])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    x0 = x.value[0][0]
    y0 = x.value[1][0]
    dist0 = x.value[2][0]
    theta0 = x.value[3][0]% (2*pi)
    dtheta0 = x.value[4][0]
    #   prediction
    x = matrix([[x0 + dist0*cos(theta0+dtheta0)],
                [y0 + dist0*sin(theta0+dtheta0)],
                [dist0],
                [theta0+dtheta0],
                [dtheta0]])

    A = matrix([[1.,0.,cos(theta0+dtheta0),-dist0*sin(theta0+dtheta0),-dist0*sin(theta0+dtheta0)],
                 [0.,1.,sin(theta0+dtheta0),dist0*cos(theta0+dtheta0),dist0*cos(theta0+dtheta0)],
                 [0.,0.,1.,0.,0.],
                 [0.,0.,0.,1.,dt],
                 [0.,0.,0.,0.,1.]])
    P = A * P * A.transpose()

    target_estimate = (x.value[0][0], x.value[1][0])

    # next prediction  guess
    # measurement update function
    def measurement_update(x0, y0, x, P):
        Z = matrix([[x0, y0]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P
        return x, P
    #next a few preditions
    target_x0 = target_estimate[0]
    target_y0 = target_estimate[1]
    for i in range(20):
        x_next, P_next = measurement_update(target_x0, target_y0, x, P)
        x0 = x_next.value[0][0]
        y0 = x_next.value[1][0]
        dist0 = x_next.value[2][0]
        theta0 = x_next.value[3][0] % (2 * pi)
        dtheta0 = x_next.value[4][0]
        #   prediction
        x_next = matrix([[x0 + dist0 * cos(theta0 + dtheta0)],
                    [y0 + dist0 * sin(theta0 + dtheta0)],
                    [dist0],
                    [theta0 + dtheta0],
                    [dtheta0]])
        A = matrix([[1., 0., cos(theta0 + dtheta0), -dist0 * sin(theta0 + dtheta0), -dist0 * sin(theta0 + dtheta0)],
                    [0., 1., sin(theta0 + dtheta0), dist0 * cos(theta0 + dtheta0), dist0 * cos(theta0 + dtheta0)],
                    [0., 0., 1., 0., 0.],
                    [0., 0., 0., 1., dt],
                    [0., 0., 0., 0., 1.]])
        P_next = A * P * A.transpose()

        x = x_next
        P = P_next
        target_x0 = x.value[0][0]
        target_y0 = x.value[1][0]

    OTHER[3] = x
    OTHER[4] = P
    target_estimate = (x.value[0][0], x.value[1][0])



    heading_to_target = get_heading(hunter_position, target_estimate)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target

    dist = distance_between(hunter_position, target_estimate)
    if dist > max_distance:
        distance = max_distance  # full speed ahead!
    else:
        distance = dist

    return turning, distance, OTHER
'''
#########################################################

################################# next one prediction, not very well
'''
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if not OTHER:
        #OTHER=[[], [], [], [], []]
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        #             x,   y,   dist, theta, dtheta
        x = matrix([[0.], [0.], [0.], [0.], [0.]])
        P = matrix([[1000, 0, 0, 0, 0],
                    [0, 1000, 0, 0, 0],
                    [0, 0, 1000, 0, 0],
                    [0, 0, 0, 1000, 0],
                    [0, 0, 0, 0, 1000]])

        OTHER = [measurements, hunter_positions, hunter_headings, x, P]
    else:
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, x, P = OTHER  # now I can always refer to these variables


    x_now = target_measurement[0]
    y_now = target_measurement[1]

    dt = 1  # one step
    #u = matrix([[0.], [0.], [0.], [0.], [0.]])

    H = matrix([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0]])
    R = matrix([[measurement_noise, 0],
                [0, measurement_noise]])
    I = matrix([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1]])

    # measurement update
    Z = matrix([[x_now, y_now]])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    x0 = x.value[0][0]
    y0 = x.value[1][0]
    dist0 = x.value[2][0]
    theta0 = x.value[3][0]% (2*pi)
    dtheta0 = x.value[4][0]
    #   prediction
    x = matrix([[x0 + dist0*cos(theta0+dtheta0)],
                [y0 + dist0*sin(theta0+dtheta0)],
                [dist0],
                [theta0+dtheta0],
                [dtheta0]])

    A = matrix([[1.,0.,cos(theta0+dtheta0),-dist0*sin(theta0+dtheta0),-dist0*sin(theta0+dtheta0)],
                 [0.,1.,sin(theta0+dtheta0),dist0*cos(theta0+dtheta0),dist0*cos(theta0+dtheta0)],
                 [0.,0.,1.,0.,0.],
                 [0.,0.,0.,1.,dt],
                 [0.,0.,0.,0.,1.]])
    P = A * P * A.transpose()

    OTHER[3] = x
    OTHER[4] = P
    target_estimate = (x.value[0][0], x.value[1][0])

    heading_to_target = get_heading(hunter_position, target_estimate)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target

    dist = distance_between(hunter_position, target_estimate)
    velocity_ratio= 1
    if dist0 != 0:
        velocity_ratio = max_distance/dist0
    if dist >= max_distance and velocity_ratio <= 1:
        ###next a few preditions  i
        i = 1
        heading_to_next_a_few = heading_to_target + i * dtheta0
        heading_difference = heading_to_next_a_few - hunter_heading
        turning = heading_difference
        distance = max_distance

    elif dist >= max_distance and velocity_ratio > 1:
        distance = max_distance  # full speed ahead!
    else:
        distance = dist

    return turning, distance, OTHER
'''
############################################### still not good
'''
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if not OTHER:
        #OTHER=[[], [], [], [], []]
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        #             x,   y,   dist, theta, dtheta
        x = matrix([[0.], [0.], [0.], [0.], [0.]])
        P = matrix([[1000, 0, 0, 0, 0],
                    [0, 1000, 0, 0, 0],
                    [0, 0, 1000, 0, 0],
                    [0, 0, 0, 1000, 0],
                    [0, 0, 0, 0, 1000]])

        OTHER = [measurements, hunter_positions, hunter_headings, x, P]
    else:
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, x, P = OTHER  # now I can always refer to these variables


    x_now = target_measurement[0]
    y_now = target_measurement[1]

    dt = 1  # one step
    #u = matrix([[0.], [0.], [0.], [0.], [0.]])

    H = matrix([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0]])
    R = matrix([[measurement_noise, 0],
                [0, measurement_noise]])
    I = matrix([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1]])

    # measurement update
    Z = matrix([[x_now, y_now]])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    x0 = x.value[0][0]
    y0 = x.value[1][0]
    dist0 = x.value[2][0]
    theta0 = x.value[3][0]% (2*pi)
    dtheta0 = x.value[4][0]
    #   prediction
    x = matrix([[x0 + dist0*cos(theta0+dtheta0)],
                [y0 + dist0*sin(theta0+dtheta0)],
                [dist0],
                [theta0+dtheta0],
                [dtheta0]])

    A = matrix([[1.,0.,cos(theta0+dtheta0),-dist0*sin(theta0+dtheta0),-dist0*sin(theta0+dtheta0)],
                 [0.,1.,sin(theta0+dtheta0),dist0*cos(theta0+dtheta0),dist0*cos(theta0+dtheta0)],
                 [0.,0.,1.,0.,0.],
                 [0.,0.,0.,1.,dt],
                 [0.,0.,0.,0.,1.]])
    P = A * P * A.transpose()

    OTHER[3] = x
    OTHER[4] = P
    target_estimate = (x.value[0][0], x.value[1][0])


    target_now = target_estimate

    dists = []
    thetas = []
    dthetas = []
    first_thetas = []
    avgdist = 0
    avgdtheta = 0
    first_theta = 0
    if len(measurements) >= 2:
        for i in range(len(measurements)-1):
            dists.append(distance_between(measurements[i], measurements[i+1]))
            theta = atan2((measurements[i+1][1]-measurements[i][1]), (measurements[i+1][0]-measurements[i][0]))
            thetas.append(theta)
            if len(thetas) >= 2:
                dtheta = thetas[-1] - thetas[-2]
                dthetas.append(dtheta)
        avgdist = sum(dists)/float(len(dists))
        if len(dthetas) >= 1:
            avgdtheta = sum(dthetas)/float(len(dthetas))
            for i in range(len(thetas)):
                first_thetas.append(thetas[i] - i*avgdtheta)
            first_theta = sum(first_thetas)/float(len(first_thetas))

    theta = angle_trunc(first_theta + len(first_thetas)*avgdtheta)
    step = 1
    if dtheta0 != 0:
        while distance_between(hunter_position, target_now) > max_distance * step:
            s1 = distance_between(hunter_position, target_now)
            s2 = max_distance * step
            step += 1
            target_now = (target_now[0] + avgdist*cos(theta+avgdtheta), target_now[1] + avgdist*sin(theta+avgdtheta))
            theta = angle_trunc(theta + avgdtheta)

    heading_to_target = get_heading(hunter_position, target_now)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target

    dist = distance_between(hunter_position, target_now)
    if step > 1:
        distance = max_distance  # full speed ahead!
    elif dist > max_distance:
        distance = max_distance
    else:
        distance = dist

    return turning, distance, OTHER
'''
'''   ###############still
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if not OTHER:
        #OTHER=[[], [], [], [], []]
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        #             x,   y,   dist, theta, dtheta
        x = matrix([[0.], [0.], [0.], [0.], [0.]])
        P = matrix([[1000, 0, 0, 0, 0],
                    [0, 1000, 0, 0, 0],
                    [0, 0, 1000, 0, 0],
                    [0, 0, 0, 1000, 0],
                    [0, 0, 0, 0, 1000]])

        OTHER = [measurements, hunter_positions, hunter_headings, x, P]
    else:
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, x, P = OTHER  # now I can always refer to these variables


    x_now = target_measurement[0]
    y_now = target_measurement[1]

    dt = 1  # one step
    #u = matrix([[0.], [0.], [0.], [0.], [0.]])

    H = matrix([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0]])
    R = matrix([[0.05, 0],
                [0, 0.05]])
    I = matrix([[1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1]])

    # measurement update
    Z = matrix([[x_now, y_now]])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P

    x0 = x.value[0][0]
    y0 = x.value[1][0]
    dist0 = x.value[2][0]
    theta0 = x.value[3][0]% (2*pi)
    dtheta0 = x.value[4][0]
    #   prediction
    x = matrix([[x0 + dist0*cos(theta0+dtheta0)],
                [y0 + dist0*sin(theta0+dtheta0)],
                [dist0],
                [theta0+dtheta0],
                [dtheta0]])

    A = matrix([[1.,0.,cos(theta0+dtheta0),-dist0*sin(theta0+dtheta0),-dist0*sin(theta0+dtheta0)],
                 [0.,1.,sin(theta0+dtheta0),dist0*cos(theta0+dtheta0),dist0*cos(theta0+dtheta0)],
                 [0.,0.,1.,0.,0.],
                 [0.,0.,0.,1.,dt],
                 [0.,0.,0.,0.,1.]])
    P = A * P * A.transpose()

    OTHER[3] = x
    OTHER[4] = P
    target_estimate = (x.value[0][0], x.value[1][0])


    target_now = target_estimate

    step = 1
    if dtheta0 != 0:
        while distance_between(hunter_position, target_now) > max_distance * step:
            x0 = x.value[0][0]
            y0 = x.value[1][0]
            dist0 = x.value[2][0]
            theta0 = x.value[3][0]
            dtheta0 = x.value[4][0]
            #   prediction
            x = matrix([[x0 + dist0 * cos(theta0 + dtheta0)],
                        [y0 + dist0 * sin(theta0 + dtheta0)],
                        [dist0],
                        [theta0 + dtheta0],
                        [dtheta0]])

            A = matrix([[1., 0., cos(theta0 + dtheta0), -dist0 * sin(theta0 + dtheta0), -dist0 * sin(theta0 + dtheta0)],
                        [0., 1., sin(theta0 + dtheta0), dist0 * cos(theta0 + dtheta0), dist0 * cos(theta0 + dtheta0)],
                        [0., 0., 1., 0., 0.],
                        [0., 0., 0., 1., dt],
                        [0., 0., 0., 0., 1.]])
            P = A * P * A.transpose()
            step += 1
            target_now = (target_now[0] + x.value[2][0]*cos(x.value[3][0]), target_now[1] + x.value[2][0]*sin(x.value[3][0]))
            if step >=50:
                target_now = target_estimate


    heading_to_target = get_heading(hunter_position, target_now)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target

    dist = distance_between(hunter_position, target_now)
    if step > 1:
        distance = max_distance  # full speed ahead!
    elif dist > max_distance:
        distance = max_distance
    else:
        distance = dist
        
    return turning, distance, OTHER
'''