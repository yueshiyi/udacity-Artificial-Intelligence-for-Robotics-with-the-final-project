# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

def circle_fitting(m_points):
    X1 = 0.0
    Y1 = 0.0
    X2 = 0.0
    Y2 = 0.0
    X3 = 0.0
    Y3 = 0.0
    X1Y1 = 0.0
    X1Y2 = 0.0
    X2Y1 = 0.0

    for i in range(len(m_points)):
        X1 = X1 + m_points[i][0]
        Y1 = Y1 + m_points[i][1]
        X2 = X2 + m_points[i][0]* m_points[i][0]
        Y2 = Y2 + m_points[i][1]* m_points[i][1]
        X3 = X3 + m_points[i][0] * m_points[i][0] * m_points[i][0]
        Y3 = Y3 + m_points[i][1] * m_points[i][1] * m_points[i][1]
        X1Y1 = X1Y1 + m_points[i][0] * m_points[i][1]
        X1Y2 = X1Y2 + m_points[i][0]* m_points[i][1] * m_points[i][1]
        X2Y1 = X2Y1 + m_points[i][0] * m_points[i][0] * m_points[i][1]

    N = len(m_points)
    C = N * X2 - X1 * X1
    D = N * X1Y1 - X1 * Y1
    E = N * X3 + N * X1Y2 - (X2 + Y2) * X1
    G = N * Y2 - Y1 * Y1
    H = N * X2Y1 + N * Y3 - (X2 + Y2) * Y1
    a = (H * D - E * G) / (C * G - D * D)
    b = (H * C - E * D) / (D * D - G * C)
    c = -(a * X1 + b * Y1 + X2 + Y2) / N

    A = a / (-2)
    B = b / (-2)
    R = sqrt(a * a + b * b - 4 * c) / 2
    m_fCenterX = A
    m_fCenterY = B
    m_fRadius = R
    return m_fCenterX, m_fCenterY, m_fRadius


####################################################### ???  got lucky with 999, 1000, 1000 but still dose net work, crying
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if not OTHER:
        # OTHER=[[], [], [], [], []]
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

        estimate_measurements = []

        OTHER = [measurements, hunter_positions, hunter_headings, x, P, estimate_measurements]
    else:
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, x, P, estimate_measurements = OTHER  # now I can always refer to these variables


    x_now = target_measurement[0]
    y_now = target_measurement[1]

    if len(measurements) < 20:
        OTHER[5].append(target_measurement)
    else:
        cx, cy, r = circle_fitting(measurements)
        half_line_theta = atan2(y_now-cy, x_now-cx)
        est_x_now, est_y_now = cx+r*cos(half_line_theta), cy+r*sin(half_line_theta)
        estimate_measurements.append((est_x_now, est_y_now))
        OTHER[5] = estimate_measurements
        x_now = est_x_now
        y_now = est_y_now


    dt = 1  # one step
    # u = matrix([[0.], [0.], [0.], [0.], [0.]])

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
    theta0 = x.value[3][0] % (2 * pi)
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
            target_now = (
            target_now[0] + x.value[2][0] * cos(x.value[3][0]), target_now[1] + x.value[2][0] * sin(x.value[3][0]))
            if step >= 50:
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


def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance  # 0.97 is an example. It will change.
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
measurement_noise = 2.0*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

#print(demo_grading(hunter, target, naive_next_move))

def visualization_demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
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

    est_measuredbroken_robot = turtle.Turtle()
    est_measuredbroken_robot.shape('circle')
    est_measuredbroken_robot.color('black')
    est_measuredbroken_robot.penup()
    est_measuredbroken_robot.resizemode('user')
    est_measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
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

        est_measuredbroken_robot.setheading(target_bot.heading*180/pi)
        est_measuredbroken_robot.goto(OTHER[5][-1][0]*size_multiplier, OTHER[5][-1][1]*size_multiplier-100)
        est_measuredbroken_robot.stamp()
        #End of visualization
        ctr += 1
        if ctr >= 1000:
            print("It took too many steps to catch the target.")
    return caught
#visualization_demo_grading(hunter, target, naive_next_move)
visualization_demo_grading(hunter, target, next_move)


########################### dose not work
'''
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if not OTHER:
        # OTHER=[[], [], [], [], []]
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

    cx = 0.0
    cy = 0.0
    r = 0.0
    if len(measurements) >= 20:
        cx, cy, r = circle_fitting(measurements)


    x_now = target_measurement[0]
    y_now = target_measurement[1]

    dt = 1  # one step
    # u = matrix([[0.], [0.], [0.], [0.], [0.]])

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
    theta0 = x.value[3][0] % (2 * pi)
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
            target_now = (
            target_now[0] + x.value[2][0] * cos(x.value[3][0]), target_now[1] + x.value[2][0] * sin(x.value[3][0]))
            if step >= 50:
                target_now = target_estimate

    if len(measurements) >= 30:
        pointx = sum(measurements[:][0])/float(len(measurements))
        w = r**2 - (pointx-cx)**2
        if w > 0:
            pointy = cy - sqrt(w)
            pointy = pointy + random.random()*max_distance
            target_now = (pointx, pointy)

    heading_to_target = get_heading(hunter_position, target_now)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target

    dist = distance_between(hunter_position, target_now)

    if dist > max_distance:
        distance = max_distance
    else:
        distance = dist

    return turning, distance, OTHER

'''



