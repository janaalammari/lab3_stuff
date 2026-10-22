"""
ROB-UY 3303 - Lab 03: Path Following
STARTER CODE - complete every function marked STUDENT TODO.
Run the unit tests with:   python lab_3.py
The GUI (gui_03.py) imports this file and uses your controllers on the robot.

Write your code only where it says STUDENT TODO. Sections marked
PROVIDED are given to you: do not change them.

Units: meters, seconds, radians.  v = forward speed (m/s), w = rotational
speed (rad/s, + = counter-clockwise), phi_dot_l / phi_dot_r = wheel speeds (rad/s).
"""
import math
import numpy as np
# =====================================================================
# STUDENT TODO - ROBOT CONSTANTS (your Lab 2 values, same as in gui_03.py)
TICKS_PER_REV_LEFT = 376.2   # TODO: your measured ticks per revolution, LEFT wheel (encoder A)
TICKS_PER_REV_RIGHT = 391.333  # TODO: your measured ticks per revolution, RIGHT wheel (encoder B)
# =====================================================================
WHEEL_RADIUS_M =  0.0381    # wheel radius r (m)
WHEELBASE_M =  0.175      # distance between the wheels = 2L (m)
MAX_WHEEL_SPEED= 8 #placeholder value

# =====================================================================
# PART 1 - WHEEL VELOCITIES
# =====================================================================
# STUDENT TODO: get_wheel_velocities(v, w) -> (phi_dot_l, phi_dot_r)
# Use the equations you derived in Step 1.
def get_wheel_velocities(v, w):
   val1=(w*WHEELBASE_M)/WHEEL_RADIUS_M  #val1=phiR-phiL
   val2=(2*v)/WHEEL_RADIUS_M    #val2=phiL+phiR
   phi_dot_l=(val2-val1)/2
   phi_dot_r=(val2+val1)/2
   return phi_dot_l, phi_dot_r

# STUDENT TODO: 5 unit tests, each returns True if it passes.
#   1: v > 0, w = 0     2: v < 0, w = 0     3: v = 0, w > 0
#   4: v = 0, w < 0     5: v = 0, w = 0
TOL = 1e-9


def _close(a, b):
    return abs(a - b) < TOL

def wheel_velocity_unit_test_1():
    v = 1.0
    w= 0.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    return phi_l > 0 and phi_r > 0 and _close(phi_l, phi_r)

def wheel_velocity_unit_test_2():
    v = -1.0
    w = 0.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    return phi_l < 0 and phi_r < 0 and _close(phi_l, phi_r)

def wheel_velocity_unit_test_3():
    v = 0.0
    w = 1.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    return phi_l < 0 and phi_r > 0 and _close(abs(phi_l), abs(phi_r))

def wheel_velocity_unit_test_4():
    v = 0.0
    w = -1.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    return phi_l > 0 and phi_r < 0 and _close(abs(phi_l), abs(phi_r))


def wheel_velocity_unit_test_5():
    v = 0.0
    w = 0.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    return _close(phi_l, 0.0) and _close(phi_r, 0.0)

# STUDENT TODO: run the 5 tests and print how many pass out of 5.
def run_wheel_velocity_unit_tests():
    passed=0
    if wheel_velocity_unit_test_1()==True:passed+=1
    if wheel_velocity_unit_test_2()==True: passed+=1
    if wheel_velocity_unit_test_3()==True: passed+=1
    if wheel_velocity_unit_test_4()==True: passed+=1
    if wheel_velocity_unit_test_5()==True: passed+=1
    print(passed,"out of 5 tests passed")

# =====================================================================
# PROVIDED - ROBOT SIMULATOR (do not change)
# =====================================================================
DT = 0.1                    # s, time step (same as the GUI control loop)
POS_TOL = 0.03              # m, goal reached if closer than this
ANG_TOL = 0.1               # rad, and heading error smaller than this


def wrap_angle(a):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def get_robot_velocities(phi_dot_l, phi_dot_r):
    """Wheel speeds (rad/s) -> robot (v, w)."""
    L = WHEELBASE_M / 2.0
    v = WHEEL_RADIUS_M * (phi_dot_r + phi_dot_l) / 2.0
    w = WHEEL_RADIUS_M * (phi_dot_r - phi_dot_l) / (2.0 * L)
    return v, w


def robot_simulator(X_tm1, phi_dot_l, phi_dot_r, delta_T, n_substeps=10):
    """Moves the robot for delta_T seconds with constant wheel speeds.
    Returns X_t (new pose [x, y, theta]) and the list of poses visited."""
    x, y, th = X_tm1
    h = delta_T / n_substeps
    poses = []
    for _ in range(n_substeps):
        v, w = get_robot_velocities(phi_dot_l, phi_dot_r)
        x += v * h * math.cos(th + w * h / 2.0)
        y += v * h * math.sin(th + w * h / 2.0)
        th = wrap_angle(th + w * h)
        poses.append([x, y, th])
    return [x, y, th], poses


def is_at_goal(X_des, X):
    """True if X is within POS_TOL and ANG_TOL of X_des."""
    return (math.hypot(X_des[0] - X[0], X_des[1] - X[1]) < POS_TOL and
            abs(wrap_angle(X_des[2] - X[2])) < ANG_TOL)


# =====================================================================
# PART 3 - POINT TRACKING
# =====================================================================
# STUDENT TODO: point_tracking_controller(X_des, X) -> (phi_dot_l, phi_dot_r)
#   X_des = desired pose [x, y, theta], X = current pose [x, y, theta].
#   Use the controller of Lecture 05A (rho, alpha, beta, gains, backwards
#   method), then get_wheel_velocities(v, w). Add any helper functions you need.
pass  # TODO: replace this with your implementation

# if one wheel is faster, divide both by same factor so faster one equals max
def cap_wheel_speeds(phi_l, phi_r):
    m = max(abs(phi_l), abs(phi_r))
    if m > MAX_WHEEL_SPEED:
        s = MAX_WHEEL_SPEED / m
        return phi_l * s, phi_r * s
    return phi_l, phi_r

def point_tracking_controller(X_des, X):
    #defning gains
    gain_rho = 0.5
    gain_alpha = 2.5
    gain_beta = -1.5
    #restrictions
    BETA_RADIUS = 0.05  # m, final-heading term at full strength only this close
    BACK_MARGIN = 0.15  # rad, reverse only when the goal is clearly behind
    #distributing variables
    x1,y1,theta1 = X
    x2,y2,theta2 = X_des
    #finding rho and alpha
    rho = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    # if rho<-1.5, alpha&beta make no sense
    if rho < 0.015:
        w = 1.5 * wrap_angle(theta2 - theta1)
        phi_l, phi_r = get_wheel_velocities(0, w)
        return cap_wheel_speeds(phi_l, phi_r)
    alpha=-theta1+math.atan2(y2-y1,x2-x1)
    alpha=wrap_angle(alpha)
    #backwards
    if abs(alpha)>math.pi/2+BACK_MARGIN:
        alpha=-theta1+math.atan2(-(y2-y1),-(x2-x1))
        alpha = wrap_angle(alpha)
        rho=-rho
    #idk why but it works
    beta = theta2-theta1 - alpha
    beta = wrap_angle(beta)
    if abs(rho) > BETA_RADIUS:
        gain_beta = gain_beta * BETA_RADIUS / abs(rho)
    #tranformining it into v&w
    v=gain_rho*rho
    w=gain_alpha*alpha+gain_beta*beta
    phi_l, phi_r = get_wheel_velocities(v, w)

    return cap_wheel_speeds(phi_l, phi_r)



# STUDENT TODO: 6 unit tests (cases in the assignment), each returns
# (passed, visited_poses). Loop: point_tracking_controller -> robot_simulator
# until is_at_goal(X_des, X). Add a time limit so a test can fail.
pass  # TODO: replace this with your implementation


def point_tracker_unit_test_1():
   X=[0,0,0]
   X_des=[2,2,0]
   time=0
   time_limit=30
   delta_T=DT
   visited_poses=[X.copy()]
   while is_at_goal(X_des, X)==False and time<time_limit:
       phi_l, phi_r= point_tracking_controller(X_des, X)
       X,posses = robot_simulator(X, phi_l, phi_r,delta_T)
       visited_poses.extend(posses)
       time+=delta_T
   passed=is_at_goal(X_des, X)
   return passed,visited_poses

def point_tracker_unit_test_2():
    X = [0, 0, 0]
    X_des = [2, -2, 0]
    time = 0
    time_limit = 30
    delta_T = DT
    visited_poses = [X.copy()]
    while is_at_goal(X_des, X) == False and time < time_limit:
        phi_l, phi_r = point_tracking_controller(X_des, X)
        X, posses = robot_simulator(X, phi_l, phi_r, delta_T)
        visited_poses.extend(posses)
        time += delta_T
    passed = is_at_goal(X_des, X)
    return passed, visited_poses

def point_tracker_unit_test_3():
    X = [0, 0, 0]
    X_des = [-2, 2, 0]
    time = 0
    time_limit = 30
    delta_T = DT
    visited_poses = [X.copy()]
    while is_at_goal(X_des, X) == False and time < time_limit:
        phi_l, phi_r = point_tracking_controller(X_des, X)
        X, posses = robot_simulator(X, phi_l, phi_r, delta_T)
        visited_poses.extend(posses)
        time += delta_T
    passed = is_at_goal(X_des, X)
    return passed, visited_poses

def point_tracker_unit_test_4():
    X = [0, 0, 0]
    X_des = [-2, -2, 0]
    time = 0
    time_limit = 30
    delta_T = DT
    visited_poses = [X.copy()]
    while is_at_goal(X_des, X) == False and time < time_limit:
        phi_l, phi_r = point_tracking_controller(X_des, X)
        X, posses = robot_simulator(X, phi_l, phi_r, delta_T)
        visited_poses.extend(posses)
        time += delta_T
    passed = is_at_goal(X_des, X)
    return passed, visited_poses

def point_tracker_unit_test_5():
    X = [0, 0, 0]
    X_des = [2, 2, math.pi]
    time = 0
    time_limit = 30
    delta_T = DT
    visited_poses = [X.copy()]
    while is_at_goal(X_des, X) == False and time < time_limit:
        phi_l, phi_r = point_tracking_controller(X_des, X)
        X, posses = robot_simulator(X, phi_l, phi_r, delta_T)
        visited_poses.extend(posses)
        time += delta_T
    passed = is_at_goal(X_des, X)
    return passed, visited_poses

def point_tracker_unit_test_6():
    X = [0, 0, 0]
    X_des = [3, 3,math.pi]
    time = 0
    time_limit = 60
    delta_T = DT
    visited_poses = [X.copy()]
    while is_at_goal(X_des, X) == False and time < time_limit:
        phi_l, phi_r = point_tracking_controller(X_des, X)
        X, posses = robot_simulator(X, phi_l, phi_r, delta_T)
        visited_poses.extend(posses)
        time += delta_T
    passed = is_at_goal(X_des, X)
    return passed, visited_poses

# STUDENT TODO: run the 6 tests, print how many pass, plot all paths on one figure.
def run_point_tracker_unit_tests(plot=True):
    passed=0
    passed1, poses1 = point_tracker_unit_test_1()
    if passed1 == True: passed += 1
    passed2, poses2 = point_tracker_unit_test_2()
    if passed2 == True: passed += 1
    passed3, poses3 = point_tracker_unit_test_3()
    if passed3 == True:passed += 1
    passed4, poses4 = point_tracker_unit_test_4()
    if passed4 == True:passed += 1
    passed5, poses5 = point_tracker_unit_test_5()
    if passed5 == True:passed += 1
    passed6, poses6 = point_tracker_unit_test_6()
    if passed6 == True:passed += 1
    print(passed, "out of 6 tests passed")
pass

 # TODO: replace this with your implementation


# =====================================================================
# PART 4 - PATH TRACKING
# =====================================================================
# STUDENT TODO: path_tracking_controller(P, X) -> (phi_dot_l, phi_dot_r)
#   P = path, list of poses [[x, y, theta], ...], X = current pose.
#   Use the method of Lecture 06A (closest point, look-ahead distance Delta,
#   point tracking to that point). Add any helper functions you need.
pass  # TODO: replace this with your implementation

import math

#helper functions
def closest_on_path(P, X):
    x, y, theta = X
    best_dist = math.inf
    best_i = 0
    best_t = 0.0
    for i in range(len(P) - 1):
        x1, y1, _ = P[i]
        x2, y2, _ = P[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        distance_sq = dx * dx + dy * dy
        if distance_sq < 1e-12:
            t = 0.0
        else:
            t = ((x - x1) * dx + (y - y1) * dy) / distance_sq
            t = max(0.0, min(1.0, t))
        dist = math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))
        if dist < best_dist - 1e-9:
            best_dist = dist
            best_i = i
            best_t = t
    return best_i, best_t


def move_along_path(P, i, t, delta):
    remaining = delta
    while i < len(P) - 1:
        x1, y1, theta1 = P[i]
        x2, y2, theta2 = P[i + 1]
        distance = math.hypot(x2 - x1, y2 - y1)
        left = distance * (1.0 - t)
        if distance > 1e-12 and remaining <= left:
            t = t + remaining / distance
            return [x1 + t * (x2 - x1), y1 + t * (y2 - y1),
                    wrap_angle(theta1 + t * wrap_angle(theta2 - theta1))]
        remaining -= left
        i += 1
        t = 0.0
    return list(P[-1])   # past the end of the path:


def path_tracking_controller(P, X):
    delta = 0.6   # lookahead distance delta
    if len(P) == 1:
        return point_tracking_controller(P[0], X)
    i, t = closest_on_path(P, X)                 # p_c,t
    p_des = move_along_path(P, i, t, delta)      # p_des,t, delta further along
    phi_l, phi_r = point_tracking_controller(p_des, X)
    return phi_l, phi_r

# STUDENT TODO: 3 unit tests (cases in the assignment), each returns
# (passed, visited_poses). Stop when is_at_goal(P[-1], X).
pass  # TODO: replace this with your implementation


def path_tracker_unit_test_1():
    X = [0, 0, 0]
    P=[[0,0.1,0],[2,0.1,0]]
    delta_T = DT
    time_limit = 15
    time=0
    visited_poses = [X.copy()]
    x1,y1,theta1=X
    x2,y2,theta2=P[-1]
    distance=math.sqrt(math.pow(x2-x1,2)+math.pow(y2-y1,2))
    total_theta=abs(theta1-theta2)
    while (distance>=0.02 or total_theta>0.3) and time < time_limit:
        phi_l, phi_r = path_tracking_controller(P,X)
        X, posses = robot_simulator(X, phi_l, phi_r, delta_T)
        x1, y1, theta1 = X
        visited_poses.extend(posses)
        distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
        total_theta = abs(theta1 - theta2)
        time+=delta_T
    passed = is_at_goal(P[-1], X)
    return passed, visited_poses

def path_tracker_unit_test_2():
    X = [0, 0, 0]
    P = [[0, 0, 0], [1.0, 0, 0],[1.0,0,math.pi/2],[1.0,1.0,math.pi/2]]
    delta_T = DT
    time_limit = 15
    time = 0
    visited_poses = [X.copy()]
    x1, y1, theta1 = X
    x2, y2, theta2 = P[-1]
    distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
    total_theta = abs(theta1 - theta2)
    while (distance >= 0.02 or total_theta > 0.3) and time < time_limit:
        phi_l, phi_r = path_tracking_controller(P, X)
        X, posses = robot_simulator(X, phi_l, phi_r, delta_T)
        x1, y1, theta1 = X
        visited_poses.extend(posses)
        distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
        total_theta = abs(theta1 - theta2)
        time += delta_T
    passed = is_at_goal(P[-1], X)
    return passed, visited_poses


def path_tracker_unit_test_3():
    X = [0, 0, 0]
    P = [[0, 0, math.pi/4], [2.0, 2.0, math.pi/4]]
    delta_T = DT
    time_limit = 15
    time = 0
    visited_poses = [X.copy()]
    x1, y1, theta1 = X
    x2, y2, theta2 = P[-1]
    distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
    total_theta = abs(theta1 - theta2)
    while (distance >= 0.02 or total_theta > 0.3) and time < time_limit:
        phi_l, phi_r = path_tracking_controller(P, X)
        X, posses = robot_simulator(X, phi_l, phi_r, delta_T)
        x1, y1, theta1 = X
        visited_poses.extend(posses)
        distance = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))
        total_theta = abs(theta1 - theta2)
        time += delta_T
    passed = is_at_goal(P[-1], X)
    return passed, visited_poses


# STUDENT TODO: run the 3 tests, print how many pass, plot each path as a subfigure.
def run_path_tracker_unit_tests(plot=True):
    passed=0
    passed1, poses1 = path_tracker_unit_test_1()
    if passed1 == True: passed += 1
    passed2, poses2 = path_tracker_unit_test_2()
    if passed2 == True: passed += 1
    passed3, poses3 = path_tracker_unit_test_3()
    if passed3 == True: passed += 1
    print(passed, "out of 3 tests passed")


if __name__ == "__main__":
    print("=== Part 1: wheel velocities ===")
    run_wheel_velocity_unit_tests()
    print("\n=== Part 3: point tracking ===")
    run_point_tracker_unit_tests()
    try:
        run_extra_point_tests()
    except NameError:
        pass
    print("\n=== Part 4: path tracking ===")
    run_path_tracker_unit_tests()
