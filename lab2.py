from controller import Robot, Supervisor, Compass
import math
    
# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# setup motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

# setup supervisor
robot_node = robot.getFromDef('e-Puck')
trans_field = robot_node.getField('translation')
rot_field = robot_node.getField('rotation')

box_1_node = robot.getFromDef('C1')
box_2_node = robot.getFromDef('C2')
box_3_node = robot.getFromDef('C3')
box_4_node = robot.getFromDef('C4')
box_5_node = robot.getFromDef('C5')
box_6_node = robot.getFromDef('C6')
box_7_node = robot.getFromDef('C7')
box_8_node = robot.getFromDef('C8')
box_9_node = robot.getFromDef('C9')

boxes_fields = [
    box_1_node.getField('translation'),
    box_2_node.getField('translation'),
    box_3_node.getField('translation'),
    box_4_node.getField('translation'),
    box_5_node.getField('translation'),
    box_6_node.getField('translation'),
    box_7_node.getField('translation'),
    box_8_node.getField('translation'),
    box_9_node.getField('translation')
]

box_position_hist = [box.getSFVec3f() for box in boxes_fields]

# setup speed
max_speed = 6.28

# setup devices
compass = robot.getDevice('compass')
compass.enable(timestep)
led = robot.getDevice('led0')

def log(**kwargs):
    print('='*60)
    for variable, value in kwargs.items():
        print(f'{variable} = {value}')
    print('='*60)

def calculate_threshold(property, value, threshold):
    return (property >= value - threshold) or (property <= value + threshold)

def get_robot_position():
    x, _, z = trans_field.getSFVec3f()
    return (x, -z)

def bearing_in_degrees():
    north = compass.getValues()
    rad = math.atan2(north[1], north[2])
    bearing = (rad - 1.5708) / math.pi * 180
    if bearing < 0:
        bearing += 360
    return bearing

def distance_to(objective):
    current = get_robot_position()
    dx = current[0] - objective[0]
    dz = current[1] - objective[1]
    return math.sqrt(dx**2 + dz**2)

def distance_to_time(velocity, distance):
    speed = velocity*0.0205
    return distance/speed

def get_robot_heading():
    bearing = bearing_in_degrees()
    heading = 360 - bearing + 90
    if heading > 360:
        heading -= 360
    return heading

def theta_to(objective):
    current = get_robot_position()
    return math.degrees(math.atan2(objective[1]-current[1], objective[0]-current[0]))

def turn_theta(heading, theta):
    turn = theta - heading
    if turn > 180:
        turn = -(360 - turn)
    elif turn < -180:
        turn += 360
    return turn

def rotate_heading(theta):
    if calculate_threshold(theta, 0, 0.00001):
        rotation_time = abs(theta)/((360*(3.14*0.0205))/(math.pi*0.052))
        if theta > 0:
            left_motor.setVelocity(-0.5 * max_speed)
            right_motor.setVelocity(0.5 * max_speed)
        elif theta < 0:
            left_motor.setVelocity(0.5 * max_speed)
            right_motor.setVelocity(-0.5 * max_speed)
        start_time = robot.getTime()
        while robot.getTime() - start_time <= rotation_time:
            robot.step(timestep)

def balance_heading(objective):
    heading = get_robot_heading()
    theta = theta_to(objective)
    turn = turn_theta(heading, theta)
    if turn > 1:
        return True
    return False

def target_box_moved(target_box):
    current_position = boxes_fields[target_box].getSFVec3f()
    x_diff = abs(current_position[0] - box_position_hist[target_box][0])
    z_diff = abs(current_position[2] - box_position_hist[target_box][2])
    if x_diff > 0.01 or z_diff > 0.01:
        led.set(1)
        return True
    led.set(0)
    return False


def walk(time, velocity, objective, target_box):
    left_motor.setVelocity(velocity)
    right_motor.setVelocity(velocity)
    start_time = robot.getTime()
    while robot.getTime() - start_time < time:
        if target_box_moved(target_box):
            break
        if balance_heading(objective):
            go_to(objective, target_box)
            break
        robot.step(timestep)

def go_to(objective, target_box):
    heading = get_robot_heading()
    theta = theta_to(objective)
    turn = turn_theta(heading, theta)
    rotate_heading(turn)
    distance = distance_to(objective)
    time = distance_to_time(max_speed, distance)
    walk(time, max_speed, objective, target_box)


def run():
    target_box = 0
    go_to((0.248612, -0.10), target_box)

    target_box = 3
    go_to((0.15, -0.35), target_box)
    go_to((0, -0.25), target_box)

    target_box = 4
    go_to((0, 0), target_box)

    target_box = 1
    go_to((0.25, 0), target_box)

    target_box = 5
    go_to((0, 0.25), target_box)

    target_box = 2
    go_to((0.25, 0.20), target_box)

    target_box = 8
    go_to((0.15, 0.15), target_box)
    go_to((-0.15, 0.20), target_box)
    go_to((-0.25, 0.25), target_box)

    target_box = 7
    go_to((-0.251386, 5.57208e-08), target_box)

    target_box = 6
    go_to((-0.15, 0.05), target_box)
    go_to((-0.15, -0.15), target_box)
    go_to((-0.25, -0.25), target_box)

runned = False
while robot.step(timestep) != -1: 
    if not runned: 
        run()
        runned = True
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)