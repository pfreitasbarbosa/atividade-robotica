from controller import Robot, DistanceSensor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# setup motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)

# setup speed
max_speed = 6.28

# setup sensors
sensors = []
for i in range(8):
    sensor = f'ps{i}'
    sensors.append(robot.getDevice(sensor))
    sensors[i].enable(timestep)

# timers
start_time = robot.getTime()

while robot.step(timestep) != -1: 
    current_time = robot.getTime()
    delta_time = current_time - start_time
    
    if delta_time < 1.024:
        left_speed = 0.3 * max_speed
        right_speed = 0.5 * max_speed
    elif delta_time < 3.232:
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed
    elif delta_time < 5.44:
        left_speed = 0.5 * max_speed
        right_speed = 0.4 * max_speed
    elif delta_time < 7.424:
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed
    elif delta_time < 8.576:
        left_speed = 0.23 * max_speed
        right_speed = 0.5 * max_speed
    elif delta_time < 11.776:
        left_speed = 1 * max_speed
        right_speed = 1 * max_speed
    elif delta_time < 13.792:
        left_speed = 0.3 * max_speed
        right_speed = 0.5 * max_speed
    elif delta_time < 14.88:
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed
    elif delta_time < 16.8:
        left_speed = 0.3 * max_speed
        right_speed = 0.5 * max_speed
    elif delta_time < 20.0:
        left_speed = 1 * max_speed
        right_speed = 1 * max_speed
    elif delta_time < 20.576:
        left_speed = 0.25 * max_speed
        right_speed = 1 * max_speed
    elif delta_time < 22.496:
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed    
    elif delta_time < 23.872:
        left_speed = 0.20 * max_speed
        right_speed = 0.5 * max_speed   
    elif delta_time < 25.472:
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed   
    elif delta_time < 27.072:
        left_speed = 0.3 * max_speed
        right_speed = 0.5 * max_speed   
    elif delta_time < 28.896:
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed 
    elif delta_time < 32:
        left_speed = 0.3 * max_speed
        right_speed = 0.7 * max_speed
    elif delta_time >= 32:
        left_speed = 0
        right_speed = 0
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)