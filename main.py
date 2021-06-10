#!/usr/bin/env pybricks-micropython
"""
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port,Stop,Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase

left_motor = Motor(Port.A) # Initialize the motors.
right_motor = Motor(Port.D)
fork = Motor(Port.B,Direction.COUNTERCLOCKWISE, [8, 40])
fork.control.limits(speed = 50, acceleration = 50)

line_sensor = ColorSensor(Port.S3) # Initialize the color sensor.

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=109) # Initialize the drive base.

BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2 # Calculate the light threshold. Choose values based on your measurements.

DRIVE_SPEED = 100 # Set the drive speed at 100 millimeters per second.

# if turn positive turns right if negative turns left
# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.
# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.2

class Side():
    right = 0
    left = 1

class Turn():
    right = 0
    left = 1
    do_not_turn = 2

class Location:
    def __init__(self, color, id_char, id_num):
        self.color = color # Blue, Yellow, Green, Red(House)
        self.id_char = id_char # A or B
        self.id_num = id_num # 0, 1, 2, 3

class Block:
    def __init__(self, color, spawn, destination):
        self.color = color # Blue, Yellow, Green
        self.spawn = spawn
        self.destination = destination

class House:
    def __init__(self, id_num):
        self.id_num = id_num

def straight_line(side):
        # Calculate the deviation from the threshold.
    
    if(side == Side.left):
        PROPORTIONAL_GAIN = 1.2
    else:
        PROPORTIONAL_GAIN = -1.2
    
    deviation = line_sensor.reflection() - threshold

    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # You can wait for a short time or do other things in this loop.
    wait(10)

    return turn_rate

def straight_line_turn(next_turn,side,time_limit):
    turn_rate = 10
    time = 0
    test = 0
    angle = 0
    if next_turn == Turn.left:
        angle = -90
    elif next_turn == Turn.right:
        angle = 90
    elif next_turn == Turn.do_not_turn:
        angle = 0

    while True:
        time += 1
        
        if (turn_rate < 5) and (turn_rate > -5) and (time > time_limit) :
            robot.drive(DRIVE_SPEED, 0)
            turn_rate = 0
            test = 1
            if line_sensor.reflection()>70 and angle == 0:
                return
            if line_sensor.reflection() < 20:
                time = 0
                test = 0
                robot.straight(35)
                if angle == 0:
                    return
                
                robot.turn(angle)
                turn_rate = 100
                wait(10)
                return
        
        if test == 0:
            turn_rate = straight_line(side)
        
        wait(1)

blocks = []

if __name__ == '__main__':


    '''
    blocks.append(Block('yellow', Location('green', 'A', 0), Location('blue', 'B', 3)))

    robot.straight(100) # start bot 

    straight_line_turn(Turn.left, Side.left,200)
    straight_line_turn(Turn.right,Side.right,80)
    robot.straight(200)
    robot.turn(180)
    straight_line_turn(Turn.right,Side.right,100)

    straight_line_turn(Turn.left,Side.left,150)
    straight_line_turn(Turn.left,Side.left,150)
    straight_line_turn(Turn.do_not_turn,Side.left,100)
    robot.turn(175)
    straight_line_turn(Turn.right,Side.right,100)
    straight_line_turn(Turn.right,Side.right,150)
    straight_line_turn(Turn.left,Side.left,150)
    robot.straight(200)
    
    robot.turn(175)
    straight_line_turn(Turn.right, Side.right, 100)
    straight_line_turn(Turn.right, Side.right, 200)
    robot.straight(80)
    robot.turn(175)
    straight_line_turn(Turn.left, Side.left, 30)
    straight_line_turn(Turn.left, Side.left, 250)
    robot.straight(200)
    robot.turn(180)
    '''

    #straight_line_turn(Turn.left,Side.left,150)
    #straight_line_turn(Turn.left,Side.left,150)
    #straight_line_turn(Turn.left,Side.left,150)

    fork.run(-50)
    fork.run(20)
"""

#!/usr/bin/env pybricks-micropython


from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait

ev3 = EV3Brick()

elbow_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [1, 40])

elbow_motor.control.limits(speed=100, acceleration=100)
#base_motor.control.limits(speed=60, acceleration=120)

elbow_sensor = ColorSensor(Port.S3)


elbow_motor.run_time(-30, 1000)
elbow_motor.run(15)
while elbow_sensor.reflection() < 32:
    wait(10)
elbow_motor.reset_angle(0)
elbow_motor.hold()


def robot_pick(position):

    elbow_motor.run_target(60, -20)

    elbow_motor.run_target(60, 20)


def robot_release(position):

    elbow_motor.run_target(60, -40)

    elbow_motor.run_target(60, 0)


for i in range(3):
    #ev3.speaker.beep()
    wait(100)


LEFT = 160
MIDDLE = 100
RIGHT = 40

while True:
    robot_pick(LEFT)

    #robot_release(MIDDLE)

    #robot_pick(RIGHT)
    #robot_release(LEFT)

    #robot_pick(MIDDLE)
    #robot_release(RIGHT)
