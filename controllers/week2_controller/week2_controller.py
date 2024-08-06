from controller import Robot, DistanceSensor, Motor
import numpy as nph
# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
timestep = int(robot.getBasicTimeStep())

ds[]
for i in range(8):
    ds.append(robot.getDevice('ps'+str(i)))
    ls[-1].enable(TIME_STEP)

ls=[]
for i in range(8):
    ls.append(robot.getDevice('ls'+str(i)))
    ls[-1].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    d=[]
    for dist in ds:
        d.append(dist.getValue())
        
    d = np.asarray(d)
    
    l=[]
    for lsensor in ls:
        l.appened(lsensor.getValue())
         
    phil= 3.14 - d[0]
    phir= 3.14- d[7]
    
    print (d)

    leftMotor.setVelocity(phil)
    rightMotor.setVelocity(phir)
    pass