from controller import Robot, DistanceSensor, Motor
import numpy as np
# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# initialize devices
timestep = int(robot.getBasicTimeStep())

ds=[]
for i in range(8):
    ds.append(robot.getDevice('ps'+str(i)))
    ds[-1].enable(timestep)

ls=[]
for i in range(8):
    ls.append(robot.getDevice('ls'+str(i)))
    ls[-1].enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Define STATES
state = 'FOLLOW'

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    d=[]
    for dist in ds:
        d.append(dist.getValue())
        
    d = np.asarray(d)
    d=d/1000*3.24
    
    l=[]
    for lsensor in ls:
        l.append(lsensor.getValue())
        
    l = np.asarray(l)
    l=l/9000*3.14  
    
    if state=='FOLLOW':
        phil=3.14-d[0]-d[1]-d[2]+l[7]+l[6]+l[5]
        phir=3.14-d[7]-d[6]-d[5]+l[0]+l[1]+l[2]
        if(max(d)>1):
            state ='AVOID'
    #Stop using light follow mode
    elif state=='AVOID':
        phil=3.14-d[0]-d[1]-d[2]
        phir=3.14-d[7]-d[6]-d[5] 
        if(max(d)<1):
            state = 'FOLLOW'
     
    print (d)

    leftMotor.setVelocity(phil)
    rightMotor.setVelocity(phir)
    pass