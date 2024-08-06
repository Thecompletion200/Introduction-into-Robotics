from controller import Robot
import math
from matplotlib import pyplot as plt
import numpy as np
MAX_SPEED = 6.28
BLACK_THRESHOLD = 500
WHITE_THRESHOLD = 350
# Constants
WHEEL_RADIUS = 0.0201  # meters
WHEEL_DISTANCE = 0.052  # meters

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

deltaT = timestep/1000

# Grab ground sensors
gs = [robot.getDevice('gs' + str(i)) for i in range(3)]
for gsensor in gs:
    gsensor.enable(timestep)

# Grab wheels
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf')) 

#Configure Lidarw
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

#Create display
display = robot.getDevice('display')

#Add compass and GPS
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)
# Initialize odometer variables
total_distance = 0
total_rotation = 0

# Initialize coordinate variables
xw = 0  # Initial x position in world coordinates
yw = 0.028  # Initial y position in world coordinates
alpha = 1.57

#Lidar Coords
angles=np.linspace(3.1415,-3.1415,360)

# Define the coordinates in world coordinates
x_world = 0.305
y_world = 0.25

# Define the scale factor
scale_factor = 10

# Convert the world coordinates to pixels
x_pixel = int(x_world * scale_factor) + 150  # Adjust for centering on a 300x300 display
y_pixel = 300 - (int(y_world * scale_factor) + 150)  # Adjust for centering and invert y-axis

print("Pixel value for X-coordinate:", x_pixel)
print("Pixel value for Y-coordinate:", y_pixel)


def world2map(xw, yw):
    # Define the scale factor to map the world coordinates to pixels
    scale_factor = 300  # Assuming each meter corresponds to 10 pixels
    
    # Scale the world coordinates to pixel coordinates
    xw += 0.153
    yw += .25
    px = int(xw * scale_factor)  # Adjust for centering on a 300x300 display
    py = 299 -(int(yw * scale_factor))  # Invert y-axis and adjust for centering
    px = max(px, 0)
    py = max(py, 0)
    px = min(px, 299)
    py = min(py, 299)
    return px, py

map = np.zeros((300,300))
cmap = False

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Use the GPS and compass
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    alpha = np.arctan2(compass.getValues()[0],compass.getValues()[1])
    
    # Read the sensors:
    g = [gsensor.getValue() for gsensor in gs]
    #Lidar Stuff
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] = 100
    
    x_r, y_r = [], []
    x_w, y_w = [], []
                     
    w_T_r = np.array([[np.cos(alpha), -np.sin(alpha), xw],
                      [np.sin(alpha), np.cos(alpha), yw],
                      [0, 0, 1]])
    
    X_i = np.array([ranges*np.cos(angles), ranges*np.sin(angles), np.ones((360,))])
    D = w_T_r @ X_i
        
    #Draw on display ()
    px, py = world2map(xw,yw)
    print(px, py) 
    display.drawPixel(px,py)
    display.setColor(0xFF0000)
    
    if(cmap==False and yw<-0.1 and xw<0.1):
        print("Arrived")
        cmap = True
        from scipy import signal
        cspace = signal.convolve2d(map, np.ones((15,15)), mode='same')
        plt.figure(0)
        plt.imshow(cspace)
        plt.show()
        
        plt.figure(1)
        plt.imshow(cspace>0.9)
        plt.show()
        
    for d in D.transpose():
        px,py = world2map(d[0], d[1])
        map[px,py]+=0.03
        if(map[px,py] > 1):
            map[px,py]=1
        v= int(map[px,py]*255)
        color=(v*256**2+v*256+v)
        display.setColor(int(color))
        display.drawPixel(px,py)
        
    #Plot on plt    
    # plt.ion()
    # plt.plot(D[0,:], D[1,:], '.')
    # plt.pause(0.01)
    # plt.show()
    
    # Line following code
    # Drive straight
    if g[0] > BLACK_THRESHOLD and g[1] < WHITE_THRESHOLD and g[2] > BLACK_THRESHOLD:
        phildot, phirdot = MAX_SPEED, MAX_SPEED
    # Turn right
    elif g[2] < WHITE_THRESHOLD:
        phildot, phirdot = 0.3 * MAX_SPEED, -0.1 * MAX_SPEED
    # Turn left or adjust
    elif g[0] < BLACK_THRESHOLD:
        phildot, phirdot = 0.1 * MAX_SPEED, 0.3 * MAX_SPEED
    else:
        phildot, phirdot = 0.25 * MAX_SPEED, 0.25 * MAX_SPEED

    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)
    
    # Calculate displacement of the robot
    left_speed = leftMotor.getVelocity()
    right_speed = rightMotor.getVelocity()
    # We need to find rad/s
    # In this case our Delta Rotation is defined here
    delta_rotation = (((right_speed - left_speed) * WHEEL_RADIUS) / WHEEL_DISTANCE) * (deltaT)
    # In this case our Delta Distance is defined here
    delta_distance = (((left_speed + right_speed) * WHEEL_RADIUS) / 2) * (deltaT)

    # Update total distance and rotation
    total_distance += delta_distance
    total_rotation += delta_rotation
    alpha += delta_rotation

    # Update robot's real-world coordinates
    yw += math.cos(alpha) * delta_distance
    xw += math.sin(alpha) * delta_distance
    
    # Display drawing
    # Before calling world2map
    # print("xw:", xw)
    # print("yw:", yw)
    # px, py = world2map(xw,yw)
    # display.setColor(0xFF0000)
    # display.drawPixel(px,py)
    
    #lidar Coords
    #x.append(x_i)
    #y.append(y_i)

    # Print odometer information
    print("Total Distance: {:.3f} meters".format(total_distance))
    print("Total Rotation: {:.3f} radians ({:.1f} degrees)".format(total_rotation, total_rotation * 180 / math.pi))
    print("Robot Coordinates (x, y): ({:.3f}, {:.3f})".format(xw, yw))
    print("Error:", math.sqrt(xw**2+yw**2))
    
    # Check if robot has returned to (0,0)
   # Check if robot has returned to (0,0)
    if abs(xw) < 0.2 and abs(yw) < 0.2 and total_distance > 3:
        if(xw > -0.08):
            print("Robot returned to coordinates (0, 0). Stopping...")
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break  # Exit the loop
            
