import socket
import time
import sys
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import math
import numpy as np

IP_ADDRESS = '192.168.0.205'

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

positions = {}
rotations = {}
distances = []
derised_x = []
desired_y = []
real_x = []
real_y = []

orientation_error = []
xd = [-1, 1, 1, -1, -1] # x coordinates for square position
yd = [0, 0, -2, -2, 0] # y coordinates for square position


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
   
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

def run(position, robot_id, i):

    while True:
        # Distance to goal
        x_curr = positions[robot_id][0]
        y_curr = positions[robot_id][1]
        distToGoal = math.sqrt((xd[i] - x_curr)**2 + (yd[i] - y_curr)**2)

        # Compute the desired orientation
        alpha = math.atan2((yd[i] - y_curr), (xd[i] - x_curr))
        theta = math.radians(rotations[robot_id]) 

        kw = 200
        # omega = (alpha - theta) * kw
        w = kw * math.degrees(math.atan2(math.sin(alpha - theta), math.cos(alpha - theta)))
        # print("w: " + str(w))
        # proportional-controller
        k = 2000
        v =  distToGoal * k

        print(str(distToGoal) + " i: " +str(i))
        # position-controller
        u = np.array([v - w, v + w])
        u[u > 1500] = 1500
        u[u < -1500] = -1500
        # Send control input to the motors
        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])    
        s.send(command.encode('utf-8'))
        derised_x.append(xd[i])
        desired_y.append(yd[i])
        real_x.append(x_curr)
        real_y.append(y_curr)
        time.sleep(.1)
    #  distances.append(distToGoal)
        if(distToGoal < 0.45):
            break

if __name__ == "__main__":
    try:

        clientAddress = "192.168.0.46"
        optitrackServerAddress = "192.168.0.4"
        robot_id = 205

         # This will create a new NatNet client
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.rigid_body_listener = receive_rigid_body_frame

        is_running = streaming_client.run()
        # P controller for angle

        while is_running:
            if robot_id in positions:
                for i in range(len(yd)):
                    run(positions, robot_id, i)
                    
                break
        


    except KeyboardInterrupt:
        # STOP
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        print("derised x")
        print(derised_x)
        print("derised y")
        print(desired_y)
        print("real x")
        print(real_x)
        print("real x")
        print(real_y)
        sys.exit()
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    print("derised x")
    print(derised_x)
    print("derised y")
    print(desired_y)
    print("real x")
    print(real_x)
    print("real x")
    print(real_y)
    # Close the connection
    print("Connection closed!")
    s.shutdown(2)
    s.close()
    sys.exit()
