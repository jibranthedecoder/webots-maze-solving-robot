from controller import Robot
import math



def angle_difference(target, current):
    """
    Calculate the shortest path difference between two angles considering wrapping.
    Returns difference in radians between -pi and pi.
    """
    diff = target - current
    # Normalize the difference to [-pi, pi]
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff

def wrap_to_pi(angle):
    """Wrap an angle to the range [-pi, pi]"""
    return angle_difference(angle, 0)

def calculate_turn_angle(left_start, right_start, left_current, right_current):
    """
    Calculate the angle turned based on encoder readings
    """
    # Calculate the distance each wheel has turned (in meters)
    left_distance = (left_current - left_start) * WHEEL_RADIUS
    right_distance = (right_current - right_start) * WHEEL_RADIUS
    
    # Calculate the turn angle using the difference in wheel distances
    # θ = (right_distance - left_distance) / wheel_base
    angle = (right_distance - left_distance) / WHEEL_BASE
    
    return angle

def pid_turn(target_angle_offset, max_speed=9.5, angle_tol = 1.0):
    #angle_tol is the accuracy in turning angle
    """Perform a turn using PID with proper angle wrapping"""
    # Get initial encoder values
    left_start = left_encoder.getValue()
    right_start = right_encoder.getValue()
    
    # Calculate current starting angle
    current_angle = calculate_turn_angle(left_start, right_start, 
                                       left_encoder.getValue(), right_encoder.getValue())
    start_angle = wrap_to_pi(current_angle)
    
    # Calculate absolute target angle (wrapped)
    target_angle = wrap_to_pi(start_angle + target_angle_offset)

    #print(f"START_ANGLE: {math.degrees(start_angle):.2f}°")
    #print(f"TARGET_ANGLE: {math.degrees(target_angle):.2f}°")
    #print(f"TURNING: {math.degrees(target_angle_offset):.2f}°")

    # PID data
    previous_error = 0
    integral_turn = 0
    dt = TIME_STEP*MSEC  # Convert to seconds

    while robot.step(TIME_STEP) != -1:
        # Calculate current angle
        current_angle = calculate_turn_angle(left_start, right_start,
                                           left_encoder.getValue(), right_encoder.getValue())
        current_angle_wrapped = wrap_to_pi(current_angle)
        
        # Calculate error using angle difference (handles wrapping)
        error = angle_difference(target_angle, current_angle_wrapped)
        
        # Check if target reached
        if abs(error) <= math.radians(angle_tol):
            break
        
        # PID calculation
        integral_turn += error * dt
        derivative = (error - previous_error) / dt
        
        # Calculate PID output
        pid_output = (Kp_ROT * error + 
                     Ki_ROT * integral_turn + 
                     Kd_ROT * derivative)
        
        # Constrain output to max speed

        speed = min(abs(pid_output), max_speed)

        # Apply motor commands based on error direction
        if error < 0:  # Need to turn clockwise
            left_motor.setVelocity(speed)
            right_motor.setVelocity(-speed)
        else:  # Need to turn counter-clockwise
            left_motor.setVelocity(-speed)
            right_motor.setVelocity(speed)
        
        previous_error = error
    
    # Stop motors
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


def PID(Kp, Ki, Kd, Ts, setpoint, measurement, max_speed = 9.5):
    #Position controller, max_speed    
    global integral, y_prev
    # PID calculations

    e = setpoint - measurement 
    P = Kp*e
    if Ki == 0: #No integral action
        wi = 0
    else:
        wi = 1    

    integral = wi*(integral + Ki*e*Ts)
    D = -Kd*(measurement - y_prev)/Ts  

    u = P + integral + D 

    # update stored data of y for next iteration 
    y_prev = measurement

    if u > max_speed:
        u = max_speed
    elif u < -max_speed:
        u = -max_speed
        
    return u

#SIMULATION SETUP
# Time step of the simulation
TIME_STEP = 20
MAX_ROT_MOTOR = 9.53 #Device limit
MAX_ROT_BASE = 9 #Used rotation base level, less than MAX_ROT_MOTOR
WHEEL_BASE = 0.095
WHEEL_RADIUS = 0.021
MSEC = 1e-3

#Position controller intialization
integral = 0
y_prev = 0


DIST_SENS_SCALE_FAC = 0.001 #Distance sensor readings are 0...1000, with this scaling 0...1
RIGHT_ANGLE_CALIB = 1.975 #rad
SLOW_SPEED = 9.0 #rad
Kp_POS, Ki_POS, Kd_POS = 1.0, 0.0 , 0.01 #PID parameters for position controller
Kp_ROT, Ki_ROT, Kd_ROT = 1.0, 0.0, 0.01 #PID parameters for rotation controller
setpoint = 880
#Classification of sensor readings

FRONT_CLOSE = 900 #Wall in front nearby
FRONT_MED_CLOSE = 600 #Wall in front at medium distance
FRONT_FAR = 300 #far ahead

LEFT_CLOSE = 800
LEFT_MED_CLOSE = 600
RIGHT_CLOSE= 800
RIGHT_MED_CLOSE = 600

OUT = 250 #very far (for any sensor)

IDLE = 0.0 #Stop the motirs

counter = 0 #Iteration counter 

# Initialize the robot
robot = Robot()

#Wheel encoders
left_encoder = robot.getDevice('motor.left.position')
right_encoder = robot.getDevice('motor.right.position')
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# Get the wheel motors
left_motor = robot.getDevice('motor.left')
right_motor = robot.getDevice('motor.right')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(IDLE)
right_motor.setVelocity(IDLE)

# Get the distance sensors
ds_left = robot.getDevice("distance sensor left")
ds_left.enable(TIME_STEP)

ds_front = robot.getDevice("distance sensor front")
ds_front.enable(TIME_STEP)

ds_right = robot.getDevice("distance sensor right")
ds_right.enable(TIME_STEP)

#Get the inertial unit
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)


state = "move_fwd_max_vel" #The initial state


#MAIN LOOP
while robot.step(TIME_STEP) != -1:

    t = counter*TIME_STEP*MSEC
    counter += 1
    
    # Read the distance sensor values
    d_L = ds_left.getValue()
    d_F = ds_front.getValue()
    d_R = ds_right.getValue()
    
    enc_left = left_encoder.getValue()
    enc_right = right_encoder.getValue()
    
    #Read the inertial unit
    # Get orientation values (roll, pitch, yaw)
    orientation = compass.getRollPitchYaw()
    
    # Yaw is the compass heading (in radians)
    yaw = orientation[2]
    
    # Convert to degrees (0-360, where 0 is North)
    heading_degrees = (yaw * 180 / math.pi) % 360
    
    
    
    print(f"Heading: {heading_degrees:.1f}°")
    
    #Finite state machine for robot actions        
    match (state):
        case "move_fwd_max_vel":
#            print(state)
            if d_R > RIGHT_MED_CLOSE:
                du = PID(Kp_POS, Ki_POS, Kd_POS, MSEC*TIME_STEP, setpoint*DIST_SENS_SCALE_FAC, d_R*DIST_SENS_SCALE_FAC)
                #print("d_R=",d_R,"SP",setpoint)
            elif d_L > LEFT_MED_CLOSE:
                du = PID(-Kp_POS, -Ki_POS, -Kd_POS, MSEC*TIME_STEP, setpoint*DIST_SENS_SCALE_FAC, d_L*DIST_SENS_SCALE_FAC)
                #print("d_L=",d_L)
            else:
                du = 0.0    
            #print("du = ",du)
            left_motor.setVelocity(MAX_ROT_BASE)
            vel = min(MAX_ROT_MOTOR,MAX_ROT_BASE - du)
            #print("Right velo=",vel)
            right_motor.setVelocity(vel)  

            if d_L < OUT and (d_F > FRONT_CLOSE):
                state = "turn_left"
            elif d_R < RIGHT_CLOSE and d_F > FRONT_CLOSE:        
                state = "turn_right"
            elif d_L < LEFT_CLOSE and d_F > FRONT_CLOSE:
                state = "turn_left"
            elif d_L > LEFT_CLOSE and d_R > RIGHT_CLOSE and d_F > FRONT_CLOSE:
                state = "dead_end"    
            elif d_L < OUT and d_F < OUT and d_R < OUT:
                state = "end"
                
        case "turn_right":
#            print(state)
            pid_turn(-RIGHT_ANGLE_CALIB)            
            integral = 0.0 #Reset PID integral
            state = 'move_fwd_max_vel'             

        case "turn_left":
#            print(state)
            pid_turn(RIGHT_ANGLE_CALIB)            
            integral = 0.0 #reset PID integral
            state = 'move_fwd_max_vel'                                            

        case "dead_end":
#            print(state)        
            if d_R > RIGHT_MED_CLOSE:
                du = PID(Kp_POS, Ki_POS, Kd_POS, MSEC*TIME_STEP, setpoint*DIST_SENS_SCALE_FAC, d_R*DIST_SENS_SCALE_FAC)
            elif d_L > LEFT_MED_CLOSE:
                du = PID(-Kp_POS, -Ki_POS, -Kd_POS, MSEC*TIME_STEP, setpoint*DIST_SENS_SCALE_FAC, d_L*DIST_SENS_SCALE_FAC)
            else:
                du = 0.0    

            integral = 0.0 #Reset PID integral
            
            left_motor.setVelocity(-MAX_ROT_BASE)
            vel = -min(MAX_ROT_MOTOR,MAX_ROT_BASE - du)
            right_motor.setVelocity(max(vel,-MAX_ROT_BASE))      

             
            if d_R < RIGHT_CLOSE and d_F < FRONT_FAR:        
                state = "turn_right"
            elif d_L < LEFT_CLOSE and d_F < FRONT_FAR:
                state = "turn_left"
                
        case "end":
        
        
            left_motor.setVelocity(IDLE)
            right_motor.setVelocity(IDLE)
            
                        

