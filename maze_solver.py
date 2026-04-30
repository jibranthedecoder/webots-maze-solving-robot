from controller import Robot
import math


# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

def angle_difference(target, current):
    """Return the shortest signed difference between two angles in radians."""
    diff = target - current
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def wrap_to_pi(angle):
    """Wrap an angle to the range [-pi, pi]."""
    return angle_difference(angle, 0)


def calculate_turn_angle(left_start, right_start, left_current, right_current):
    """Estimate robot turn angle from wheel encoder differences."""
    left_distance = (left_current - left_start) * WHEEL_RADIUS
    right_distance = (right_current - right_start) * WHEEL_RADIUS
    return (right_distance - left_distance) / WHEEL_BASE


def pid_turn(target_angle_offset, max_speed=9.5, angle_tol=1.0):
    """Rotate the robot by a target angle offset using encoder feedback."""
    left_start = left_encoder.getValue()
    right_start = right_encoder.getValue()

    current_angle = calculate_turn_angle(
        left_start,
        right_start,
        left_encoder.getValue(),
        right_encoder.getValue(),
    )
    start_angle = wrap_to_pi(current_angle)
    target_angle = wrap_to_pi(start_angle + target_angle_offset)

    previous_error = 0
    integral_turn = 0
    dt = TIME_STEP * MSEC

    while robot.step(TIME_STEP) != -1:
        current_angle = calculate_turn_angle(
            left_start,
            right_start,
            left_encoder.getValue(),
            right_encoder.getValue(),
        )
        current_angle_wrapped = wrap_to_pi(current_angle)
        error = angle_difference(target_angle, current_angle_wrapped)

        if abs(error) <= math.radians(angle_tol):
            break

        integral_turn += error * dt
        derivative = (error - previous_error) / dt
        pid_output = Kp_ROT * error + Ki_ROT * integral_turn + Kd_ROT * derivative
        speed = min(abs(pid_output), max_speed)

        if error < 0:
            left_motor.setVelocity(speed)
            right_motor.setVelocity(-speed)
        else:
            left_motor.setVelocity(-speed)
            right_motor.setVelocity(speed)

        previous_error = error

    left_motor.setVelocity(IDLE)
    right_motor.setVelocity(IDLE)


def PID(Kp, Ki, Kd, Ts, setpoint, measurement, max_speed=9.5):
    """Position PID controller used for wall-distance correction."""
    global integral, y_prev

    error = setpoint - measurement
    proportional = Kp * error
    integral_enabled = 0 if Ki == 0 else 1
    integral = integral_enabled * (integral + Ki * error * Ts)
    derivative = -Kd * (measurement - y_prev) / Ts

    output = proportional + integral + derivative
    y_prev = measurement

    if output > max_speed:
        output = max_speed
    elif output < -max_speed:
        output = -max_speed

    return output


# -----------------------------------------------------------------------------
# Simulation setup
# -----------------------------------------------------------------------------

TIME_STEP = 20
MSEC = 1e-3

MAX_ROT_MOTOR = 9.53  # motor limit
MAX_ROT_BASE = 9.0    # used base speed, below the motor limit

WHEEL_BASE = 0.095
WHEEL_RADIUS = 0.021

# Position controller state
integral = 0
 y_prev = 0

DIST_SENS_SCALE_FAC = 0.001
RIGHT_ANGLE_CALIB = 1.975  # rad, calibrated approximately 90 degrees for this robot/world
SLOW_SPEED = 9.0

Kp_POS, Ki_POS, Kd_POS = 1.0, 0.0, 0.01
Kp_ROT, Ki_ROT, Kd_ROT = 1.0, 0.0, 0.01

setpoint = 880

# Distance sensor thresholds. The sensors return values approximately in 0...1000.
FRONT_CLOSE = 900
FRONT_MED_CLOSE = 600
FRONT_FAR = 300

LEFT_CLOSE = 800
LEFT_MED_CLOSE = 600
RIGHT_CLOSE = 800
RIGHT_MED_CLOSE = 600

OUT = 250
IDLE = 0.0

counter = 0

robot = Robot()

# Wheel encoders
left_encoder = robot.getDevice('motor.left.position')
right_encoder = robot.getDevice('motor.right.position')
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# Wheel motors
left_motor = robot.getDevice('motor.left')
right_motor = robot.getDevice('motor.right')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(IDLE)
right_motor.setVelocity(IDLE)

# Distance sensors
# The project task uses three distance sensors: left, front, and right.
ds_left = robot.getDevice('distance sensor left')
ds_front = robot.getDevice('distance sensor front')
ds_right = robot.getDevice('distance sensor right')
ds_left.enable(TIME_STEP)
ds_front.enable(TIME_STEP)
ds_right.enable(TIME_STEP)

# Heading sensor.
# In this Webots world the device named "compass" is expected to provide
# roll/pitch/yaw values. If the world uses a pure Compass device instead,
# this part must be adapted to compass.getValues().
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

state = 'move_fwd_max_vel'


# -----------------------------------------------------------------------------
# Main control loop
# -----------------------------------------------------------------------------

while robot.step(TIME_STEP) != -1:
    t = counter * TIME_STEP * MSEC
    counter += 1

    d_L = ds_left.getValue()
    d_F = ds_front.getValue()
    d_R = ds_right.getValue()

    enc_left = left_encoder.getValue()
    enc_right = right_encoder.getValue()

    # Heading is read for debugging/documentation and potential future use.
    orientation = compass.getRollPitchYaw()
    yaw = orientation[2]
    heading_degrees = (yaw * 180 / math.pi) % 360

    match state:
        case 'move_fwd_max_vel':
            # Follow the right wall when available; otherwise use the left wall.
            if d_R > RIGHT_MED_CLOSE:
                du = PID(
                    Kp_POS,
                    Ki_POS,
                    Kd_POS,
                    MSEC * TIME_STEP,
                    setpoint * DIST_SENS_SCALE_FAC,
                    d_R * DIST_SENS_SCALE_FAC,
                )
            elif d_L > LEFT_MED_CLOSE:
                du = PID(
                    -Kp_POS,
                    -Ki_POS,
                    -Kd_POS,
                    MSEC * TIME_STEP,
                    setpoint * DIST_SENS_SCALE_FAC,
                    d_L * DIST_SENS_SCALE_FAC,
                )
            else:
                du = 0.0

            left_motor.setVelocity(MAX_ROT_BASE)
            right_motor.setVelocity(min(MAX_ROT_MOTOR, MAX_ROT_BASE - du))

            if d_L < OUT and d_F > FRONT_CLOSE:
                state = 'turn_left'
            elif d_R < RIGHT_CLOSE and d_F > FRONT_CLOSE:
                state = 'turn_right'
            elif d_L < LEFT_CLOSE and d_F > FRONT_CLOSE:
                state = 'turn_left'
            elif d_L > LEFT_CLOSE and d_R > RIGHT_CLOSE and d_F > FRONT_CLOSE:
                state = 'dead_end'
            elif d_L < OUT and d_F < OUT and d_R < OUT:
                state = 'end'

        case 'turn_right':
            pid_turn(-RIGHT_ANGLE_CALIB)
            integral = 0.0
            state = 'move_fwd_max_vel'

        case 'turn_left':
            pid_turn(RIGHT_ANGLE_CALIB)
            integral = 0.0
            state = 'move_fwd_max_vel'

        case 'dead_end':
            if d_R > RIGHT_MED_CLOSE:
                du = PID(
                    Kp_POS,
                    Ki_POS,
                    Kd_POS,
                    MSEC * TIME_STEP,
                    setpoint * DIST_SENS_SCALE_FAC,
                    d_R * DIST_SENS_SCALE_FAC,
                )
            elif d_L > LEFT_MED_CLOSE:
                du = PID(
                    -Kp_POS,
                    -Ki_POS,
                    -Kd_POS,
                    MSEC * TIME_STEP,
                    setpoint * DIST_SENS_SCALE_FAC,
                    d_L * DIST_SENS_SCALE_FAC,
                )
            else:
                du = 0.0

            integral = 0.0
            left_motor.setVelocity(-MAX_ROT_BASE)
            reverse_right = -min(MAX_ROT_MOTOR, MAX_ROT_BASE - du)
            right_motor.setVelocity(max(reverse_right, -MAX_ROT_BASE))

            if d_R < RIGHT_CLOSE and d_F < FRONT_FAR:
                state = 'turn_right'
            elif d_L < LEFT_CLOSE and d_F < FRONT_FAR:
                state = 'turn_left'

        case 'end':
            left_motor.setVelocity(IDLE)
            right_motor.setVelocity(IDLE)
