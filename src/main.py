# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Evo Vegh                                                     # 
# 	Created:      7/29/2025, 7:45:23 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import random
# Import urandom for generating random numbers
brain = Brain()

# Safe Brain screen helpers (some VEX Python builds lack certain screen APIs)
def screen_clear_line(row):
    try:
        if hasattr(brain, 'screen') and hasattr(brain.screen, 'clear_line'):
            brain.screen.clear_line(row)
    except Exception:
        pass

def screen_print(text, row=None, col=None):
    try:
        if row is not None and col is not None:
            if hasattr(brain, 'screen') and hasattr(brain.screen, 'set_cursor'):
                brain.screen.set_cursor(row, col)
        if hasattr(brain, 'screen') and hasattr(brain.screen, 'print'):
            brain.screen.print(text)
        else:
            print(text)
    except Exception:
        try:
            print(text)
        except Exception:
            pass

def autonomous():
    calibrate_drivetrain()
    IntakeFirst1.set_velocity(100, PERCENT)
    IntakeSecond2.set_velocity(100, PERCENT)
    IntakeThird3.set_velocity(100, PERCENT)
    IntakeFourth4.set_velocity(100, PERCENT)
    ramp_actuator.set(True)  # Lower ramp
    IntakeFirst1.spin(FORWARD, 100, PERCENT)
    IntakeSecond2.spin(FORWARD, 100, PERCENT)
    # Set drive velocity for this upcoming autonomous drive only
    
    drive_pid(11,speed_percent=50)  # Drive forward 7.5 inches
    turn_pid(273)   # Turn right 90 degrees
    
    drive_pid(3)  # Drive forward 36 inches
    

 

def toggle_gate():

    if gateC.value():
        gateC.set(False)
    else:
        gateC.set(True)

def toggle_ramp():

    if ramp_actuator.value():
        ramp_actuator.set(False)
    else:
        ramp_actuator.set(True)

def toggle_roller():

    if RollerIntakeA.value():
        RollerIntakeA.set(False)
    else:
        RollerIntakeA.set(True)

controller_1 = Controller(PRIMARY)  # Ensure controller_1 is defined before usage
def outake_control(dir=FORWARD, spd=100):
    """ Start and Stop IntakeFirst3
    """
    global is_outake_spinning
    if not is_outake_spinning:
        outake.spin(dir, spd, VelocityUnits.PERCENT)
        is_outake_spinning = True
    else:
        outake.stop()
        is_outake_spinning = False   

controller_1.buttonLeft.pressed(toggle_gate)
controller_1.buttonRight.pressed(toggle_ramp)
controller_1.buttonY.pressed(toggle_roller)
def intake_control(dir=FORWARD, spd=100):
    """ Start and Stop IntakeFirst3
    """
    global is_intake_spinning
    if not is_intake_spinning:
        IntakeFirst1.spin(dir, spd, VelocityUnits.PERCENT)
        is_intake_spinning = True
    else:
        IntakeFirst1.stop()
        is_intake_spinning = False  

# Removed duplicate definition of score_backward_alternate



def score_backward():
    """ Start IntakeFirst3 and IntakeSecond2 in reverse
    """
    intake_control(REVERSE)
    

controller_1.buttonR2.pressed(score_backward)
def score_forward():
    """ Start IntakeFirst3 and IntakeSecond2
    """
    intake_control(FORWARD)


controller_1.buttonR1.pressed(score_forward)

def outake_forward():
    """ Start IntakeFirst3 and IntakeSecond2
    """
    outake_control(FORWARD)
def outake_backward():
    """ Start IntakeFirst3 and IntakeSecond2 in reverse
    """
    outake_control(REVERSE)

controller_1.buttonL2.pressed(outake_forward)
controller_1.buttonL1.pressed(outake_backward)


def loader_control(dir=FORWARD, spd=100):
    """ Start and Stop the loader motor """
    global is_loader_spinning
    if not is_loader_spinning:
        IntakeFourth4.spin(dir, spd, VelocityUnits.PERCENT)
        is_loader_spinning = True
    else:
        IntakeFourth4.stop()
        is_loader_spinning = False

def load_ball():
    loader_control(FORWARD)

controller_1.buttonA.pressed(load_ball)




        

def inches_to_mm(inches):
    # Convert inches to millimeters
    return inches * 25.4
    pass # Placeholder for function logic
    # convert inches to millimeters
    return inches * 25.4    



# ---------------------------------------------------------------------------- #
       





# create competition instance
def user_control():
    # Add user control code here
    brain.screen.clear_screen()
    brain.screen.print("user control code")
    calibrate_drivetrain()

comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()

# robot configuration
left_motor_a = Motor(Ports.PORT9, GearSetting.RATIO_6_1,True)
left_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)
right_motor_b = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain_inertial = Inertial(Ports.PORT12)
drivetrain = SmartDrive(left_drive_smart, right_drive_smart, drivetrain_inertial, 319.19, 320, 40, MM, 2)

IntakeFirst1 = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
IntakeSecond2 = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
IntakeThird3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
outake = MotorGroup( IntakeSecond2, IntakeThird3)
controller_1 = Controller(PRIMARY)
left_drive_smart.set_stopping(BrakeType.BRAKE)
right_drive_smart.set_stopping(BrakeType.BRAKE) 
IntakeFirst1.set_stopping(BrakeType.BRAKE)
IntakeSecond2.set_stopping(BrakeType.BRAKE)
IntakeFourth4 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
IntakeThird3.set_stopping(BrakeType.BRAKE)
IntakeFourth4.set_stopping(BrakeType.BRAKE)
IntakeFourth4.set_velocity(100, PERCENT)
# Set the velocity of the motors
IntakeFirst1.set_velocity(100, PERCENT)
IntakeSecond2.set_velocity(70, PERCENT)
IntakeThird3.set_velocity(100, PERCENT)
IntakeFourth4.set_velocity(100, PERCENT)
Inertials = Inertial(Ports.PORT21)

#pnumatics

ramp_actuator = DigitalOut(brain.three_wire_port.b)
RollerIntakeA = DigitalOut(brain.three_wire_port.a)
gateC = DigitalOut(brain.three_wire_port.c)



# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    # Calculate a seed value using battery voltage, current, and system time
    seed_value = int(brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res())
    random.seed(seed_value)  # Use the calculated seed value to initialize the random module

# Set random seed 
initializeRandomSeed()

vexcode_initial_drivetrain_calibration_completed = False
def calibrate_drivetrain():
    # Calibrate the Drivetrain Inertial
    global vexcode_initial_drivetrain_calibration_completed
    sleep(200, MSEC)
    
    while drivetrain_inertial.is_calibrating():
        sleep(25, MSEC)
    vexcode_initial_drivetrain_calibration_completed = True
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)




# Calibrate the Drivetrain
calibrate_drivetrain()

# --- Robot info display helpers -------------------------------------------------
def _safe_attr(obj, name):
    try:
        return getattr(obj, name)
    except Exception:
        return None

def get_robot_info():
    info = {}
    try:
        # SmartDrive constructor used: SmartDrive(..., wheel_travel, track_width, wheel_diameter, ...)
        # We try to read common attribute names if available.
        info[12.57] = _safe_attr(drivetrain, 'wheel_travel')
        info[14] = _safe_attr(drivetrain, 'track_width')
        info[4] = _safe_attr(drivetrain, 'wheel_diameter')
        info[2:1] = _safe_attr(drivetrain, 'external_gear_ratio')
    except Exception:
        pass

    try:
        left_ports = []
        if 'left_motor_a' in globals():
            left_ports.append(str(_safe_attr(left_motor_a, 'port') or 'P?'))
        if 'left_motor_b' in globals():
            left_ports.append(str(_safe_attr(left_motor_b, 'port') or 'P?'))
        info['left_motors'] = ','.join(left_ports) if left_ports else None
    except Exception:
        info['left_motors'] = None

    try:
        right_ports = []
        if 'right_motor_a' in globals():
            right_ports.append(str(_safe_attr(right_motor_a, 'port') or 'P?'))
        if 'right_motor_b' in globals():
            right_ports.append(str(_safe_attr(right_motor_b, 'port') or 'P?'))
        info['right_motors'] = ','.join(right_ports) if right_ports else None
    except Exception:
        info['right_motors'] = None

    # Compute circumference if diameter is available (fallback to 40 mm)
    try:
        d = info.get('wheel_diameter')
        if d is None:
            d = 40
        info['wheel_circumference'] = float(d) * 3.14159 if d is not None else None
    except Exception:
        info['wheel_circumference'] = None

    info['drivetrain_repr'] = str(drivetrain) if 'drivetrain' in globals() else None
    return info

def display_robot_info_loop():
    try:
        while True:
            try:
                info = get_robot_info()
                wd = info.get('wheel_diameter')
                circ = info.get('wheel_circumference')
                tw = info.get('track_width')
                gr = info.get('gear_ratio')
                lm = info.get('left_motors') or 'n/a'
                rm = info.get('right_motors') or 'n/a'

                s1 = 'Wdia:' + (str(int(wd)) if wd is not None else 'n/a') + 'mm'
                s1 += ' C:' + (str(int(circ)) + 'mm' if circ is not None else 'n/a')
                screen_print(s1, 6, 1)

                s2 = 'Track:' + (str(int(tw)) + 'mm' if tw is not None else 'n/a')
                s2 += ' Gear:' + (str(gr) if gr is not None else 'n/a')
                screen_print(s2, 7, 1)

                s3 = 'L:' + lm + ' R:' + rm
                screen_print(s3, 8, 1)
            except Exception:
                pass
            wait(1000, MSEC)
    except Exception:
        return

# Start robot info display thread
robot_info_thread = Thread(display_robot_info_loop)


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")



# define variables used for controlling motors based on controller inputs
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            # stop the motors if the brain is calibrating
            if drivetrain_inertial.is_calibrating():
                left_drive_smart.stop()
                right_drive_smart.stop()
                while drivetrain_inertial.is_calibrating():
                    sleep(25, MSEC)
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_side_speed = controller_1.axis3.position() + controller_1.axis1.position()
            drivetrain_right_side_speed = controller_1.axis3.position() - controller_1.axis1.position()
            
            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor
                    left_drive_smart.stop()
                    # tell the code that the left motor has been stopped
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drivetrain_l_needs_to_be_stopped_controller_1 = True
            # check if the value is inside of the deadband range
            if drivetrain_right_side_speed < 5 and drivetrain_right_side_speed > -5:
                # check if the right motor has already been stopped
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    # stop the right drive motor
                    right_drive_smart.stop()
                    # tell the code that the right motor has been stopped
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drivetrain_r_needs_to_be_stopped_controller_1 = True
            
            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_l_needs_to_be_stopped_controller_1:
                left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
                right_drive_smart.spin(FORWARD)
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True

# define variable to track the state of outake spinning
is_outake_spinning = False

# define variable to track the state of intake spinning
is_intake_spinning = False

# define variable to track the state of loader spinning
is_loader_spinning = False

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

#region Robot Configuration

myVariable = 0

def when_started1():
    global myVariable
    
    drivetrain.set_drive_velocity(100, PERCENT)

when_started1()

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def calculate(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# Drive PID: moves a set distance (mm)
def drive_pid (target_inches, timeout_ms=2000, speed_percent=None):
    """Drive to target_inches using PID. If speed_percent is provided it temporarily
    limits the drive output to that percent for this call only.
    """
    drive_pid = PID(0.5, 0.01, 0.1)
    heading_pid = PID(1.0, 0.0, 0.2)  # Heading correction
    left_drive_smart.reset_position()
    right_drive_smart.reset_position()
    initial_heading = drivetrain_inertial.rotation()
    start_time = brain.timer.time(MSEC)
    while True:
        avg_pos = ((left_drive_smart.position(RotationUnits.REV) + right_drive_smart.position(RotationUnits.REV)) / 2) * (319.19 * 3.14159 / 360)
        # Define INCH as millimeters per inch
        INCH = 25.4
        drive_power = drive_pid.calculate(target_inches * INCH, avg_pos)
        # Apply temporary speed cap if requested
        if speed_percent is not None:
            cap = max(0, min(100, int(speed_percent)))
        else:
            cap = 100
        drive_power = max(min(drive_power, cap), -cap)
        # Heading correction
        current_heading = drivetrain_inertial.rotation()
        heading_error = initial_heading - current_heading
        correction = heading_pid.calculate(0, heading_error)
        # Apply correction: add to left, subtract from right
        left_power = drive_power + correction
        right_power = drive_power - correction
        left_power = max(min(left_power, cap), -cap)
        right_power = max(min(right_power, cap), -cap)
        left_drive_smart.set_velocity(left_power, PERCENT)
        right_drive_smart.set_velocity(right_power, PERCENT)
        left_drive_smart.spin(FORWARD)
        right_drive_smart.spin(FORWARD)
        wait(20, MSEC)
        if abs(target_inches - avg_pos) < 0.2 or (brain.timer.time(MSEC) - start_time) > timeout_ms:
            break
    left_drive_smart.stop()
    right_drive_smart.stop()

# Turn PID: turns to a target heading (deg)
def turn_pid(target_deg, timeout_ms=2000):
    turn_pid = PID(1.2, 0.01, 0.2)
    start_time = brain.timer.time(MSEC)
    while True:
        current_heading = drivetrain_inertial.rotation()
        turn_power = turn_pid.calculate(target_deg, current_heading)
        turn_power = max(min(turn_power, 50), -50)
        left_drive_smart.set_velocity(turn_power, PERCENT)
        right_drive_smart.set_velocity(-turn_power, PERCENT)
        left_drive_smart.spin(FORWARD)
        right_drive_smart.spin(FORWARD)
        wait(20, MSEC)
        if abs(target_deg - current_heading) < 2 or (brain.timer.time(MSEC) - start_time) > timeout_ms:
            break
    left_drive_smart.stop()
    right_drive_smart.stop()
   
#---------------------------------------------------------------------------------------------------------------------------------------#
#---------------------------------------------------------------------------------------------------------------------------------------#

