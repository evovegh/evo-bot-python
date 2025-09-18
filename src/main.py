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
import urandom

brain = Brain()

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control", 0, 0)
    while True:
        # IntakeFirst1 control
        if controller_1.buttonR1.pressing():
            IntakeFirst1.spin(FORWARD)
        elif controller_1.buttonR2.pressing():
            IntakeFirst1.spin(FORWARD)
        else:
            IntakeFirst1.stop()

        # IntakeFourth4 control
        if controller_1.buttonL1.pressing():
            IntakeFourth4.spin(FORWARD)
        elif controller_1.buttonL2.pressing():
            IntakeFourth4.spin(REVERSE)
        else:
            IntakeFourth4.stop()

        # Ramp control
        if controller_1.buttonUp.pressing():
            ramp_actuator.set(True)
        elif controller_1.buttonDown.pressing():
            ramp_actuator.set(False)
            wait(100, MSEC)  # ensure ramp is fully retracted

        # Roller intake control
        if controller_1.buttonA.pressing():
            RollerIntakeA.set(True)
        elif controller_1.buttonB.pressing():
            RollerIntakeA.set(False)
            wait(100, MSEC)  # ensure roller is fully retracted

        # IntakeSecond2 and IntakeThird3 control
        if controller_1.buttonR1.pressing():
            IntakeSecond2.spin(FORWARD)
            IntakeThird3.spin(FORWARD)
        elif controller_1.buttonR2.pressing():
            IntakeSecond2.spin(REVERSE)
            IntakeThird3.spin(REVERSE)
        else:
            IntakeSecond2.stop()
            IntakeThird3.stop()

        # Wait before repeating the process
        wait(20, MSEC)


def inches_to_mm(inches):
    # convert inches to millimeters
    return inches * 25.4    



# ---------------------------------------------------------------------------- #
       





# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()

# robot configuration
left_motor_a = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
left_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
right_motor_b = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain_inertial = Inertial(Ports.PORT12)
drivetrain = SmartDrive(left_drive_smart, right_drive_smart, drivetrain_inertial, 319.19, 320, 40, MM, 2)

IntakeFirst1 = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
IntakeSecond2 = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
IntakeThird3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
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

#pnumatics

ramp_actuator = DigitalOut(brain.three_wire_port.a)
RollerIntakeA = DigitalOut(brain.three_wire_port.b)



# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
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

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

#region Robot Configuration

myVariable = 0

def when_started1():
    global myVariable
    IntakeFirst1.set_velocity(100, PERCENT)
    IntakeSecond2.set_velocity(100, PERCENT)
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
def drive_pid (target_inches, timeout_ms=2000):
    drive_pid = PID(0.5, 0.01, 0.1)
    heading_pid = PID(1.0, 0.0, 0.2)  # Heading correction
    left_drive_smart.reset_position()
    right_drive_smart.reset_position()
    initial_heading = drivetrain_inertial.rotation()
    start_time = brain.timer.time(MSEC)
    while True:
        avg_pos = (left_drive_smart.position(MM) + right_drive_smart.position(MM)) / 2
        # Define INCH as millimeters per inch
        INCH = 25.4
        drive_power = drive_pid.calculate(target_inches * INCH, avg_pos)
        drive_power = max(min(drive_power, 100), -100)
        # Heading correction
        current_heading = drivetrain_inertial.rotation()
        heading_error = initial_heading - current_heading
        correction = heading_pid.calculate(0, heading_error)
        # Apply correction: add to left, subtract from right
        left_power = drive_power + correction
        right_power = drive_power - correction
        left_power = max(min(left_power, 100), -100)
        right_power = max(min(right_power, 100), -100)
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

# Autonomous code
autonomous()
RED = Color(255, 0, 0)
brain.screen.set_pen_color(RED)
brain.screen.clear_screen()
brain.screen.set_cursor(1, 1)
brain.screen.print("SEGMENTATION FAULT")       
calibrate_drivetrain()

