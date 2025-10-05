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
import time
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
            # fallback to console
            print(text)
    except Exception:
        try:
            print(text)
        except Exception:
            pass

# Autonomous speed limits (percent)
AUTON_DRIVE_MAX = 90
AUTON_TURN_MAX = 80

def set_auton_speed(drive_percent, turn_percent):
    """Update autonomous speed limits at runtime (clamped to -100..100)."""
    global AUTON_DRIVE_MAX, AUTON_TURN_MAX
    AUTON_DRIVE_MAX = max(min(int(drive_percent), 100), -100)
    AUTON_TURN_MAX = max(min(int(turn_percent), 100), -100)

# Teleop max drive speed (percent). This scales controller joystick outputs.
MAX_DRIVE_SPEED = 100

def set_max_drive_speed(percent):
    """Set maximum teleop drive speed (0-100)."""
    global MAX_DRIVE_SPEED
    MAX_DRIVE_SPEED = max(0, min(100, int(percent)))

# Preset speeds and runtime controls
PRESET_DRIVE_SPEEDS = [100, 75, 50, 25]
current_speed_index = 0

SETTINGS_FILE = "max_drive_speed.txt"

def save_max_speed():
    """Save current MAX_DRIVE_SPEED to a small file (best-effort)."""
    try:
        with open(SETTINGS_FILE, "w") as f:
            f.write(str(MAX_DRIVE_SPEED))
    except Exception:
        # filesystem may not be available on target; ignore errors
        pass

def load_max_speed():
    """Load MAX_DRIVE_SPEED from settings file if present (best-effort)."""
    try:
        with open(SETTINGS_FILE, "r") as f:
            v = int(f.read().strip())
            set_max_drive_speed(v)
    except Exception:
        # ignore read/parse errors and leave default
        pass

def show_max_speed(timeout_ms=1000):
    """Display the current MAX_DRIVE_SPEED on the Brain screen briefly."""
    try:
        # Save cursor location then write message (safe helpers)
        screen_clear_line(5)
        screen_print(str(MAX_DRIVE_SPEED) + "%", 1, 5)
        wait(timeout_ms, MSEC)
        # Leave the value visible — don't clear further to avoid erasing other messages
    except Exception:
        pass

def cycle_drive_speed_next():
    """Cycle to the next preset speed (bound to a controller button)."""
    global current_speed_index
    current_speed_index = (current_speed_index + 1) % len(PRESET_DRIVE_SPEEDS)
    set_max_drive_speed(PRESET_DRIVE_SPEEDS[current_speed_index])
    save_max_speed()
    show_max_speed(800)

def cycle_drive_speed_prev():
    """Cycle to the previous preset speed (bound to a controller button)."""
    global current_speed_index
    current_speed_index = (current_speed_index - 1) % len(PRESET_DRIVE_SPEEDS)
    
    set_max_drive_speed(PRESET_DRIVE_SPEEDS[current_speed_index])
    save_max_speed()
    show_max_speed(800)

# Ramping (limits how quickly the applied motor command can change, percent/sec)
RAMP_RATE_PERCENT_PER_SEC = 300.0  # default: 300% per second (0->100 in ~0.33s)

def set_ramp_rate(percent_per_sec):
    """Set ramp rate in percent per second (positive float)."""
    global RAMP_RATE_PERCENT_PER_SEC
    RAMP_RATE_PERCENT_PER_SEC = max(0.0, float(percent_per_sec))

# internal applied values for ramping (initialized to 0)
_applied_left = 0.0
_applied_right = 0.0
def now_seconds():
    """Return a monotonic-ish timestamp in seconds, compatible with different runtimes."""
    # prefer time.monotonic when available
    try:
        if hasattr(time, 'monotonic'):
            return time.monotonic()
    except Exception:
        pass
    # fallback to time.time
    try:
        if hasattr(time, 'time'):
            return float(time.time())
    except Exception:
        pass
    # final fallback to VEX brain timer (milliseconds -> seconds)
    try:
        return float(brain.timer.time(MSEC)) / 1000.0
    except Exception:
        return 0.0

_last_ramp_time = now_seconds()


# Always-on display thread for MAX_DRIVE_SPEED
def display_max_speed_loop():
    """Continuously shows MAX_DRIVE_SPEED in the top-right corner of the Brain screen."""
    # We'll print on row 1, column ~12 (adjust if your screen size differs)
    try:
        while True:
            try:
                screen_print(str(MAX_DRIVE_SPEED) + "%", 1, 12)
            except Exception:
                # ignore intermittent display errors
                pass
            wait(500, MSEC)
    except Exception:
        return



 

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

# start the always-on display (create the thread where startup code runs)
display_max_speed_thread = Thread(display_max_speed_loop)
# note: in this project Thread(...) starts automatically; no .start() call required

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
    


def score_forward():
    """ Start IntakeFirst3 and IntakeSecond2
    """
    intake_control(FORWARD)

def outake_forward():
    """ Start IntakeFirst3 and IntakeSecond2
    """
    outake_control(FORWARD)
def outake_backward():
    """ Start IntakeFirst3 and IntakeSecond2 in reverse
    """
    outake_control(REVERSE)


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





        

def inches_to_mm(inches):
    # Convert inches to millimeters
    return inches * 25.4
    pass # Placeholder for function logic
    # convert inches to millimeters
    return inches * 25.4    



# ---------------------------------------------------------------------------- #
       

# Duplicate calibrate_drivetrain definition removed.


# create competition instance
def user_control():
    # Add user control code here
    brain.screen.clear_screen()
    brain.screen.print("user control code")

# Autonomous code
def autonomous():
       
    calibrate_drivetrain()
   # IntakeFirst1.set_velocity(100, PERCENT)
    #IntakeSecond2.set_velocity(100, PERCENT)
    #IntakeThird3.set_velocity(100, PERCENT)
    #IntakeFourth4.set_velocity(100, PERCENT)
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)
    #ramp_actuator.set(True)

   # IntakeFirst1.spin(FORWARD, 10,TURNS , wait = False)
    #IntakeSecond2.spin(FORWARD, 10,TURNS, wait = False)
   
    drivetrain.drive_for(FORWARD, 36, INCHES)  # Drive forward 36 inches



   # turn_pid (305) # Drive forward 36 inches
     
   # IntakeFirst1.spin(REVERSE)
   # IntakeSecond2.spin(REVERSE)
   # IntakeThird3.spin(FORWARD)
pass  # Placeholder for autonomous logic

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
# Controller button bindings (centralized after controller creation)
controller_1.buttonLeft.pressed(toggle_gate)
controller_1.buttonRight.pressed(toggle_ramp)
controller_1.buttonY.pressed(toggle_roller)
# Bind controller buttons X/B to cycle drive speed presets
controller_1.buttonX.pressed(cycle_drive_speed_prev)
controller_1.buttonB.pressed(cycle_drive_speed_next)
controller_1.buttonR2.pressed(score_backward)
controller_1.buttonR1.pressed(score_forward)
controller_1.buttonL1.pressed(outake_forward)
controller_1.buttonL2.pressed(outake_backward)
controller_1.buttonA.pressed(load_ball)
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
Inertial1 = Inertial(Ports.PORT21)
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
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, remote_control_code_enabled, _applied_left, _applied_right, _last_ramp_time, target_left, target_right
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
            
            # calculate the drivetrain motor velocities from the controller joystick axes
            # left = forward + turn
            # right = forward - turn
            # Some controller runtimes map forward/back to axis3, others to axis2.
            # Read axis3 first, but if it is near-zero try axis2 as a fallback.
            try:
                forward = controller_1.axis3.position()
            except Exception:
                forward = 0
            try:
                # fallback if axis3 appears dead but axis2 has input
                if abs(forward) < 4:
                    a2 = controller_1.axis2.position()
                    if abs(a2) > abs(forward):
                        forward = a2
            except Exception:
                pass
            try:
                turn = controller_1.axis1.position()
            except Exception:
                turn = 0

            # raw joystick mixed speeds
            drivetrain_left_side_speed = forward + turn
            drivetrain_right_side_speed = forward - turn
            # scale by MAX_DRIVE_SPEED so teleop driving can be limited centrally
            # protect against a saved 0% value which would fully disable driving
            scale = max(1, MAX_DRIVE_SPEED) / 100.0
            drivetrain_left_side_speed *= scale
            drivetrain_right_side_speed *= scale

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
            
            # apply ramping to smoothly move from previous applied values to targets
            global _applied_left, _applied_right, _last_ramp_time
            now_t = now_seconds()
            dt = now_t - _last_ramp_time if _last_ramp_time is not None else 0.02
            _last_ramp_time = now_t

            # desired targets
            target_left = drivetrain_left_side_speed
            target_right = drivetrain_right_side_speed

            # max change allowed this loop (percent)
            ramp_rate = globals().get('RAMP_RATE_PERCENT_PER_SEC', 300.0)
            max_delta = ramp_rate * dt

            def ramp_toward(current, target, max_delta):
                diff = target - current
                if abs(diff) <= max_delta:
                    return target
                return current + max_delta * (1 if diff > 0 else -1)

            _applied_left = ramp_toward(_applied_left, target_left, max_delta)
            _applied_right = ramp_toward(_applied_right, target_right, max_delta)

            # If both the target and applied values are within the deadband, zero them
            # and make sure the motors are stopped. This prevents the robot from
            # coasting on small residual applied values after releasing the stick.
            DEADBAND = 5.0
            if abs(target_left) < DEADBAND and abs(_applied_left) < DEADBAND:
                _applied_left = 0.0
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    left_drive_smart.stop()
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            if abs(target_right) < DEADBAND and abs(_applied_right) < DEADBAND:
                _applied_right = 0.0
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    right_drive_smart.stop()
                    drivetrain_r_needs_to_be_stopped_controller_1 = False

            # only tell the left drive motor to spin if the values are not in the deadband range
            if abs(_applied_left) >= 0.5:
                left_drive_smart.set_velocity(_applied_left, PERCENT)
                left_drive_smart.spin(FORWARD)
            else:
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    left_drive_smart.stop()
                    drivetrain_l_needs_to_be_stopped_controller_1 = False

            # only tell the right drive motor to spin if the values are not in the deadband range
            if abs(_applied_right) >= 0.5:
                right_drive_smart.set_velocity(_applied_right, PERCENT)
                right_drive_smart.spin(FORWARD)
            else:
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    right_drive_smart.stop()
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
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

# Load saved max speed (best-effort)
load_max_speed()
#show_max_speed(800)
# Start the always-on display thread for max drive speed
#display_max_speed_thread = Thread(display_max_speed_loop)

class PIDController:
    """A time-aware PID controller with integral clamping and derivative filtering.

    Methods:
    - compute(measurement): returns control output (float)
    - reset(): clears integral and derivative history
    Usage: set controller.setpoint before calling compute().
    """
    def __init__(self, kp, ki, kd, *, output_limits=(None, None), integral_limits=(None, None), derivative_filter_alpha=0.0, sample_time=0.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        self.min_output, self.max_output = output_limits
        self.min_integral, self.max_integral = integral_limits
        # derivative filter alpha (0=no filter, closer to 1 = more smoothing)
        self.alpha = float(derivative_filter_alpha)
        self.sample_time = float(sample_time)

        self.integral = 0.0
        self.prev_measurement = None
        self.prev_derivative = 0.0
        self.last_time = None
        self.setpoint = 0.0

    def reset(self, preserve_setpoint=True):
        self.integral = 0.0
        self.prev_measurement = None
        self.prev_derivative = 0.0
        self.last_time = None
        if not preserve_setpoint:
            self.setpoint = 0.0

    def compute(self, measurement, now=None):
        """Compute PID output given current measurement. Uses time.monotonic() if now is None."""
        if now is None:
            now = now_seconds()

        if self.last_time is None:
            dt = 0.0
        else:
            dt = now - self.last_time

        # enforce optional minimum sample time
        if self.sample_time and dt < self.sample_time:
            return None

        error = self.setpoint - measurement

        # Proportional
        p = self.kp * error

        # Integral (with clamping / anti-windup)
        if dt > 0.0:
            self.integral += error * dt
            if self.min_integral is not None:
                self.integral = max(self.min_integral, self.integral)
            if self.max_integral is not None:
                self.integral = min(self.max_integral, self.integral)
        i = self.ki * self.integral

        # Derivative (on measurement to avoid setpoint step bump)
        d = 0.0
        if dt > 0.0 and self.prev_measurement is not None:
            derivative = (measurement - self.prev_measurement) / dt
            # derivative on error is -derivative_of_measurement
            deriv_val = -derivative
            filtered = self.alpha * self.prev_derivative + (1.0 - self.alpha) * deriv_val
            d = self.kd * filtered
            self.prev_derivative = filtered

        output = p + i + d

        # clamp output
        if self.min_output is not None:
            output = max(self.min_output, output)
        if self.max_output is not None:
            output = min(self.max_output, output)

        # save state
        self.prev_measurement = measurement
        self.last_time = now

        return output

# Drive PID: moves a set distance (mm)
def drive_pid (target_inches, timeout_ms=2000):
    drive_pid = PIDController(0.5, 0.01, 0.1, output_limits=(-AUTON_DRIVE_MAX, AUTON_DRIVE_MAX), integral_limits=(-500, 500), derivative_filter_alpha=0.6)
    heading_pid = PIDController(1.0, 0.0, 0.2, output_limits=(-AUTON_TURN_MAX, AUTON_TURN_MAX), derivative_filter_alpha=0.6)  # Heading correction
    left_drive_smart.reset_position()
    right_drive_smart.reset_position()
    initial_heading = drivetrain_inertial.rotation()
    start_time = brain.timer.time(MSEC)
    while True:
        avg_pos = ((left_drive_smart.position(RotationUnits.REV) + right_drive_smart.position(RotationUnits.REV)) / 2) * (319.19 * 3.14159 / 360)
        # Define INCH as millimeters per inch
        INCH = 25.4
        drive_pid.setpoint = target_inches * INCH
        drive_power = drive_pid.compute(avg_pos)
        if drive_power is None:
            # PID requested no update (sample_time); skip this loop
            wait(5, MSEC)
            continue
        drive_power = max(min(drive_power, AUTON_DRIVE_MAX), -AUTON_DRIVE_MAX)
        # Heading correction
        current_heading = drivetrain_inertial.rotation()
        heading_error = initial_heading - current_heading
        heading_pid.setpoint = 0
        correction = heading_pid.compute(heading_error * 0)  # run compute with a measurement consistent with heading error handling
        if correction is None:
            correction = 0
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
    turn_pid = PIDController(1.2, 0.01, 0.2, output_limits=(-AUTON_TURN_MAX, AUTON_TURN_MAX), derivative_filter_alpha=0.6)
    start_time = brain.timer.time(MSEC)
    while True:
        current_heading = drivetrain_inertial.rotation()
        turn_pid.setpoint = target_deg
        turn_power = turn_pid.compute(current_heading)
        if turn_power is None:
            wait(5, MSEC)
            continue
        turn_power = max(min(turn_power, AUTON_TURN_MAX), -AUTON_TURN_MAX)
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

# Define maximum drive output for autonomous PID (already declared above)

_debug_thread = None
_debug_enabled = False

def debug_display_loop():
    try:
        while True:
            # row 2..3, adjust columns if you need different layout
            try:
                al = int(globals().get('_applied_left', 0))
                ar = int(globals().get('_applied_right', 0))
                tl = int(globals().get('target_left', 0))
                tr = int(globals().get('target_right', 0))
                # also show raw axis values to help diagnose mapping problems
                try:
                    a3 = controller_1.axis3.position()
                except Exception:
                    a3 = 0
                try:
                    a2 = controller_1.axis2.position()
                except Exception:
                    a2 = 0
                try:
                    a1 = controller_1.axis1.position()
                except Exception:
                    a1 = 0
                screen_print("T L:" + str(al).rjust(4) + "% Tgt:" + str(tl).rjust(4) + "%", 2, 1)
                screen_print("T R:" + str(ar).rjust(4) + "% Tgt:" + str(tr).rjust(4) + "%", 3, 1)
                screen_print("A3:" + str(a3).rjust(4) + " A2:" + str(a2).rjust(4) + " A1:" + str(a1).rjust(4), 4, 1)
            except Exception:
                # ignore per-iteration display errors
                pass
            wait(200, MSEC)
    except Exception:
        return

def toggle_debug():
    global _debug_thread, _debug_enabled
    if not _debug_enabled:
        _debug_thread = Thread(debug_display_loop)
        _debug_enabled = True
    else:
        # best-effort: just flip flag; thread will exit only if you add a stop condition
        _debug_enabled = False
        brain.screen.clear_line(2)
        brain.screen.clear_line(3)

# bind to a button (example: controller Up to toggle)
controller_1.buttonUp.pressed(toggle_debug)

# new config
RAMP_UP_RATE_PERCENT_PER_SEC = 400.0   # faster accelerate
RAMP_DOWN_RATE_PERCENT_PER_SEC = 800.0 # faster braking/slowdown

def ramp_toward_asymmetric(current, target, dt):
    diff = target - current
    if diff > 0:
        max_delta = RAMP_UP_RATE_PERCENT_PER_SEC * dt
    else:
        max_delta = RAMP_DOWN_RATE_PERCENT_PER_SEC * dt
    if abs(diff) <= max_delta:
        return target
    return current + max_delta * (1 if diff > 0 else -1)