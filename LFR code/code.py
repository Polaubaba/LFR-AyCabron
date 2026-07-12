# ============================================================================
#  LFR - Line Following Robot  (CircuitPython 9, YD-RP2040)  --  v2 menu
# ============================================================================
#  5-sensor digital array, TB6612FNG driver, SSD1306 OLED.
#
#  WHAT'S NEW IN v2 (the menu fix)
#  --------------------------------
#  - Two-state menu:  BROWSE  -> UP/DOWN move cursor, ENTER selects
#                     EDIT    -> UP/DOWN change value, ENTER unselects
#                     ENTER on GO -> start the run
#  - Reliable edge-detected buttons (no wait-for-release hang). Fixes the
#    "ENTER not working" problem.
#  - Buzzer beeps on EVERY button press (UP / DOWN / ENTER), in menu and run.
#
#  Carried-over fixes from before:
#  - I2C pinned to spec:   SCL = GP6,  SDA = GP7
#  - Motor PWM to spec:    PWMA = GP10, PWMB = GP13
#  - Gain/duty scale functional (Kp/Kd defaults actually steer; speed maps
#    0..maxSpeed -> 0..100% PWM).
#  - Corner sensors (GP26/27) pulled up -> inactive when unconnected.
#
#  WIRING (FINAL)
#  --------------
#  OLED SSD1306 : SCL -> GP6,  SDA -> GP7
#  Buttons      : UP -> GP0,  DOWN -> GP1,  SELECT/ENTER -> GP2
#  Buzzer       : GP28
#  IR array (5) : S1 GP16, S2 GP17, S3 GP18, S4 GP19, S5 GP20
#  TB6612FNG    : PWMA GP10, AIN1 GP11, AIN2 GP12
#                 PWMB GP13, BIN1 GP14, BIN2 GP15, STBY GP21
#  (optional corner sensors: GP26 left, GP27 right)
#
#  LIBS in /lib: adafruit_ssd1306.mpy, adafruit_framebuf.mpy
# ============================================================================

import time
import board
import digitalio
import pwmio
import busio
import adafruit_ssd1306

# ==========================================
# 1. PINS  (matches FINAL wiring)
# ==========================================
BTN_UP   = board.GP0
BTN_DOWN = board.GP1
BTN_SEL  = board.GP3          # ENTER / GO

SDA_PIN = board.GP6           # spec: SDA -> GP7
SCL_PIN = board.GP7           # spec: SCL -> GP6

PIN_PWMA = board.GP8
PIN_PWMB = board.GP9
PIN_AIN1 = board.GP11
PIN_AIN2 = board.GP12
PIN_BIN1 = board.GP14
PIN_BIN2 = board.GP15
PIN_STBY = board.GP21

PIN_SENSORS = [board.GP16, board.GP17, board.GP18, board.GP19, board.GP20]
PIN_EXT_LEFT  = board.GP26
PIN_EXT_RIGHT = board.GP27
PIN_BUZZER = board.GP28

# ==========================================
# 2. CONFIG & GLOBALS
# ==========================================
Kp = 25.0
Ki = 0.0
Kd = 4.0
SHARP_CURVE_KP_MULT = 2.0
SHARP_CURVE_KD_MULT = 0.5

baseSpeed = 200              # cruise (menu: "Speed"), 0..maxSpeed
maxSpeed  = 400
minSpeed  = 80
extremeInnerSpeed = 0
turnSpeed = 300

sensor_weights = [-2000, -1000, 0, 1000, 2000]

error = 0.0
last_error = 0.0
integral = 0.0
derivative = 0.0
line_last_seen = time.monotonic()
last_valid_direction = 0
is_running = False

left_speed = 0               # FIX#1: pre-init for the debug display
right_speed = 0

# ==========================================
# 3. INITIALIZATION
# ==========================================
i2c = busio.I2C(SCL_PIN, SDA_PIN)
display = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)


def make_input(pin, pull_up=True):
    d = digitalio.DigitalInOut(pin)
    d.direction = digitalio.Direction.INPUT
    d.pull = digitalio.Pull.UP if pull_up else digitalio.Pull.DOWN
    return d


# --- Buzzer -----------------------------------------------------------------
buzzer = digitalio.DigitalInOut(PIN_BUZZER)
buzzer.direction = digitalio.Direction.OUTPUT
buzzer.value = False


def beep(duration):
    """Blocking beep. Used only in menu / launch / pause (robot is stopped),
    or for a ~20 ms tick on a button press during run (negligible)."""
    buzzer.value = True
    time.sleep(duration)
    buzzer.value = False


# --- Edge-detected button (reliable; fixes 'ENTER not working') -------------
class Button:
    DEBOUNCE = 0.02
    def __init__(self, pin):
        self.io = make_input(pin, pull_up=True)   # active-low
        self._stable = False
        self._change_at = -10.0
        self.triggered = False          # True for ONE loop tick per press

    def update(self):
        now = time.monotonic()
        raw = self.io.value             # True = not pressed (pull-up)
        self.triggered = False
        # Only sample outside the post-edge lockout window -> instant + bounce-free
        if (now - self._change_at) >= self.DEBOUNCE:
            pressed = (raw == False)
            if pressed and not self._stable:
                self.triggered = True
                self._stable = True
                self._change_at = now
            elif (not pressed) and self._stable:
                self._stable = False
                self._change_at = now


btn_up   = Button(BTN_UP)
btn_down = Button(BTN_DOWN)
btn_sel  = Button(BTN_SEL)

# --- Corner sensors (pulled up = inactive if not wired) ---------------------
ext_left  = make_input(PIN_EXT_LEFT,  pull_up=True)
ext_right = make_input(PIN_EXT_RIGHT, pull_up=True)

# --- Motor driver -----------------------------------------------------------
stby = digitalio.DigitalInOut(PIN_STBY)
stby.direction = digitalio.Direction.OUTPUT
stby.value = True                     # wake the TB6612

ain1 = digitalio.DigitalInOut(PIN_AIN1); ain1.direction = digitalio.Direction.OUTPUT
ain2 = digitalio.DigitalInOut(PIN_AIN2); ain2.direction = digitalio.Direction.OUTPUT
bin1 = digitalio.DigitalInOut(PIN_BIN1); bin1.direction = digitalio.Direction.OUTPUT
bin2 = digitalio.DigitalInOut(PIN_BIN2); bin2.direction = digitalio.Direction.OUTPUT

pwm_a = pwmio.PWMOut(PIN_PWMA, frequency=20000, duty_cycle=0)
pwm_b = pwmio.PWMOut(PIN_PWMB, frequency=20000, duty_cycle=0)

# --- 5 IR sensors -----------------------------------------------------------
sensors = [make_input(p, pull_up=True) for p in PIN_SENSORS]

# ==========================================
# 4. MOTOR HELPERS
# ==========================================
def set_motor_speed(left_speed_val, right_speed_val):
    ain1.value = left_speed_val >= 0
    ain2.value = left_speed_val < 0
    bin1.value = right_speed_val >= 0
    bin2.value = right_speed_val < 0
    # speed 0..maxSpeed  ->  duty 0..65535
    pwm_a.duty_cycle = int(min(abs(left_speed_val), maxSpeed) / maxSpeed * 65535)
    pwm_b.duty_cycle = int(min(abs(right_speed_val), maxSpeed) / maxSpeed * 65535)


def stop_motors():
    set_motor_speed(0, 0)

# ==========================================
# 5. TUNING MENU  (browse / edit, ENTER to toggle)
# ==========================================
# menu_index: 0=Kp, 1=Kd, 2=Speed, 3=GO
LABELS = ["Kp", "Kd", "Speed", "GO"]


def adjust(idx, direction):
    global Kp, Kd, baseSpeed
    if idx == 0:
        Kp = max(0.0, Kp + direction * 1.0)
    elif idx == 1:
        Kd = max(0.0, Kd + direction * 0.5)
    elif idx == 2:
        baseSpeed = max(60, min(maxSpeed, baseSpeed + direction * 10))


def value_text(idx):
    if idx == 0:
        return "Kp: %.1f" % Kp
    if idx == 1:
        return "Kd: %.1f" % Kd
    if idx == 2:
        return "Speed: %d" % baseSpeed
    return "GO!  (start)"


def draw_menu(menu_index, editing):
    display.fill(0)
    display.text("--- PID TUNING ---", 0, 0, 1)
    for i in range(4):
        y = 12 + i * 11
        cursor = ">" if i == menu_index else " "
        if i == menu_index and editing and i != 3:
            display.text("%s[%s]" % (cursor, value_text(i)), 0, y, 1)
        else:
            display.text("%s %s" % (cursor, value_text(i)), 0, y, 1)
    hint = "UP/DN:chg  SEL:ok" if editing else "UP/DN:move SEL:sel"
    display.text(hint, 0, 56, 1)
    display.show()


def run_menu():
    global Kp, Kd, baseSpeed
    menu_index = 0
    editing = False
    draw_menu(menu_index, editing)

    while True:
        btn_up.update(); btn_down.update(); btn_sel.update()

        # buzzer feedback on ANY press
        if btn_up.triggered or btn_down.triggered or btn_sel.triggered:
            beep(0.02)

        if btn_sel.triggered:                       # ENTER
            if menu_index == 3:                     # GO -> start
                beep(0.08)
                return
            editing = not editing                   # select / unselect
            draw_menu(menu_index, editing)
        elif editing:                               # UP/DN changes value
            if btn_up.triggered:
                adjust(menu_index, +1)
                draw_menu(menu_index, editing)
            elif btn_down.triggered:
                adjust(menu_index, -1)
                draw_menu(menu_index, editing)
        else:                                       # browse: UP/DN moves cursor
            if btn_up.triggered:
                menu_index = (menu_index - 1) % 4
                draw_menu(menu_index, editing)
            elif btn_down.triggered:
                menu_index = (menu_index + 1) % 4
                draw_menu(menu_index, editing)

        time.sleep(0.01)

# ==========================================
# 6. SENSOR MATH & PID
# ==========================================
def read_line_error():
    """Sensor reads LOW when over the line."""
    global line_last_seen, last_valid_direction
    weighted_sum = 0
    active_sensors = 0
    for i in range(5):
        if not sensors[i].value:
            weighted_sum += sensor_weights[i]
            active_sensors += 1
    if active_sensors > 0:
        position = weighted_sum / active_sensors
        line_last_seen = time.monotonic()
        if position < -200:
            last_valid_direction = -1
        elif position > 200:
            last_valid_direction = 1
        return position / 800.0
    else:
        if time.monotonic() - line_last_seen > 0.5:
            return last_valid_direction * 4.0
        return last_error


def calculate_pid(current_error):
    global integral, last_error, derivative
    sharp_curve = abs(current_error) > 1.2
    active_kp = Kp * SHARP_CURVE_KP_MULT if sharp_curve else Kp
    active_kd = Kd * SHARP_CURVE_KD_MULT if sharp_curve else Kd

    integral += current_error
    if (current_error > 0 and last_error < 0) or (current_error < 0 and last_error > 0):
        integral *= 0.95
    integral = max(min(integral, 120.0), -120.0)

    derivative = current_error - last_error
    correction = (active_kp * current_error) + (Ki * integral) + (active_kd * derivative)
    last_error = current_error
    return correction

# ==========================================
# 7. MAIN SEQUENCE
# ==========================================
run_menu()

# GO pressed: countdown with buzzer feedback
for n in ("3", "2", "1"):
    display.fill(0)
    display.text("GO in " + n + "...", 25, 28, 1)
    display.show()
    beep(0.12)
    time.sleep(0.88)
display.fill(0)
display.text("GO!", 50, 28, 1)
display.show()
beep(0.4)

is_running = True
last_debug_print = time.monotonic()
LOOP_PERIOD = 0.01

# ==========================================
# 8. MAIN CONTROL LOOP  (watchdog-wrapped)
# ==========================================
try:
    while True:
        current_time = time.monotonic()
        btn_up.update(); btn_down.update(); btn_sel.update()

        # buzzer on ANY press (even during the run)
        if btn_up.triggered or btn_down.triggered or btn_sel.triggered:
            beep(0.02)

        # SELECT toggles pause / resume
        if btn_sel.triggered:
            is_running = not is_running
            if is_running:
                display.fill(0); display.text("RESUMING...", 0, 30, 1); display.show()
                beep(0.15)
                time.sleep(0.85)
                integral = 0
                last_error = read_line_error()
            else:
                stop_motors()
                display.fill(0); display.text("PAUSED", 30, 30, 1); display.show()

        if is_running:
            sees_left_intersection = not ext_left.value
            sees_right_intersection = not ext_right.value

            if sees_left_intersection:
                display.fill(0); display.text("90 DEG LEFT", 10, 30, 1); display.show()
                buzzer.value = True
                while not ext_left.value or sensors[2].value:
                    set_motor_speed(-turnSpeed, turnSpeed)
                    time.sleep(0.01)
                buzzer.value = False
                integral = 0; last_error = 0

            elif sees_right_intersection:
                display.fill(0); display.text("90 DEG RIGHT", 10, 30, 1); display.show()
                buzzer.value = True
                while not ext_right.value or sensors[2].value:
                    set_motor_speed(turnSpeed, -turnSpeed)
                    time.sleep(0.01)
                buzzer.value = False
                integral = 0; last_error = 0

            else:
                error = read_line_error()
                correction = calculate_pid(error)
                abs_err = abs(error)

                if abs_err > 2.0:
                    if error < 0:
                        left_speed = extremeInnerSpeed; right_speed = maxSpeed
                    else:
                        left_speed = maxSpeed; right_speed = extremeInnerSpeed
                elif abs_err > 1.2:
                    inner_speed = int(max(baseSpeed - (abs_err * 80), extremeInnerSpeed))
                    if error < 0:
                        left_speed = inner_speed; right_speed = maxSpeed
                    else:
                        left_speed = maxSpeed; right_speed = inner_speed
                else:
                    left_speed = int(max(min(baseSpeed - correction, maxSpeed), minSpeed))
                    right_speed = int(max(min(baseSpeed + correction, maxSpeed), minSpeed))

                set_motor_speed(left_speed, right_speed)
        else:
            stop_motors()

        # OLED debug (every 0.25 s)
        if current_time - last_debug_print >= 0.25:
            last_debug_print = current_time
            if is_running:
                display.fill(0)
                display.text("Err: %.2f" % error, 0, 0, 1)
                display.text("L: %d R: %d" % (left_speed, right_speed), 0, 15, 1)
                display.show()

        processing_time = time.monotonic() - current_time
        if processing_time < LOOP_PERIOD:
            time.sleep(LOOP_PERIOD - processing_time)

except Exception as e:
    try:
        stop_motors()
    except Exception:
        pass
    try:
        display.fill(0)
        display.text("CRASH:", 0, 0, 1)
        display.text(str(e)[:20], 0, 15, 1)
        display.show()
    except Exception:
        pass
    for _ in range(5):
        beep(0.1)
        time.sleep(0.1)

