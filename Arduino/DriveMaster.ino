#include <Encoder.h>
#include <Servo.h>
#include <avr/pgmspace.h>
#include "Config.h"

#if BNO055_ENABLE
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#endif

struct PID {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float integral;
    float prevError;

    float maxOutput;
    float maxIntegral;
};

// PJRC Encoder handles the two external-interrupt encoders (pins
// 2,3,18,19) reliably. The two pin-change encoders all live in the
// single PCINT0 group on PORTB, where PJRC Encoder is unreliable, so
// they are decoded by the hand-written PCINT0 ISR below.
//
// Wheel-to-encoder assignment and per-wheel counting direction for
// the pin-change encoders are configured entirely in Config.h via the
// ENC3_*_BIT / ENC4_*_BIT and ENC3_DIR / ENC4_DIR macros.
Encoder enc1(ENC1_A, ENC1_B);
Encoder enc2(ENC2_A, ENC2_B);

// Quadrature transition table indexed by (prevState << 2) | newState,
// where state is (A << 1) | B. +1 / -1 per valid edge, 0 for no-change
// or illegal (double) transitions.
static const int8_t QUAD_TABLE[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

volatile long enc3Count = 0;
volatile long enc4Count = 0;
volatile uint8_t enc3PrevState = 0;
volatile uint8_t enc4PrevState = 0;

// ============================================================
// BNO055 IMU STATE
//   imuReady is set true once begin() succeeds. It is checked
//   lazily: if the IMU fails to come up at boot, the firmware
//   keeps running and telemetry reports the IMU as offline rather
//   than blocking the control loop. A retry is attempted at the
//   IMU sample cadence so a late-powered or reconnected sensor is
//   picked up without a reboot.
//
//   The fused outputs cached here are heading/roll/pitch (deg) from
//   the absolute orientation Euler vector, angular velocity (deg/s)
//   from the gyro, linear acceleration (m/s^2, gravity removed), and
//   the four calibration status bytes (0..3 each).
// ============================================================
#if BNO055_ENABLE
Adafruit_BNO055 bno(-1, BNO055_ADDRESS, &Wire);
bool imuReady = false;
unsigned long lastImuSample = 0;

float imuHeading = 0.0, imuRoll = 0.0, imuPitch = 0.0;
float imuGx = 0.0, imuGy = 0.0, imuGz = 0.0;
float imuAx = 0.0, imuAy = 0.0, imuAz = 0.0;
uint8_t calSys = 0, calGyro = 0, calAccel = 0, calMag = 0;
#endif

// FUNCTION: PCINT0 ISR - decodes enc3 and enc4 from a single PORTB read
ISR(PCINT0_vect)
{
    uint8_t p = PINB;

    uint8_t s3 = (((p >> ENC3_A_BIT) & 0x01) << 1) | ((p >> ENC3_B_BIT) & 0x01);
    uint8_t idx3 = (enc3PrevState << 2) | s3;
    enc3Count += (long)ENC3_DIR * QUAD_TABLE[idx3 & 0x0F];
    enc3PrevState = s3;

    uint8_t s4 = (((p >> ENC4_A_BIT) & 0x01) << 1) | ((p >> ENC4_B_BIT) & 0x01);
    uint8_t idx4 = (enc4PrevState << 2) | s4;
    enc4Count += (long)ENC4_DIR * QUAD_TABLE[idx4 & 0x0F];
    enc4PrevState = s4;
}

// FUNCTION: Configure PCINT0 for the four PORTB encoder pins
void setupPinChangeEncoders()
{
    // Inputs with pull-ups on PB0, PB2, PB4, PB5
    DDRB  &= ~((1 << ENC3_A_BIT) | (1 << ENC3_B_BIT) |
               (1 << ENC4_A_BIT) | (1 << ENC4_B_BIT));
    PORTB |=  ((1 << ENC3_A_BIT) | (1 << ENC3_B_BIT) |
               (1 << ENC4_A_BIT) | (1 << ENC4_B_BIT));

    // Seed the previous-state with current pin levels so the first
    // edge after boot is not misread.
    uint8_t p = PINB;
    enc3PrevState = (((p >> ENC3_A_BIT) & 0x01) << 1) | ((p >> ENC3_B_BIT) & 0x01);
    enc4PrevState = (((p >> ENC4_A_BIT) & 0x01) << 1) | ((p >> ENC4_B_BIT) & 0x01);

    // Enable pin-change interrupts for the four bits, then the PCINT0 group.
    PCMSK0 |= (1 << ENC3_A_BIT) | (1 << ENC3_B_BIT) |
              (1 << ENC4_A_BIT) | (1 << ENC4_B_BIT);
    PCICR  |= (1 << PCIE0);
}

// ============================================================
// LIFTER CONTROL  (left + right Cytron channels, hardcoded values)
//   Lifters can be driven TOGETHER (LIFT command) via lifterUp/Down/Stop,
//   or INDIVIDUALLY via the right-only (rightLifter*) and left-only
//   (leftLifter*) helpers. Each side uses its OWN hardcoded PWM magnitude
//   (LEFT_* vs RIGHT_*) from Config.h so a mechanically mismatched pair
//   can be balanced.
//
//   The KFS lifter (kfsLifter*) is a third, independent Cytron channel
//   on the ARM_LIFTER pins, driven only by its own KFSLIFT command.
//
//   Per-side state is cached for telemetry / debug visibility:
//     0 = STOP, 1 = UP, -1 = DOWN
//   The combined lifterState mirrors both sides only when they match;
//   when they differ it reports STOP-agnostic 2 (MIXED). STOP is the
//   default / boot state. The KFS lifter is NOT part of the combined
//   state and reports independently.
// ============================================================
int rightLifterState = 0;
int leftLifterState  = 0;
int kfsLifterState   = 0;

// FUNCTION: Combined lifter state for telemetry
//   1 = both UP, -1 = both DOWN, 0 = both STOP, 2 = sides differ (MIXED)
int lifterCombinedState()
{
    if (rightLifterState == leftLifterState) return rightLifterState;
    return 2;
}

// ---- RIGHT LIFTER ONLY -------------------------------------

// FUNCTION: Drive the right lifter UP at RIGHT_LIFT_UP_PWM
void rightLifterUp()
{
    digitalWrite(RIGHT_LIFTER_DIR, RIGHT_LIFTER_DIR_UP);
    analogWrite(RIGHT_LIFTER_PWM, RIGHT_LIFT_UP_PWM);
    rightLifterState = 1;
}

// FUNCTION: Drive the right lifter DOWN at RIGHT_LIFT_DOWN_PWM
void rightLifterDown()
{
    digitalWrite(RIGHT_LIFTER_DIR, RIGHT_LIFTER_DIR_DOWN);
    analogWrite(RIGHT_LIFTER_PWM, RIGHT_LIFT_DOWN_PWM);
    rightLifterState = -1;
}

// FUNCTION: Stop the right lifter (PWM = 0, DIR left as-is)
void rightLifterStop()
{
    analogWrite(RIGHT_LIFTER_PWM, 0);
    rightLifterState = 0;
}

// ---- LEFT LIFTER ONLY --------------------------------------

// FUNCTION: Drive the left lifter UP at LEFT_LIFT_UP_PWM
void leftLifterUp()
{
    digitalWrite(LEFT_LIFTER_DIR, LEFT_LIFTER_DIR_UP);
    analogWrite(LEFT_LIFTER_PWM, LEFT_LIFT_UP_PWM);
    leftLifterState = 1;
}

// FUNCTION: Drive the left lifter DOWN at LEFT_LIFT_DOWN_PWM
void leftLifterDown()
{
    digitalWrite(LEFT_LIFTER_DIR, LEFT_LIFTER_DIR_DOWN);
    analogWrite(LEFT_LIFTER_PWM, LEFT_LIFT_DOWN_PWM);
    leftLifterState = -1;
}

// FUNCTION: Stop the left lifter (PWM = 0, DIR left as-is)
void leftLifterStop()
{
    analogWrite(LEFT_LIFTER_PWM, 0);
    leftLifterState = 0;
}

// ---- KFS LIFTER ONLY ---------------------------------------

// FUNCTION: Drive the KFS lifter UP at KFS_LIFT_UP_PWM
void kfsLifterUp()
{
    digitalWrite(ARM_LIFTER_DIR, KFS_LIFTER_DIR_UP);
    analogWrite(ARM_LIFTER_PWM, KFS_LIFT_UP_PWM);
    kfsLifterState = 1;
}

// FUNCTION: Drive the KFS lifter DOWN at KFS_LIFT_DOWN_PWM
void kfsLifterDown()
{
    digitalWrite(ARM_LIFTER_DIR, KFS_LIFTER_DIR_DOWN);
    analogWrite(ARM_LIFTER_PWM, KFS_LIFT_DOWN_PWM);
    kfsLifterState = -1;
}

// FUNCTION: Stop the KFS lifter (PWM = 0, DIR left as-is)
void kfsLifterStop()
{
    analogWrite(ARM_LIFTER_PWM, 0);
    kfsLifterState = 0;
}

// ---- ALL LIFTERS TOGETHER (drive pair) ---------------------

// FUNCTION: Set all lifter pins to their idle (stopped) state
//   This is the default state: PWM = 0 on every side, all STOP.
//   Includes the KFS lifter so the ARM_LIFTER channel boots idle.
void setupLifters()
{
    pinMode(RIGHT_LIFTER_PWM, OUTPUT);
    pinMode(RIGHT_LIFTER_DIR, OUTPUT);
    pinMode(LEFT_LIFTER_PWM,  OUTPUT);
    pinMode(LEFT_LIFTER_DIR,  OUTPUT);
    pinMode(ARM_LIFTER_PWM,   OUTPUT);
    pinMode(ARM_LIFTER_DIR,   OUTPUT);

    analogWrite(RIGHT_LIFTER_PWM, 0);
    analogWrite(LEFT_LIFTER_PWM,  0);
    analogWrite(ARM_LIFTER_PWM,   0);
    rightLifterState = 0;
    leftLifterState  = 0;
    kfsLifterState   = 0;
}

// FUNCTION: Drive both drive-pair lifters UP, each at its own hardcoded UP PWM
//   (KFS lifter is independent and not affected by LIFT.)
void lifterUp()
{
    rightLifterUp();
    leftLifterUp();
}

// FUNCTION: Drive both drive-pair lifters DOWN, each at its own hardcoded DOWN PWM
//   (KFS lifter is independent and not affected by LIFT.)
void lifterDown()
{
    rightLifterDown();
    leftLifterDown();
}

// FUNCTION: Stop both drive-pair lifters (PWM = 0, DIR left as-is). Default state.
//   (KFS lifter is independent and not affected by LIFT.)
void lifterStop()
{
    rightLifterStop();
    leftLifterStop();
}

// ============================================================
// SMOOTH (RATE-LIMITED) SERVO FRAMEWORK
//   Every servo on the bot -- the three arm joints and both grippers
//   -- is wrapped in a SmoothServo so NO motion is ever abrupt. A
//   command never calls write() directly; it only sets `target`. The
//   non-blocking updateServos() stepper (called every loop) walks
//   `current` toward `target` at the servo's own slew rate (deg/sec
//   from Config.h) and writes the integer angle to the hardware only
//   when it actually changes.
//
//   Fields:
//     servo    : the underlying Servo driver
//     pin      : signal pin (attached in setupServos)
//     current  : live angle (float, deg) -- ramps toward target
//     target   : commanded goal angle (deg)
//     degPerMs : slew rate converted to deg per millisecond
//     lastInt  : last integer angle actually written (write only on change)
//
//   Boot: current == target == the boot angle, so the bot powers up
//   already at rest at a known pose with nothing to slew.
// ============================================================
struct SmoothServo {
    Servo servo;
    uint8_t pin;
    float current;
    float target;
    float degPerMs;
    int lastInt;
};

// Servo index map (used by the SERVO command and telemetry):
//   0 = arm base, 1 = arm mid, 2 = arm front,
//   3 = staff gripper, 4 = KFS gripper, 5 = staff rotate
enum {
    SRV_ARM_BASE = 0,
    SRV_ARM_MID,
    SRV_ARM_FRONT,
    SRV_STAFF_GRIP,
    SRV_KFS_GRIP,
    SRV_STAFF_ROTATE,
    SRV_COUNT
};

SmoothServo servos[SRV_COUNT];

// State caches kept for telemetry / debug visibility. These now track
// the COMMANDED logical state (the target the command asked for); the
// physical angle is still ramping via current and is reported
// separately in telemetry.
//   gripperState / kfsGripperState : 0 = CLOSED, 1 = OPEN
//   armState : 0 = STOW, 1 = MID, 2 = EXTEND, -1 = MANUAL (individual move)
int gripperState    = 1;
int kfsGripperState = 1;
int armState        = 0;

// FUNCTION: Initialise one SmoothServo (attach + seed at a boot angle)
//   current and target are both set to bootAngle so there is no
//   start-up lurch; the servo holds bootAngle until a command moves it.
void initServo(uint8_t idx, uint8_t pin, float slewDps, int bootAngle)
{
    servos[idx].pin      = pin;
    servos[idx].current  = (float)bootAngle;
    servos[idx].target   = (float)bootAngle;
    servos[idx].degPerMs = slewDps / 1000.0f;
    servos[idx].lastInt  = bootAngle;
    servos[idx].servo.attach(pin);
    servos[idx].servo.write(bootAngle);
}

// FUNCTION: Set a servo's target angle (clamped). Does NOT move it now;
//   the move happens gradually in updateServos(). This is the single
//   choke point every command routes through, so smoothing is universal.
void setServoTarget(uint8_t idx, float angle)
{
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    servos[idx].target = angle;
}

// FUNCTION: Non-blocking servo stepper -- ramp every servo toward target
//   Called every loop with the elapsed milliseconds since the last call.
//   Each servo moves at most (degPerMs * dtMs) degrees this tick, so it
//   slews at its configured deg/sec regardless of how big the commanded
//   jump was. The hardware write() happens only when the rounded integer
//   angle changes, to avoid spamming identical positions.
void updateServos(unsigned long dtMs)
{
    for (uint8_t i = 0; i < SRV_COUNT; i++) {
        SmoothServo &s = servos[i];
        float diff = s.target - s.current;
        if (diff != 0.0f) {
            float step = s.degPerMs * (float)dtMs;
            if (fabs(diff) <= step) {
                s.current = s.target;          // close enough: snap to target
            } else {
                s.current += (diff > 0 ? step : -step);
            }
            int ang = (int)(s.current + 0.5f);
            if (ang != s.lastInt) {
                s.servo.write(ang);
                s.lastInt = ang;
            }
        }
    }
}

// ============================================================
// STAFF GRIPPER  (servo on STAFF_GRIPPER_PIN, hardcoded angles)
//   GRIPPER,OPEN / GRIPPER,CLOSE set the target to one of two fixed
//   angles from Config.h; the gripper then SLEWS there. Boots OPEN.
//     gripperState: 0 = CLOSED, 1 = OPEN
// ============================================================

// FUNCTION: Target the staff gripper to its hardcoded OPEN angle
void gripperOpen()
{
    setServoTarget(SRV_STAFF_GRIP, STAFF_GRIPPER_OPEN_ANGLE);
    gripperState = 1;
}

// FUNCTION: Target the staff gripper to its hardcoded CLOSE angle
void gripperClose()
{
    setServoTarget(SRV_STAFF_GRIP, STAFF_GRIPPER_CLOSE_ANGLE);
    gripperState = 0;
}

// ============================================================
// KFS GRIPPER  (servo on KFS_GRIPPER_PIN, hardcoded angles)
//   KFSGRIP,OPEN / KFSGRIP,CLOSE set the target; the gripper SLEWS
//   there. Boots OPEN.
//     kfsGripperState: 0 = CLOSED, 1 = OPEN
// ============================================================

// FUNCTION: Target the KFS gripper to its hardcoded OPEN angle
void kfsGripperOpen()
{
    setServoTarget(SRV_KFS_GRIP, KFS_GRIPPER_OPEN_ANGLE);
    kfsGripperState = 1;
}

// FUNCTION: Target the KFS gripper to its hardcoded CLOSE angle
void kfsGripperClose()
{
    setServoTarget(SRV_KFS_GRIP, KFS_GRIPPER_CLOSE_ANGLE);
    kfsGripperState = 0;
}

// ============================================================
// KFS ARM  (three servos: base + mid + front, driven as one mechanism)
//   The arm has THREE fixed whole-arm poses. Each pose sets a TARGET
//   angle (from Config.h) on all three servos at once; they then SLEW
//   together to the pose. Individual servos can also be moved with the
//   SERVO command, which sets armState to MANUAL (-1).
//
//   armState (telemetry): 0 = STOW, 1 = MID, 2 = EXTEND, -1 = MANUAL
//   Boots in the STOW pose.
// ============================================================

// FUNCTION: Target all three arm servos to one pose's angle set
//   Helper used by every pose so the three targets always stay together.
void armWritePose(int baseAngle, int midAngle, int frontAngle)
{
    setServoTarget(SRV_ARM_BASE,  baseAngle);
    setServoTarget(SRV_ARM_MID,   midAngle);
    setServoTarget(SRV_ARM_FRONT, frontAngle);
}

// FUNCTION: Move the arm to the STOW pose (all three servos)
void armStow()
{
    armWritePose(KFS_ARM_BASE_STOW_ANGLE,
                 KFS_ARM_MID_STOW_ANGLE,
                 KFS_ARM_FRONT_STOW_ANGLE);
    armState = 0;
}

// FUNCTION: Move the arm to the MID pose (all three servos)
void armMid()
{
    armWritePose(KFS_ARM_BASE_MID_ANGLE,
                 KFS_ARM_MID_MID_ANGLE,
                 KFS_ARM_FRONT_MID_ANGLE);
    armState = 1;
}

// FUNCTION: Move the arm to the EXTEND pose (all three servos)
void armExtend()
{
    armWritePose(KFS_ARM_BASE_EXTEND_ANGLE,
                 KFS_ARM_MID_EXTEND_ANGLE,
                 KFS_ARM_FRONT_EXTEND_ANGLE);
    armState = 2;
}

// FUNCTION: Attach all six servos and seed them at their boot angles
//   Arm boots STOW; both grippers boot OPEN; staff rotate boots at
//   STAFF_ROTATE_BOOT_ANGLE. current == target at boot so nothing slews
//   until the first command.
void setupServos()
{
    initServo(SRV_ARM_BASE,    KFS_ARM_BASE_PIN,   KFS_ARM_BASE_SLEW_DPS,  KFS_ARM_BASE_STOW_ANGLE);
    initServo(SRV_ARM_MID,     KFS_ARM_MID_PIN,    KFS_ARM_MID_SLEW_DPS,   KFS_ARM_MID_STOW_ANGLE);
    initServo(SRV_ARM_FRONT,   KFS_ARM_FRONT_PIN,  KFS_ARM_FRONT_SLEW_DPS, KFS_ARM_FRONT_STOW_ANGLE);
    initServo(SRV_STAFF_GRIP,  STAFF_GRIPPER_PIN,  STAFF_GRIPPER_SLEW_DPS, STAFF_GRIPPER_OPEN_ANGLE);
    initServo(SRV_KFS_GRIP,    KFS_GRIPPER_PIN,    KFS_GRIPPER_SLEW_DPS,   KFS_GRIPPER_OPEN_ANGLE);
    initServo(SRV_STAFF_ROTATE, STAFF_ROTATE,      STAFF_ROTATE_SLEW_DPS,  STAFF_ROTATE_BOOT_ANGLE);

    armState        = 0;   // STOW
    gripperState    = 1;   // OPEN
    kfsGripperState = 1;   // OPEN
}

// Atomic readers for the volatile ISR counters.
long readEnc3()
{
    long v;
    noInterrupts();
    v = enc3Count;
    interrupts();
    return v;
}

long readEnc4()
{
    long v;
    noInterrupts();
    v = enc4Count;
    interrupts();
    return v;
}

#if BNO055_ENABLE
// FUNCTION: Attempt to bring the BNO055 online (lazy / retryable)
//   Returns true if the sensor is up. Safe to call repeatedly; once
//   imuReady is set it is a no-op. The external-crystal call improves
//   timing stability and is harmless if the board lacks the crystal.
bool tryInitIMU()
{
    if (imuReady) return true;
    if (!bno.begin()) return false;
    bno.setExtCrystalUse(true);
    imuReady = true;
    return true;
}

// FUNCTION: Poll the BNO055 and cache fused outputs
//   Called from the telemetry path at IMU_SAMPLE_MS cadence. If the
//   sensor is not up, it retries init first. All reads are best-effort:
//   the BNO055 driver is blocking on I2C, but at 100 kHz/400 kHz the
//   three vector reads cost far less than the DT_MS control budget.
void updateIMU()
{
    unsigned long now = millis();
    if (now - lastImuSample < IMU_SAMPLE_MS) return;
    lastImuSample = now;

    if (!tryInitIMU()) {
        imuReady = false;
        return;
    }

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> lin   = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    imuHeading = euler.x();   // heading / yaw (deg)
    imuRoll    = euler.y();   // roll (deg)
    imuPitch   = euler.z();   // pitch (deg)

    imuGx = gyro.x(); imuGy = gyro.y(); imuGz = gyro.z();   // deg/s
    imuAx = lin.x();  imuAy = lin.y();  imuAz = lin.z();     // m/s^2

    bno.getCalibration(&calSys, &calGyro, &calAccel, &calMag);
}
#endif

long lastCount1 = 0, lastCount2 = 0, lastCount3 = 0, lastCount4 = 0;
unsigned long lastLoopTime = 0;
float rpm1 = 0.0, rpm2 = 0.0, rpm3 = 0.0, rpm4 = 0.0;
int outCmd1 = 0, outCmd2 = 0, outCmd3 = 0, outCmd4 = 0;

bool usePID = true;
int rawCmd1 = 0, rawCmd2 = 0, rawCmd3 = 0, rawCmd4 = 0;

PID pid1 = {PID1_KP, PID1_KI, PID1_KD,  0, 0, 0, PID_MAX_OUTPUT, PID_MAX_INTEGRAL};
PID pid2 = {PID2_KP, PID2_KI, PID2_KD,  0, 0, 0, PID_MAX_OUTPUT, PID_MAX_INTEGRAL};
PID pid3 = {PID3_KP, PID3_KI, PID3_KD,  0, 0, 0, PID_MAX_OUTPUT, PID_MAX_INTEGRAL};
PID pid4 = {PID4_KP, PID4_KI, PID4_KD,  0, 0, 0, PID_MAX_OUTPUT, PID_MAX_INTEGRAL};

// Wheel-indexed pointer table so GAINW can address a single PID by number.
// Order matches IK / telemetry: 1=FL, 2=FR, 3=RL, 4=RR.
PID* const pidByWheel[4] = { &pid1, &pid2, &pid3, &pid4 };

#if BNO055_ENABLE
// ============================================================
// HEADING-HOLD STATE  (HDRIVE / HHOLD commands)
//   headHoldActive : true once HDRIVE / HHOLD,SET sets a target.
//     Cleared by any plain DRIVE / PID / RAW command so those modes
//     are never silently overridden by a stale heading loop.
//   headTarget     : desired absolute heading (deg, BNO055 frame 0..360).
//   hdVx / hdVy    : linear command latched from the last HDRIVE so the
//     control loop can re-run IK every tick with a freshly computed omega.
//   headHold       : a PID struct reused for the heading loop. Only
//     Kp / Ki / Kd / integral / prevError / maxIntegral are used here;
//     setpoint / maxOutput are unused (output is clamped to
//     MAX_ANGULAR_VEL directly inside computeHeadingOmega()).
// ============================================================
bool  headHoldActive = false;
float headTarget = 0.0;
float hdVx = 0.0, hdVy = 0.0;
PID   headHold = {HEAD_KP, HEAD_KI, HEAD_KD, 0, 0, 0, 0, HEAD_MAX_INTEGRAL};

// FUNCTION: Wrap an angle error into [-180, 180] degrees
//   So that, e.g., a target of 5 deg with a measured heading of 355 deg
//   yields an error of +10 deg (turn the short way) rather than -350.
float wrapDeg180(float e)
{
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

// FUNCTION: Run the heading PID once and return the commanded omega (rad/s)
//   error = wrap(headTarget - imuHeading). Inside the deadband the
//   proportional / output terms are suppressed and the integrator is
//   FROZEN (not zeroed) so a legitimate steady-state bias is preserved.
//   Output is clamped to MAX_ANGULAR_VEL to match the DRIVE omega range.
float computeHeadingOmega(float dt)
{
    float error = wrapDeg180(headTarget - imuHeading);

    if (HEAD_DEADBAND_DEG > 0.0f && fabs(error) < HEAD_DEADBAND_DEG) {
        headHold.prevError = error;
        return 0.0f;
    }

    float P = headHold.Kp * error;

    headHold.integral += error * dt;
    if (headHold.integral >  headHold.maxIntegral) headHold.integral =  headHold.maxIntegral;
    if (headHold.integral < -headHold.maxIntegral) headHold.integral = -headHold.maxIntegral;
    float I = headHold.Ki * headHold.integral;

    float derivative = (error - headHold.prevError) / dt;
    headHold.prevError = error;
    float D = headHold.Kd * derivative;

    float omega = P + I + D;
    if (omega >  MAX_ANGULAR_VEL) omega =  MAX_ANGULAR_VEL;
    if (omega < -MAX_ANGULAR_VEL) omega = -MAX_ANGULAR_VEL;
    return omega;
}
#endif

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// FUNCTION: LUT Interpolation (Reads from Flash Memory)
int getFeedforwardPWM(float targetRPM, const float* motorRpmMap)
{
    float firstRPM = pgm_read_float(&motorRpmMap[0]);
    float lastRPM  = pgm_read_float(&motorRpmMap[MAP_SIZE - 1]);

    if (targetRPM <= firstRPM) return pgm_read_word(&pwmMap[0]);
    if (targetRPM >= lastRPM)  return pgm_read_word(&pwmMap[MAP_SIZE - 1]);

    for (int i = 0; i < MAP_SIZE - 1; i++) {
        float r1 = pgm_read_float(&motorRpmMap[i]);
        float r2 = pgm_read_float(&motorRpmMap[i+1]);

        if (targetRPM >= r1 && targetRPM <= r2) {
            if (r1 == r2) return pgm_read_word(&pwmMap[i]);

            int p1 = (int)pgm_read_word(&pwmMap[i]);
            int p2 = (int)pgm_read_word(&pwmMap[i+1]);

            float fraction = (targetRPM - r1) / (r2 - r1);
            return p1 + (int)(fraction * (p2 - p1));
        }
    }
    return 0;
}

// FUNCTION: Calculate PID + Feedforward LUT
int computePID(PID &p, float currentRPM, float dt, const float* motorRpmMap)
{
    float error = p.setpoint - currentRPM;
    float P = p.Kp * error;

    p.integral += error * dt;
    if (p.integral > p.maxIntegral) p.integral = p.maxIntegral;
    if (p.integral < -p.maxIntegral) p.integral = -p.maxIntegral;
    float I = p.Ki * p.integral;

    float derivative = (error - p.prevError) / dt;
    p.prevError = error;
    float D = p.Kd * derivative;

    float F = getFeedforwardPWM(p.setpoint, motorRpmMap);
    float output = P + I + D + F;

    if (output > p.maxOutput) output = p.maxOutput;
    if (output < -p.maxOutput) output = -p.maxOutput;

    if (p.setpoint == 0.0) {
        output = 0;
        p.integral = 0;
    }

    return (int)output;
}

// FUNCTION: Send Exact Sabertooth Commands
//   Explicit wheel -> (serial port, Sabertooth channel) mapping.
//   IK / PID order is: m1 = FL, m2 = FR, m3 = RL, m4 = RR.
//
//   Front Sabertooth is on Serial3, rear Sabertooth is on Serial2.
//   The channel assignments below preserve the original wiring:
//     FL (m1) -> Serial3 channel M2
//     FR (m2) -> Serial3 channel M1
//     RL (m3) -> Serial2 channel M1
//     RR (m4) -> Serial2 channel M2
//   If a wheel spins the wrong way or the wrong wheel responds,
//   change ONLY the channel label on that one line below.
void setMotorSpeeds(int m1, int m2, int m3, int m4) {
    Serial3.print("M2: "); Serial3.print(MOTOR1_DIR * m1); Serial3.print("\r\n");
    Serial3.print("M1: "); Serial3.print(MOTOR2_DIR * m2); Serial3.print("\r\n");
    Serial2.print("M2: "); Serial2.print(MOTOR3_DIR * m3); Serial2.print("\r\n");
    Serial2.print("M1: "); Serial2.print(MOTOR4_DIR * m4); Serial2.print("\r\n");
}

// FUNCTION: Non-Blocking Serial Reader & Parser (Dual Mode)
// Drains the entire serial RX hardware buffer in every loop tick.
// If multiple lines are queued (e.g. from continuous /cmd_vel streams),
// it processes through all of them and leaves receivedChars containing
// ONLY the latest line.
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    while (Serial.available() > 0) {
        rc = Serial.read();
        if (rc != endMarker && rc != '\r') {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= (int)numChars) { ndx = numChars - 1; }
        }
        else if (rc == endMarker) {
            receivedChars[ndx] = '\0';
            ndx = 0;
            newData = true; // Flag that a fresh complete line is ready
        }
    }
}

// FUNCTION: Mecanum Inverse Kinematics
void mecanumIK(float vy, float vx, float omega)
{
    if (vy >  MAX_LINEAR_VEL)  vy =  MAX_LINEAR_VEL;
    if (vy < -MAX_LINEAR_VEL)  vy = -MAX_LINEAR_VEL;
    if (vx >  MAX_LINEAR_VEL)  vx =  MAX_LINEAR_VEL;
    if (vx < -MAX_LINEAR_VEL)  vx = -MAX_LINEAR_VEL;
    if (omega >  MAX_ANGULAR_VEL) omega =  MAX_ANGULAR_VEL;
    if (omega < -MAX_ANGULAR_VEL) omega = -MAX_ANGULAR_VEL;

    float r = omega * KIN_LW;

    float wFL = vy + vx + r;
    float wFR = vy - vx + r;
    float wRL = vy - vx - r;
    float wRR = vy + vx - r;

    float toRPM = 60.0f / (2.0f * PI * WHEEL_RADIUS);

    pid1.setpoint = -wFL * toRPM;
    pid2.setpoint = wFR * toRPM;
    pid3.setpoint = -wRL * toRPM;
    pid4.setpoint = wRR * toRPM;
}

void parseData() {
    if (newData == true) {
        char * strtokIndx;
        strtokIndx = strtok(receivedChars, ",");

        if (strtokIndx == NULL) {
            newData = false;
            return;
        }

        if (strcmp(strtokIndx, "RAW") == 0) {
            usePID = false;
#if BNO055_ENABLE
            headHoldActive = false;   // explicit mode switch cancels heading-hold
#endif
            rawCmd1 = atoi(strtok(NULL, ","));
            rawCmd2 = atoi(strtok(NULL, ","));
            rawCmd3 = atoi(strtok(NULL, ","));
            rawCmd4 = atoi(strtok(NULL, ","));
        }

        else if (strcmp(strtokIndx, "PID") == 0) {
            usePID = true;
#if BNO055_ENABLE
            headHoldActive = false;   // explicit mode switch cancels heading-hold
#endif
            pid1.setpoint = atof(strtok(NULL, ","));
            pid2.setpoint = atof(strtok(NULL, ","));
            pid3.setpoint = atof(strtok(NULL, ","));
            pid4.setpoint = atof(strtok(NULL, ","));
        }

        else if (strcmp(strtokIndx, "DRIVE") == 0) {
            usePID = true;
#if BNO055_ENABLE
            headHoldActive = false;   // plain DRIVE takes over from heading-hold
#endif
            float vy    = atof(strtok(NULL, ","));
            float vx    = atof(strtok(NULL, ","));
            float omega = atof(strtok(NULL, ","));
            mecanumIK(vy, vx, omega);
        }

#if BNO055_ENABLE
        // HDRIVE,vy,vx,heading - heading-hold drive. vy / vx are the same
        // linear inputs as DRIVE; the third field is the TARGET absolute
        // heading in degrees (BNO055 frame, 0..360). The omega term is NOT
        // taken from the host -- it is generated by the heading PID every
        // control tick from the live fused yaw. This both steers the bot
        // to the target heading and actively holds it against drift, so the
        // bot will not slowly slip into other angles while translating.
        // A missing heading field (atof -> 0) simply targets 0 deg.
        else if (strcmp(strtokIndx, "HDRIVE") == 0) {
            usePID = true;
            hdVy = atof(strtok(NULL, ","));
            hdVx = atof(strtok(NULL, ","));
            float newTarget = atof(strtok(NULL, ","));
            // Only reset PID state on a large target jump (>5 deg).
            // Resetting every packet at 20 Hz caused derivative spikes and oscillation.
            if (!headHoldActive || fabs(wrapDeg180(newTarget - headTarget)) > 5.0f) {
                headHold.integral  = 0;
                headHold.prevError = 0;
            }
            headTarget     = newTarget;
            headHoldActive = true;
            Serial.print(F("ACK_HDRIVE,")); Serial.println(headTarget, 1);
        }

        // HHOLD,SET - latch the CURRENT heading as the hold target, keeping
        //   the last HDRIVE linear command (hdVx/hdVy). Lets you grab
        //   "whatever I am pointing at right now" without first reading the
        //   exact heading off telemetry.
        // HHOLD,OFF - disable heading-hold; the last IK setpoints remain
        //   until the next DRIVE / HDRIVE / PID / RAW command.
        else if (strcmp(strtokIndx, "HHOLD") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "SET") == 0) {
                headTarget = imuHeading;
                headHoldActive = true;
                headHold.integral = 0;
                headHold.prevError = 0;
                Serial.print(F("ACK_HHOLD_SET,")); Serial.println(headTarget, 1);
            } else if (sub != NULL && strcmp(sub, "OFF") == 0) {
                headHoldActive = false;
                Serial.println(F("ACK_HHOLD_OFF"));
            } else {
                Serial.println(F("NAK_HHOLD"));
            }
        }
#endif

        // LIFT,UP / LIFT,DOWN / LIFT,STOP - drive BOTH drive-pair lifters
        // together with the hardcoded per-side PWM / DIR values from
        // Config.h. LIFT,STOP is the default state. Unknown sub-commands
        // are rejected with a NAK and leave the lifters untouched.
        else if (strcmp(strtokIndx, "LIFT") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "UP") == 0) {
                lifterUp();
                Serial.println(F("ACK_LIFT_UP"));
            } else if (sub != NULL && strcmp(sub, "DOWN") == 0) {
                lifterDown();
                Serial.println(F("ACK_LIFT_DOWN"));
            } else if (sub != NULL && strcmp(sub, "STOP") == 0) {
                lifterStop();
                Serial.println(F("ACK_LIFT_STOP"));
            } else {
                Serial.println(F("NAK_LIFT"));
            }
        }

        // RLIFT,UP / RLIFT,DOWN / RLIFT,STOP - drive the RIGHT lifter only,
        // same command style as LIFT. Unknown sub-commands are NAK'd.
        else if (strcmp(strtokIndx, "RLIFT") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "UP") == 0) {
                rightLifterUp();
                Serial.println(F("ACK_RLIFT_UP"));
            } else if (sub != NULL && strcmp(sub, "DOWN") == 0) {
                rightLifterDown();
                Serial.println(F("ACK_RLIFT_DOWN"));
            } else if (sub != NULL && strcmp(sub, "STOP") == 0) {
                rightLifterStop();
                Serial.println(F("ACK_RLIFT_STOP"));
            } else {
                Serial.println(F("NAK_RLIFT"));
            }
        }

        // LLIFT,UP / LLIFT,DOWN / LLIFT,STOP - drive the LEFT lifter only,
        // same command style as LIFT. Unknown sub-commands are NAK'd.
        else if (strcmp(strtokIndx, "LLIFT") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "UP") == 0) {
                leftLifterUp();
                Serial.println(F("ACK_LLIFT_UP"));
            } else if (sub != NULL && strcmp(sub, "DOWN") == 0) {
                leftLifterDown();
                Serial.println(F("ACK_LLIFT_DOWN"));
            } else if (sub != NULL && strcmp(sub, "STOP") == 0) {
                leftLifterStop();
                Serial.println(F("ACK_LLIFT_STOP"));
            } else {
                Serial.println(F("NAK_LLIFT"));
            }
        }

        // KFSLIFT,UP / KFSLIFT,DOWN / KFSLIFT,STOP - drive the KFS lifter
        // (ARM_LIFTER Cytron channel) only, same command style as LIFT.
        // Independent of the LIFT drive pair. Unknown sub-commands are NAK'd.
        else if (strcmp(strtokIndx, "KFSLIFT") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "UP") == 0) {
                kfsLifterUp();
                Serial.println(F("ACK_KFSLIFT_UP"));
            } else if (sub != NULL && strcmp(sub, "DOWN") == 0) {
                kfsLifterDown();
                Serial.println(F("ACK_KFSLIFT_DOWN"));
            } else if (sub != NULL && strcmp(sub, "STOP") == 0) {
                kfsLifterStop();
                Serial.println(F("ACK_KFSLIFT_STOP"));
            } else {
                Serial.println(F("NAK_KFSLIFT"));
            }
        }

        // GRIPPER,OPEN / GRIPPER,CLOSE - drive the staff gripper servo to
        // the hardcoded open / close angles from Config.h. Unknown
        // sub-commands are rejected with a NAK.
        else if (strcmp(strtokIndx, "GRIPPER") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "OPEN") == 0) {
                gripperOpen();
                Serial.println(F("ACK_GRIPPER_OPEN"));
            } else if (sub != NULL && strcmp(sub, "CLOSE") == 0) {
                gripperClose();
                Serial.println(F("ACK_GRIPPER_CLOSE"));
            } else {
                Serial.println(F("NAK_GRIPPER"));
            }
        }

        // KFSGRIP,OPEN / KFSGRIP,CLOSE - drive the KFS gripper servo to
        // the hardcoded open / close angles from Config.h. Independent of
        // the staff gripper. Unknown sub-commands are rejected with a NAK.
        else if (strcmp(strtokIndx, "KFSGRIP") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "OPEN") == 0) {
                kfsGripperOpen();
                Serial.println(F("ACK_KFSGRIP_OPEN"));
            } else if (sub != NULL && strcmp(sub, "CLOSE") == 0) {
                kfsGripperClose();
                Serial.println(F("ACK_KFSGRIP_CLOSE"));
            } else {
                Serial.println(F("NAK_KFSGRIP"));
            }
        }

        // ROTATE,angle - drive the staff rotate servo (STAFF_ROTATE pin) to
        // an arbitrary target angle in degrees. Like every other servo
        // command this only sets the target; the servo SLEWS there at
        // STAFF_ROTATE_SLEW_DPS. Angle is clamped to [SERVO_MIN_ANGLE,
        // SERVO_MAX_ANGLE] inside setServoTarget(). Missing angle is NAK'd.
        else if (strcmp(strtokIndx, "ROTATE") == 0) {
            char * angStr = strtok(NULL, ",");
            if (angStr != NULL) {
                int ang = atoi(angStr);
                setServoTarget(SRV_STAFF_ROTATE, (float)ang);
                Serial.print(F("ACK_ROTATE,")); Serial.println(ang);
            } else {
                Serial.println(F("NAK_ROTATE"));
            }
        }

        // ARM,STOW / ARM,MID / ARM,EXTEND - move the whole KFS arm
        // (base + mid + front servos together) to one of three hardcoded
        // poses from Config.h. Unknown sub-commands are rejected with a NAK.
        else if (strcmp(strtokIndx, "ARM") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "STOW") == 0) {
                armStow();
                Serial.println(F("ACK_ARM_STOW"));
            } else if (sub != NULL && strcmp(sub, "MID") == 0) {
                armMid();
                Serial.println(F("ACK_ARM_MID"));
            } else if (sub != NULL && strcmp(sub, "EXTEND") == 0) {
                armExtend();
                Serial.println(F("ACK_ARM_EXTEND"));
            } else {
                Serial.println(F("NAK_ARM"));
            }
        }

        // SERVO,id,angle - move ONE servo to an arbitrary target angle.
        //   id:  0 = arm base, 1 = arm mid, 2 = arm front,
        //        3 = staff gripper, 4 = KFS gripper
        //   angle: target in degrees, clamped to [SERVO_MIN_ANGLE,
        //          SERVO_MAX_ANGLE] in setServoTarget().
        // Like every other servo command this only sets the target; the
        // servo SLEWS there at its configured deg/sec. Moving an arm joint
        // individually marks armState MANUAL (-1) so telemetry shows the
        // arm is no longer in a named pose. Out-of-range ids are NAK'd.
        else if (strcmp(strtokIndx, "SERVO") == 0) {
            char * idStr  = strtok(NULL, ",");
            char * angStr = strtok(NULL, ",");
            if (idStr != NULL && angStr != NULL) {
                int id  = atoi(idStr);
                int ang = atoi(angStr);
                if (id >= 0 && id < SRV_COUNT) {
                    setServoTarget((uint8_t)id, (float)ang);
                    // Reflect the logical state caches so telemetry stays honest
                    if (id == SRV_ARM_BASE || id == SRV_ARM_MID || id == SRV_ARM_FRONT) {
                        armState = -1;   // MANUAL: no longer a named pose
                    } else if (id == SRV_STAFF_GRIP) {
                        gripperState = (ang >= STAFF_GRIPPER_CLOSE_ANGLE) ? 0 : 1;
                    } else if (id == SRV_KFS_GRIP) {
                        kfsGripperState = (ang >= KFS_GRIPPER_CLOSE_ANGLE) ? 0 : 1;
                    }
                    Serial.print(F("ACK_SERVO,")); Serial.print(id);
                    Serial.print(F(",")); Serial.println(ang);
                } else {
                    Serial.println(F("NAK_SERVO"));
                }
            } else {
                Serial.println(F("NAK_SERVO"));
            }
        }

        else if (strcmp(strtokIndx, "GAIN") == 0) {
            float kp = atof(strtok(NULL, ","));
            float ki = atof(strtok(NULL, ","));
            float kd = atof(strtok(NULL, ","));
            pid1.Kp = kp; pid1.Ki = ki; pid1.Kd = kd;
            pid2.Kp = kp; pid2.Ki = ki; pid2.Kd = kd;
            pid3.Kp = kp; pid3.Ki = ki; pid3.Kd = kd;
            pid4.Kp = kp; pid4.Ki = ki; pid4.Kd = kd;
            pid1.integral = 0; pid1.prevError = 0;
            pid2.integral = 0; pid2.prevError = 0;
            pid3.integral = 0; pid3.prevError = 0;
            pid4.integral = 0; pid4.prevError = 0;
            Serial.println(F("ACK_GAIN"));
        }

        // GAINW,wheel,kp,ki,kd - set gains for ONE wheel (1..4).
        // Lets a per-wheel tuner push individual gains without
        // disturbing the other three. Resets that wheel's integrator
        // and derivative memory so the new gains take effect cleanly.
        // Out-of-range wheel indices are rejected with a NAK.
        else if (strcmp(strtokIndx, "GAINW") == 0) {
            int w = atoi(strtok(NULL, ","));
            if (w >= 1 && w <= 4) {
                float kp = atof(strtok(NULL, ","));
                float ki = atof(strtok(NULL, ","));
                float kd = atof(strtok(NULL, ","));
                PID* pp = pidByWheel[w - 1];
                pp->Kp = kp; pp->Ki = ki; pp->Kd = kd;
                pp->integral = 0; pp->prevError = 0;
                Serial.print(F("ACK_GAINW,")); Serial.println(w);
            } else {
                Serial.println(F("NAK_GAINW"));
            }
        }

#if BNO055_ENABLE
        // HGAIN,kp,ki,kd - live-tune the heading-hold PID gains without a
        // reflash. Resets the heading integrator / derivative memory so the
        // new gains take effect cleanly. Handy for dialing HEAD_KP etc. on
        // the actual surface before baking them back into Config.h.
        else if (strcmp(strtokIndx, "HGAIN") == 0) {
            float kp = atof(strtok(NULL, ","));
            float ki = atof(strtok(NULL, ","));
            float kd = atof(strtok(NULL, ","));
            headHold.Kp = kp; headHold.Ki = ki; headHold.Kd = kd;
            headHold.integral = 0; headHold.prevError = 0;
            Serial.println(F("ACK_HGAIN"));
        }

        // IMU,RESET zeroes the absolute heading reference by forcing a
        // fresh re-init on the next IMU poll. Useful before a run so the
        // current facing reads as the working zero. Also clears any active
        // heading-hold so the loop does not chase a stale target across the
        // reference jump.
        else if (strcmp(strtokIndx, "IMU") == 0) {
            char * sub = strtok(NULL, ",");
            if (sub != NULL && strcmp(sub, "RESET") == 0) {
                imuReady = false;
                headHoldActive = false;
                Serial.println(F("ACK_IMU_RESET"));
            }
        }
#endif

        newData = false;
    }
}

// FUNCTION: Main Control Loop
void updateControlLoop()
{
    unsigned long now = millis();
    if (now - lastLoopTime >= DT_MS) {
        float dt = (now - lastLoopTime) / 1000.0f;
        lastLoopTime = now;

#if BNO055_ENABLE
        // Heading-hold: regenerate omega from the current fused yaw every
        // tick and refresh the wheel setpoints via IK. This runs BEFORE the
        // encoder reads / PID so the wheel PIDs act on the freshest
        // setpoints. Only active while a HDRIVE / HHOLD target is set AND
        // the IMU is actually online; if the sensor drops out we hold the
        // last setpoints rather than command a correction from a stale
        // heading. Running this every DT_MS (not just on command receipt)
        // is what continuously cancels drift / slip.
        if (headHoldActive && imuReady) {
            float omega = computeHeadingOmega(dt);
            mecanumIK(hdVy, hdVx, omega);
        }
#endif

        long count1 = enc1.read();
        long count2 = enc2.read();
        long count3 = readEnc3();
        long count4 = readEnc4();

        rpm1 = ((count1 - lastCount1) / (float)PPR1) * (60.0f / dt);
        rpm2 = ((count2 - lastCount2) / (float)PPR2) * (60.0f / dt);
        rpm3 = ((count3 - lastCount3) / (float)PPR3) * (60.0f / dt);
        rpm4 = ((count4 - lastCount4) / (float)PPR4) * (60.0f / dt);

        lastCount1 = count1;
        lastCount2 = count2;
        lastCount3 = count3;
        lastCount4 = count4;

        if (usePID) {
            outCmd1 = computePID(pid1, rpm1, dt, rpmMap1);
            outCmd2 = computePID(pid2, rpm2, dt, rpmMap2);
            outCmd3 = computePID(pid3, rpm3, dt, rpmMap3);
            outCmd4 = computePID(pid4, rpm4, dt, rpmMap4);
            setMotorSpeeds(outCmd1, outCmd2, outCmd3, outCmd4);
        } else {
            outCmd1 = rawCmd1;
            outCmd2 = rawCmd2;
            outCmd3 = rawCmd3;
            outCmd4 = rawCmd4;
            setMotorSpeeds(rawCmd1, rawCmd2, rawCmd3, rawCmd4);
        }
    }
}

// FUNCTION: Append IMU fields to the current telemetry frame
//   Emitted as a trailing comma-separated block so the existing W*
//   fields keep their positions and parsers can ignore the IMU tail
//   if they do not know it. When the sensor is offline, IMU_OK:0 is
//   sent and the numeric fields are omitted to keep the line short.
#if BNO055_ENABLE
void appendIMUTelemetry() {

    Serial.print(F(","));
    Serial.print(F("IMU_OK:")); Serial.print(imuReady ? 1 : 0);
    if (!imuReady) return;

    Serial.print(F(","));
    Serial.print(F("HDG:"));   Serial.print(imuHeading, 2); Serial.print(F(","));
    Serial.print(F("ROLL:"));  Serial.print(imuRoll, 2);    Serial.print(F(","));
    Serial.print(F("PITCH:")); Serial.print(imuPitch, 2);   Serial.print(F(","));
    Serial.print(F("GX:"));    Serial.print(imuGx, 2);      Serial.print(F(","));
    Serial.print(F("GY:"));    Serial.print(imuGy, 2);      Serial.print(F(","));
    Serial.print(F("GZ:"));    Serial.print(imuGz, 2);      Serial.print(F(","));
    Serial.print(F("AX:"));    Serial.print(imuAx, 2);      Serial.print(F(","));
    Serial.print(F("AY:"));    Serial.print(imuAy, 2);      Serial.print(F(","));
    Serial.print(F("AZ:"));    Serial.print(imuAz, 2);      Serial.print(F(","));
    Serial.print(F("CAL:"));
    Serial.print(calSys);   Serial.print(calGyro);
    Serial.print(calAccel); Serial.print(calMag);
}
#endif

// FUNCTION: Send Telemetry
//   T and C1..C4 are sampled back-to-back so the host can difference the
//   counts against the Mega clock instead of its own serial arrival time.
//   Counts (not RPM) are the odometry source; the W* fields are PID debug.
void sendTelemetry()
{
    Serial.print(F("T:"));  Serial.print(millis());    Serial.print(F(","));
    Serial.print(F("C1:")); Serial.print(enc1.read()); Serial.print(F(","));
    Serial.print(F("C2:")); Serial.print(enc2.read()); Serial.print(F(","));
    Serial.print(F("C3:")); Serial.print(readEnc3());  Serial.print(F(","));
    Serial.print(F("C4:")); Serial.print(readEnc4());  Serial.print(F(","));

    Serial.print(F("W1_SP:"));  Serial.print(pid1.setpoint, 1); Serial.print(F(","));
    Serial.print(F("W1_RPM:")); Serial.print(rpm1, 2);          Serial.print(F(","));

    Serial.print(F("W2_SP:"));  Serial.print(pid2.setpoint, 1); Serial.print(F(","));
    Serial.print(F("W2_RPM:")); Serial.print(rpm2, 2);          Serial.print(F(","));

    Serial.print(F("W3_SP:"));  Serial.print(pid3.setpoint, 1); Serial.print(F(","));
    Serial.print(F("W3_RPM:")); Serial.print(rpm3, 2);          Serial.print(F(","));

    Serial.print(F("W4_SP:"));  Serial.print(pid4.setpoint, 1); Serial.print(F(","));
    Serial.print(F("W4_RPM:")); Serial.print(rpm4, 2);

#if BNO055_ENABLE
    // Heading-hold state tail: HHOLD (1=active, 0=off) and the current
    // target heading in degrees, emitted just before the IMU block so a
    // host can correlate target vs measured HDG in one frame.
    Serial.print(F(",HHOLD:"));   Serial.print(headHoldActive ? 1 : 0);
    Serial.print(F(",HTGT:"));    Serial.print(headTarget, 1);
    appendIMUTelemetry();
#endif

    Serial.println();
}

void setup()
{
    Serial.begin(BAUD_HOST);
    Serial2.begin(BAUD_SABERTOOTH);
    Serial3.begin(BAUD_SABERTOOTH);

    setMotorSpeeds(0, 0, 0, 0);
    setupPinChangeEncoders();
    setupLifters();
    setupServos();

#if BNO055_ENABLE
    Wire.begin();
    if (tryInitIMU()) {
        Serial.println(F("BNO055 online"));
    } else {
        Serial.println(F("BNO055 not detected (will retry)"));
    }
#endif


    lastLoopTime = millis();

    Serial.println(F("=== DUAL-MODE CONTROLLER READY ==="));
    Serial.println(F("Send: RAW,pwm1,pwm2,pwm3,pwm4"));
    Serial.println(F("Send: PID,rpm1,rpm2,rpm3,rpm4"));
    Serial.println(F("Send: DRIVE,vy,vx,omega"));
#if BNO055_ENABLE
    Serial.println(F("Send: HDRIVE,vy,vx,heading_deg"));
    Serial.println(F("Send: HHOLD,SET | HHOLD,OFF"));
    Serial.println(F("Send: HGAIN,kp,ki,kd"));
#endif
    Serial.println(F("Send: LIFT,UP | LIFT,DOWN | LIFT,STOP"));
    Serial.println(F("Send: RLIFT,UP | RLIFT,DOWN | RLIFT,STOP"));
    Serial.println(F("Send: LLIFT,UP | LLIFT,DOWN | LLIFT,STOP"));
    Serial.println(F("Send: KFSLIFT,UP | KFSLIFT,DOWN | KFSLIFT,STOP"));
    Serial.println(F("Send: GRIPPER,OPEN | GRIPPER,CLOSE"));
    Serial.println(F("Send: KFSGRIP,OPEN | KFSGRIP,CLOSE"));
    Serial.println(F("Send: ROTATE,angle  (staff rotate servo, STAFF_ROTATE pin)"));
    Serial.println(F("Send: ARM,STOW | ARM,MID | ARM,EXTEND"));
    Serial.println(F("Send: SERVO,id,angle  (id 0=base 1=mid 2=front 3=staffgrip 4=kfsgrip 5=staffrotate)"));
    Serial.println(F("Send: GAIN,kp,ki,kd"));
    Serial.println(F("Send: GAINW,wheel,kp,ki,kd"));
#if BNO055_ENABLE
    Serial.println(F("Send: IMU,RESET"));
#endif
}

void loop()
{
    recvWithEndMarker();
    parseData();

    updateControlLoop();

#if BNO055_ENABLE
    updateIMU();
#endif

    // Servo slew stepper: ramp every servo toward its target at the
    // configured per-servo deg/sec. Runs every loop pass with the real
    // elapsed time since the last pass, so motion is smooth regardless
    // of loop jitter. Independent of the DT_MS control tick.
    static unsigned long lastServo = 0;
    unsigned long nowServo = millis();
    unsigned long dtServo = nowServo - lastServo;
    if (dtServo > 0) {
        lastServo = nowServo;
        updateServos(dtServo);
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= TELEMETRY_MS) {
        lastPrint = millis();
        sendTelemetry();
    }
}
