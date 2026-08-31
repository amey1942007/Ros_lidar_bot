#ifndef CONFIG_H
#define CONFIG_H

#include <avr/pgmspace.h>

// ==============================================================
// LOOP TIMING
//   Telemetry is intentionally slower than the control loop so
//   that the serial TX burst in sendTelemetry() cannot stall the
//   control tick and corrupt the measured dt (which feeds RPM
//   and the PID derivative term).
// ==============================================================
const unsigned long DT_MS         = 50;    // control loop period (ms)
const unsigned long TELEMETRY_MS  = 100;   // telemetry print period (ms)

// ==============================================================
// ENCODER COUNTS PER REVOLUTION (at the wheel)
// ==============================================================
const int PPR1 = 1300;
const int PPR2 = 680;
const int PPR3 = 400;
const int PPR4 = 280;

#define MOTOR1_DIR (1)
#define MOTOR2_DIR (-1)
#define MOTOR3_DIR (-1)
#define MOTOR4_DIR (-1)

// ==============================================================
// ENCODER PINS
// ==============================================================
// Encoder 1 (Standard Hardware Interrupts)
#define ENC1_A 2
#define ENC1_B 3

// Encoder 2 (Standard Hardware Interrupts)
#define ENC2_A 18
#define ENC2_B 19

// Encoder 3 (Pin Change Interrupts / PCINT)
#define ENC3_A 53
#define ENC3_B 51

// Encoder 4 (Pin Change Interrupts / PCINT)
#define ENC4_A 10
#define ENC4_B 11

// ==============================================================
// CYTRON MOTOR DRIVER PINS  (PWM + DIR per channel)
//   One Cytron channel per mechanism. PWM pins are PWM-capable Mega
//   pins; DIR is any free digital pin. Functionality assigned later.
// ==============================================================
// RIGHT LIFTER
//   NOTE: A9 is NOT PWM-capable on the Mega 2560, so analogWrite()
//   here is effectively on/off (>=128 -> HIGH). Acceptable for a
//   fixed-speed lifter; real duty-cycle control would need a PWM pin
//   (2-13 or 44-46).
#define RIGHT_LIFTER_PWM   5
#define RIGHT_LIFTER_DIR   37

// LEFT LIFTER
//   NOTE: A8 is NOT PWM-capable (on/off only, same as above).
#define LEFT_LIFTER_PWM    6
#define LEFT_LIFTER_DIR    35

// ARM LIFTER  (used as the KFS LIFTER)
//   NOTE: A10 is NOT PWM-capable (on/off only, same as above).
#define ARM_LIFTER_PWM     4
#define ARM_LIFTER_DIR     33

// ==============================================================
// SERVO SIGNAL PINS  (one signal pin per servo)
//   Functionality assigned later.
// ==============================================================
#define KFS_ARM_BASE_PIN    A7
#define KFS_ARM_MID_PIN     A6
#define KFS_ARM_FRONT_PIN   A10
#define KFS_GRIPPER_PIN     A5
#define STAFF_GRIPPER_PIN   A4
#define STAFF_ROTATE A3// to rotate staff gripper

// STAFF_GRIPPER_PWM: extra pin recA4ded per request. The staff gripper
// is driven as a SERVO on STAFF_GRIPPER_PIN (13), so this pin is not
// used by the gripper command below. Kept here for wiring reference;
// A11 is an analog-capable pin, NOT a PWM pin.
#define STAFF_GRIPPER_PWM   A13

// ============================================================
// STAFF GRIPPER SERVO ANGLES  (hardcoded open / close positions)
//   GRIPPER,OPEN  writes STAFF_GRIPPER_OPEN_ANGLE
//   GRIPPER,CLOSE writes STAFF_GRIPPER_CLOSE_ANGLE
//   to the servo on STAFF_GRIPPER_PIN. Adjust to taste.
// ============================================================
#define STAFF_GRIPPER_OPEN_ANGLE    180
#define STAFF_GRIPPER_CLOSE_ANGLE   0

// ============================================================
// KFS GRIPPER SERVO ANGLES  (hardcoded open / close positions)
//   KFSGRIP,OPEN  writes KFS_GRIPPER_OPEN_ANGLE
//   KFSGRIP,CLOSE writes KFS_GRIPPER_CLOSE_ANGLE
//   to the servo on KFS_GRIPPER_PIN. Adjust to taste.
// ============================================================
#define KFS_GRIPPER_OPEN_ANGLE    0
#define KFS_GRIPPER_CLOSE_ANGLE   180

// ============================================================
// KFS ARM POSES  (whole-arm hardcoded states)
//   The KFS arm is three servos -- base / mid / front -- driven
//   together as a single mechanism. It has THREE fixed poses; each
//   pose sets all three servos simultaneously to the angles below.
//
//   ARM,STOW   -> *_STOW_ANGLE   (arm tucked / parked)
//   ARM,MID    -> *_MID_ANGLE    (intermediate / transit pose)
//   ARM,EXTEND -> *_EXTEND_ANGLE (arm reached out / working pose)
//
//   Each row is one servo; each column is one pose. Tune the angles
//   per servo so the three poses match the real mechanism. Boots in
//   the STOW pose.
// ============================================================
// BASE servo angles per pose
#define KFS_ARM_BASE_STOW_ANGLE     50
#define KFS_ARM_BASE_MID_ANGLE      130
#define KFS_ARM_BASE_EXTEND_ANGLE   150

// MID servo angles per pose
#define KFS_ARM_MID_STOW_ANGLE      50
#define KFS_ARM_MID_MID_ANGLE       130
#define KFS_ARM_MID_EXTEND_ANGLE    150

// FRONT servo angles per pose
#define KFS_ARM_FRONT_STOW_ANGLE    50
#define KFS_ARM_FRONT_MID_ANGLE     130
#define KFS_ARM_FRONT_EXTEND_ANGLE  150

// ============================================================
// SERVO SLEW RATES  (per servo, degrees per second)
//   EVERY servo motion -- pose presets, gripper open/close, AND the
//   individual SERVO command -- is rate-limited so the servo ramps
//   smoothly from its current angle to the new target instead of
//   snapping. A command only sets the TARGET; the non-blocking
//   updateServos() stepper walks the current angle toward it at the
//   rate below. Higher = faster, lower = gentler. Tune per servo to
//   match each joint's load / linkage.
//
//   The slew step per control tick is (rate_deg_s * DT_MS / 1000),
//   so the effective resolution is one step every DT_MS. A rate of 0
//   would freeze the servo, so keep these > 0.
// ============================================================
#define KFS_ARM_BASE_SLEW_DPS     30.0    // deg/sec, base servo
#define KFS_ARM_MID_SLEW_DPS      30.0    // deg/sec, mid servo
#define KFS_ARM_FRONT_SLEW_DPS    30.0    // deg/sec, front servo
#define STAFF_GRIPPER_SLEW_DPS    90.0   // deg/sec, staff gripper
#define KFS_GRIPPER_SLEW_DPS      90.0   // deg/sec, KFS gripper
#define STAFF_ROTATE_SLEW_DPS     90.0   // deg/sec, staff rotate servo

// ============================================================
// STAFF ROTATE SERVO  (servo on STAFF_ROTATE pin, arbitrary target angle)
//   ROTATE,angle drives this servo to any target angle (clamped to
//   [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE] below). Boots at the angle below.
// ============================================================
#define STAFF_ROTATE_BOOT_ANGLE   90

// ============================================================
// SERVO ANGLE LIMITS  (clamp for the individual SERVO command)
//   The SERVO command accepts an arbitrary target angle; it is
//   clamped to [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE] before becoming a
//   target so a bad host value cannot drive a joint past its safe
//   travel. The preset pose / gripper angles above are assumed to
//   already be within this range.
// ============================================================
#define SERVO_MIN_ANGLE    40
#define SERVO_MAX_ANGLE    150

// ==============================================================
// PCINT ENCODER BIT MAPPING  (enc3, enc4 on PORTB)
//   The hand-written PCINT0 ISR reads PORTB directly, so it needs
//   the PORTB bit position for each pin, not the Arduino pin number.
//   This is the SINGLE place to change wheel-to-encoder assignment
//   and counting direction for the two pin-change encoders.
//
//   Mega 2560 PORTB pin -> bit:
//     pin 53 = PB0   pin 52 = PB1   pin 51 = PB2   pin 50 = PB3
//     pin 10 = PB4   pin 11 = PB5   pin 12 = PB6   pin 13 = PB7
//
//   TO SWAP WHICH PHYSICAL ENCODER IS W3 vs W4:
//     swap the ENC3_*_BIT block with the ENC4_*_BIT block.
//   TO REVERSE A WHEEL'S COUNT DIRECTION:
//     swap that encoder's A_BIT and B_BIT (or set its INVERT flag).
// ============================================================

// enc3 reads pins 10/11 (PB4/PB5)
#define ENC3_A_BIT PB4
#define ENC3_B_BIT PB5

// enc4 reads pins 51/53 (PB2/PB0)
#define ENC4_A_BIT PB2
#define ENC4_B_BIT PB0

// Direction inversion: set to -1 to flip a wheel's sign, +1 to leave it.
// W4 was reading negative while turning CW, so it is inverted here.
#define ENC3_DIR  (1)
#define ENC4_DIR  (-1)

// ============================================================
// LIFTER HARDCODED DRIVE VALUES  (Cytron: PWM magnitude + DIR level)
//   Lifters can be driven TOGETHER (LIFT command) or INDIVIDUALLY
//   (RLIFT / LLIFT commands). Each side has its OWN hardcoded PWM
//   magnitude so a mechanically heavier / faster side can be matched
//   to the other. PWM is the analogWrite magnitude (0..255) applied
//   to that side's *_LIFTER_PWM pin; DIR is the digitalWrite level
//   (HIGH/LOW) on the *_LIFTER_DIR pin that sets travel direction.
//
//   LIFT,UP    -> both lifters UP   at their *_LIFT_UP_PWM
//   LIFT,DOWN  -> both lifters DOWN at their *_LIFT_DOWN_PWM
//   LIFT,STOP  -> both lifters PWM = 0
//   RLIFT,UP / RLIFT,DOWN / RLIFT,STOP -> right lifter only
//   LLIFT,UP / LLIFT,DOWN / LLIFT,STOP -> left  lifter only
//   KFSLIFT,UP / KFSLIFT,DOWN / KFSLIFT,STOP -> KFS lifter only
//   STOP is the DEFAULT / boot state.
//
//   If a lifter travels the wrong way, flip that side's *_DIR_UP
//   value (and its *_DIR_DOWN, which must be the logical opposite).
// ============================================================
// Right lifter PWM magnitudes
#define RIGHT_LIFT_UP_PWM    150    // analogWrite magnitude, right side up
#define RIGHT_LIFT_DOWN_PWM  150    // analogWrite magnitude, right side down

// Left lifter PWM magnitudes (different from right side)
#define LEFT_LIFT_UP_PWM     150    // analogWrite magnitude, left side up
#define LEFT_LIFT_DOWN_PWM   150    // analogWrite magnitude, left side down

// KFS lifter PWM magnitudes (ARM_LIFTER Cytron channel)
#define KFS_LIFT_UP_PWM      150    // analogWrite magnitude, KFS up
#define KFS_LIFT_DOWN_PWM    150    // analogWrite magnitude, KFS down

// Right lifter direction levels
#define RIGHT_LIFTER_DIR_UP    LOW
#define RIGHT_LIFTER_DIR_DOWN  HIGH

// Left lifter direction levels
#define LEFT_LIFTER_DIR_UP     HIGH
#define LEFT_LIFTER_DIR_DOWN   LOW

// KFS lifter direction levels
#define KFS_LIFTER_DIR_UP      LOW
#define KFS_LIFTER_DIR_DOWN    HIGH

// ============================================================
// SERIAL BAUD RATES
// ============================================================
const long BAUD_HOST       = 115200;
const long BAUD_SABERTOOTH = 9600;

// ============================================================
// CHASSIS GEOMETRY (mecanum)
//   L = wheelbase (front-to-rear axle distance), metres
//   W = track width (left-to-right wheel distance), metres
//   WHEEL_RADIUS = mecanum wheel radius, metres
// ============================================================
const float CHASSIS_L    = 0.52;
const float CHASSIS_W    = 0.63;
const float WHEEL_RADIUS = 0.05;

// Combined geometry term used in mecanum IK: (L + W) / 2
const float KIN_LW = (CHASSIS_L + CHASSIS_W) / 2.0f;

// ============================================================
// DRIVE COMMAND LIMITS
//   Linear velocities in m/s, angular in rad/s.
//   These clamp the DRIVE command inputs before IK.
// ============================================================
const float MAX_LINEAR_VEL  = 1.0;   // m/s  (vx, vy)
const float MAX_ANGULAR_VEL = 3.0;   // rad/s (omega)

// ============================================================
// BNO055 IMU  (I2C - Adafruit_BNO055)
//   On the Mega 2560, I2C is fixed at SDA=20, SCL=21. These pins
//   are clear of the encoder, serial, and Sabertooth wiring, so no
//   reassignment is needed. The default I2C address is 0x28 (set
//   to 0x29 if the ADR/COM3 pin is pulled high).
//
//   IMU_SAMPLE_MS lets the IMU be polled on its own cadence. It is
//   read inside the telemetry block, so keeping it <= TELEMETRY_MS
//   means a fresh sample accompanies every telemetry frame. The
//   BNO055 fuses internally at 100 Hz, so polling faster than ~10ms
//   buys nothing.
// ============================================================
#define BNO055_ENABLE     1          // set to 0 to compile IMU out
#define BNO055_ADDRESS    0x28       // 0x29 if ADR pin pulled high
#define BNO055_ID         0x55       // chip id returned over I2C
 
const unsigned long IMU_SAMPLE_MS = 50;   // IMU poll period (ms)

// ============================================================
// PID GAINS  { Kp, Ki, Kd }  and limits
// ============================================================
const float PID1_KP = 0.0,  PID1_KI = 0.25, PID1_KD = 0.00;
const float PID2_KP = 0.5,  PID2_KI = 0.05, PID2_KD = 0.01;
const float PID3_KP = 0.5,  PID3_KI = 0.05, PID3_KD = 0.01;
const float PID4_KP = 0.5,  PID4_KI = 0.05, PID4_KD = 0.01;

const float PID_MAX_OUTPUT   = 2047.0;
const float PID_MAX_INTEGRAL = 500.0;

// ============================================================
// HEADING-HOLD PID  (HDRIVE / HHOLD commands)
//   Closes a loop on the BNO055 fused yaw (imuHeading, deg) to
//   produce the omega term fed into mecanum IK. The loop runs every
//   control tick (DT_MS) so any deviation from the target heading is
//   corrected continuously -- this is what stops the bot from slowly
//   slipping into other angles under wheel scrub / roller slip.
//
//   Heading error is wrapped to [-180, 180] deg, and the output omega
//   is clamped to MAX_ANGULAR_VEL (shared with the DRIVE omega limit
//   above) so IK always sees a consistent angular range.
//
//   TUNING ORDER:
//     1. HEAD_KP   (rad/s per deg of error) -- raise until the bot
//        snaps to heading; back off if it oscillates / hunts.
//     2. HEAD_KD   (rad/s per deg/s) -- add to damp overshoot when
//        slewing to a new target heading.
//     3. HEAD_KI   (rad/s per deg*s) -- keep small; only needed to
//        cancel a persistent one-sided bias (e.g. a wheel that always
//        pulls). Too much causes slow wind-up oscillation.
//
//   HEAD_MAX_INTEGRAL  : anti-windup clamp on the integrator (deg*s).
//   HEAD_DEADBAND_DEG  : inside this band the proportional term is
//                        suppressed and the integrator frozen, so the
//                        bot does not jitter around the target at rest.
//                        Set to 0 to disable the deadband.
//
//   Only compiled when BNO055_ENABLE is set, since heading-hold has no
//   meaning without the IMU.
// ============================================================
#if BNO055_ENABLE
const float HEAD_KP = 0.06;    // rad/s per deg of heading error
const float HEAD_KI = 0;   // rad/s per (deg*s)
const float HEAD_KD = 0.015;   // rad/s per (deg/s)

const float HEAD_MAX_INTEGRAL = 200.0;   // deg*s, anti-windup clamp
const float HEAD_DEADBAND_DEG = 5.0;   // deg, no correction inside this band (was 0.5 — caused hunting)
#endif

// ============================================================
// FEEDFORWARD LOOKUP TABLE SIZE
//   Single zero entry on the command axis (strictly ascending),
//   so the curve has exactly one zero crossing and stays injective.
// ============================================================
const int MAP_SIZE = 83;



// ============================================================
// FEEDFORWARD LOOKUP TABLES  (SURFACE-DEPENDENT - RE-MAP PER SURFACE)
//   pwmMap:  command axis, shared by all wheels (must stay strictly ascending)
//   rpmMapN: measured wheel RPM at each pwmMap entry, one table per wheel.
// ============================================================
const int pwmMap[MAP_SIZE] PROGMEM = {
    -2047, -2000, -1950, -1900, -1850, -1800, -1750, -1700, -1650, -1600, -1550, -1500, -1450, -1400, -1350, -1300, -1250, -1200, -1150, -1100, -1050, -1000, -950, -900, -850, -800, -750, -700, -650, -600, -550, -500, -450, -400, -350, -300, -250, -200, -150, -100, -50, 0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000, 1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950, 2000, 2047
};

const float rpmMap1[MAP_SIZE] PROGMEM = {
    -268.3, -268.4, -267.7, -266.5, -259.3, -254.3, -242.9, -235.5, -229.5, -222.6, -213.6, -205.7, -200.5, -191.7, -185.0, -177.9, -171.7, -162.6, -155.5, -147.7, -139.4, -132.1, -124.7, -117.7, -111.0, -103.4, -96.5, -88.5, -81.9, -74.9, -67.9, -61.3, -54.1, -47.2, -39.6, -32.9, -26.1, -19.7, -13.3, -7.0, 0.0, 0.0, 0.0, 6.8, 13.4, 19.4, 25.4, 32.0, 38.4, 44.9, 52.0, 58.9, 64.8, 71.5, 78.2, 84.4, 91.6, 98.5, 104.7, 111.8, 118.1, 124.7, 131.2, 138.8, 145.8, 152.9, 160.1, 166.4, 173.0, 179.6, 187.9, 193.2, 199.2, 207.5, 214.9, 220.1, 226.1, 235.9, 240.8, 247.0, 248.0, 248.7, 249.1
};;

const float rpmMap2[MAP_SIZE] PROGMEM = {
    -283.4, -282.2, -282.4, -280.6, -272.6, -265.2, -257.2, -248.9, -240.9, -232.9, -224.4, -216.4, -208.6, -200.6, -192.2, -184.0, -176.2, -168.3, -160.0, -150.6, -142.0, -134.3, -126.0, -118.2, -110.8, -102.6, -95.1, -87.5, -79.6, -72.2, -64.6, -56.2, -48.4, -40.1, -31.3, -22.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.4, 31.8, 40.4, 48.5, 56.6, 64.5, 72.2, 80.1, 87.9, 95.4, 103.4, 111.4, 119.0, 126.5, 134.0, 142.6, 150.3, 158.3, 167.2, 175.1, 183.1, 192.0, 200.3, 209.2, 216.8, 224.9, 233.8, 241.8, 250.3, 258.2, 266.3, 274.0, 282.0, 283.6, 283.9, 284.1
};;

const float rpmMap3[MAP_SIZE] PROGMEM = {
    -431.9, -429.5, -430.6, -426.1, -409.9, -398.2, -383.7, -367.7, -353.9, -341.9, -329.1, -317.2, -306.8, -291.7, -280.6, -267.8, -260.0, -249.8, -240.6, -227.6, -216.8, -204.8, -193.3, -180.7, -168.0, -156.4, -144.0, -131.1, -119.4, -107.4, -95.1, -82.5, -69.5, -57.7, -43.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 42.9, 59.3, 73.0, 85.5, 98.1, 110.1, 122.2, 134.5, 146.8, 159.5, 172.0, 184.1, 196.8, 209.2, 219.8, 231.5, 244.2, 253.8, 264.0, 273.4, 283.9, 295.1, 303.2, 318.8, 329.9, 342.0, 351.6, 364.7, 379.1, 393.2, 408.9, 425.8, 430.0, 429.1, 430.5
};;

const float rpmMap4[MAP_SIZE] PROGMEM = {
    -461.6, -462.3, -461.9, -457.9, -445.4, -430.7, -418.5, -405.1, -392.9, -379.7, -365.5, -350.8, -337.9, -324.7, -310.5, -297.8, -283.0, -270.9, -258.3, -245.5, -233.0, -220.2, -206.3, -192.5, -178.8, -163.3, -148.7, -134.5, -120.1, -104.3, -89.9, -76.0, -59.3, -43.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 42.6, 57.9, 72.2, 87.1, 101.9, 119.6, 136.6, 150.8, 164.1, 178.3, 192.0, 206.7, 218.4, 232.5, 245.7, 258.7, 271.5, 285.5, 297.8, 311.0, 325.6, 339.1, 351.4, 364.9, 379.0, 392.9, 404.2, 418.6, 431.7, 447.3, 459.1, 462.4, 462.5, 463.9
};;

#endif
