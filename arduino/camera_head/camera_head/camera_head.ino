/*
 * camera_head.ino — pan/tilt camera-head servo driver + proximity buzzer.
 *
 * Runs on an Arduino Uno wired to the Raspberry Pi over UART. The Pi
 * (camera_servo_node.py) does all the thinking — limits, rate integration,
 * joint states — and sends nothing but raw pulse widths. The Uno's only job is
 * to generate rock-steady 50 Hz pulses, which it does in hardware timers, so
 * the head stays put no matter how hard SLAM/Nav2/YOLO hammer the Pi's cores.
 *
 * WIRING
 *   D6  → pan servo signal  (OT5320M / MG996R class)
 *   D5  → tilt servo signal (SG90)
 *   D9  → piezo buzzer (+) ; buzzer (-) to GND.
 *          D9 is a hardware-PWM pin (Timer 1) so analogWrite gives a true
 *          frequency output, not a software approximation. The buzzer beeps
 *          whenever LiDAR detects an obstacle closer than 45 cm; the duty
 *          cycle fades from 255 (full volume, ≤30 cm) to 0 (silent, ≥45 cm).
 *   Servo V+ → EXTERNAL supply (7.4 V pan, 5 V tilt). NEVER the Uno's 5V pin —
 *   a 20 kg servo's stall current will brown out the board mid-move.
 *   Servo GND, supply GND and Uno GND must all be tied together.
 *
 *   Link to the Pi, pick one:
 *     a) USB cable — simplest. Shows up as /dev/ttyACM* on the Pi.
 *     b) Pi GPIO UART — Uno D0/D1 (Serial). The Uno's TX idles at 5 V and the
 *        Pi's RX is 3.3 V-only, so Uno TX → Pi RX MUST go through a level
 *        shifter (or a 1k/2k divider). Pi TX → Uno RX is fine as-is.
 *        Note: D0/D1 are also the USB-serial pins — unplug the link to reflash.
 *
 * PROTOCOL (115200 8N1, ASCII, one command per line, '\n' terminated)
 *   P<us>   set pan pulse width, e.g. "P1500"
 *   T<us>   set tilt pulse width
 *   P0/T0   detach that servo — goes limp, draws no holding current
 *   B<val>  set buzzer duty 0-255 (0 = off, 255 = full beep on D9)
 *   ?       print the current pulse widths (for a human on the serial monitor)
 *   Anything else is ignored, so line noise cannot move the head.
 *
 * If the Pi goes quiet the servos simply HOLD their last position: for a head
 * carrying a camera against gravity, holding is the safe failure mode.
 */

#include <Servo.h>

const uint8_t PAN_PIN    = 6;
const uint8_t TILT_PIN   = 5;
const uint8_t BUZZER_PIN = 9;   // hardware PWM (Timer 1) — drives piezo directly

// Absolute travel guard. The Pi sends 1000-2000 us by default; anything wider
// than this is either a typo or noise and would drive a servo into its stop,
// where it stalls, buzzes and pulls locked-rotor current.
const int MIN_US = 500;
const int MAX_US = 2500;

Servo panServo, tiltServo;
int panUs = 0, tiltUs = 0;              // 0 = detached

char buf[12];
uint8_t len = 0;
bool skipLine = false;                  // overlong garbage: drop to next '\n'

void apply(Servo &servo, uint8_t pin, int &state, int us) {
  if (us == 0) {
    servo.detach();
    state = 0;
    return;
  }
  us = constrain(us, MIN_US, MAX_US);
  if (!servo.attached()) servo.attach(pin, MIN_US, MAX_US);
  servo.writeMicroseconds(us);
  state = us;
}

void handle(const char *line) {
  switch (line[0]) {
    case 'P': apply(panServo,  PAN_PIN,  panUs,  atoi(line + 1)); break;
    case 'T': apply(tiltServo, TILT_PIN, tiltUs, atoi(line + 1)); break;
    case 'B': {
      // Proximity buzzer: val is 0 (silent) .. 255 (full beep).
      // camera_servo_node.py sends this whenever LiDAR reads a new distance.
      int val = constrain(atoi(line + 1), 0, 255);
      analogWrite(BUZZER_PIN, val);
      break;
    }
    case '?':
      Serial.print(F("P")); Serial.print(panUs);
      Serial.print(F(" T")); Serial.println(tiltUs);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  // Buzzer pin: ensure it starts silent.
  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(BUZZER_PIN, 0);
  // Servos stay detached until the first command, so the head does not twitch
  // to some arbitrary pose while the Pi is still booting.
  Serial.println(F("camera_head ready"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = '\0';
      if (len && !skipLine) handle(buf);
      len = 0;
      skipLine = false;
    } else if (skipLine) {
      continue;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;
      skipLine = true;
    }
  }
}
