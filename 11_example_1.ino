#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

// for distance to degree mapping
#define DEGREE_MIN 0      // servo angle at 0 degree
#define DEGREE_MAX 180    // servo angle at 180 degrees

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficient to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)

Servo myservo;

// global variables
float dist_ema = _DIST_MAX; // filtered distance (mm)
float dist_prev = _DIST_MAX; // previous distance (mm)
unsigned long last_sampling_time = 0; // time of last sampling (ms)

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO); 
  myservo.write(DEGREE_MIN);    // initialize servo position at 0 degrees

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  // wait until next sampling time
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  // read distance from USS
  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Apply range filter
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;       // if out of range, use previous value
    digitalWrite(PIN_LED, HIGH); // LED OFF
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;       // if below range, use previous value
    digitalWrite(PIN_LED, HIGH); // LED OFF
  } else {
    dist_prev = dist_raw;       // update valid distance
    digitalWrite(PIN_LED, LOW); // LED ON
  }

  // Apply EMA filter
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // Control servo motor angle based on filtered distance
  if (dist_ema <= 180) {
    myservo.write(0); 
  } else if (dist_ema > 180 && dist_ema < 360) {
    int angle = map(dist_ema, 180, 360, 0, 180);
    myservo.write(angle);
  } else {
    myservo.write(180);
  }

  // Output to serial for debugging
  Serial.print("Raw Dist: "); Serial.print(dist_raw);
  Serial.print(", EMA Dist: "); Serial.print(dist_ema);
  Serial.print(", Servo Angle: "); Serial.println(myservo.read());

  // update last sampling time
  last_sampling_time = millis();
}

// Function to measure distance using USS
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // convert time to distance in mm
}
