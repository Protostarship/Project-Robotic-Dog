#include <HardwareSerial.h>
#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// Name the pin
HardwareSerial& DOG = Serial3;  // 14, 15

/**
 Dog Body

          FRONT

            +
 ()--()---|===|---()--()
          |   |
          |   |
          |   |
          |   |
 ()--()---|___|---()--()

           REAR
 */

// Servo joints instances
Servo fl_shoulder, fr_shoulder, rl_shoulder, rr_shoulder;
Servo fl_elbow, fr_elbow;
Servo rl_knee, rr_knee;
Servo g_shook, g_nod;

struct LegAttachments {
  // Leg array
  Servo* const Shoulder[5] = { &fl_shoulder, &fr_shoulder, &rl_shoulder, &rr_shoulder, nullptr };
  Servo* const Elbow[3] = { &fl_elbow, &fr_elbow, nullptr };  // Front
  Servo* const Knee[3] = { &rl_knee, &rr_knee, nullptr };     // Hind
  Servo* const Gimbal[3] = { &g_shook, &g_nod, nullptr };     // Gimbal (Head)
  // Array pointer
  Servo* const* const LegArray[5] = { Shoulder, Elbow, Knee, Gimbal, nullptr };
  // Array element counter
  const int ShoulderCount = sizeof(Shoulder) / sizeof(Shoulder[0]);
  const int ElbowCount = sizeof(Elbow) / sizeof(Elbow[0]);
  const int KneeCount = sizeof(Knee) / sizeof(Knee[0]);
  const int GimbalCount = sizeof(Gimbal) / sizeof(Gimbal[0]);
  const int LegCount = [](Servo* const* const arr[]) {
    int i = 0;
    while (arr[i]) i++;
    return i;
  }(LegArray);
};

enum DIRECTIONS {
  DIR_FORWARD = 'F',
  DIR_BACKWARD = 'B',
  DIR_LEFT = 'L',
  DIR_RIGHT = 'R',
  DIR_FORWARD_L = 'G',
  DIR_FORWARD_R = 'I',
  DIR_BACKWARD_L = 'H',
  DIR_BACKWARD_R = 'J',
  DIR_STOP = 'S'
};

struct WalkingConfig {
  // Servo
  const int defaultPos = 90;
  const int minPos = 0;
  const int maxPos = 180;
  // Shoulder
  const int shoulderRestPos = maxPos / 2 + 12;
  const int shoulderMaxSwingF = maxPos - 20;      // Forward
  const int shoulderMaxSwingB = defaultPos - 20;  // Backward
  // Elbow
  const int elbowRestPos = defaultPos / 2 - 10;
  const int elbowMaxSwingF = minPos + 18;     // Forward
  const int elbowMaxSwingB = defaultPos - 5;  // Backward
  // Knee
  const int kneeRestpos = defaultPos / 2 - 10;
  const int kneeMaxSwingF = minPos + 20;      // Forward
  const int kneeMaxSwingB = defaultPos - 10;  // Backward
  // Head / Gimbal
  const int gimbalRestPos = defaultPos;
  // Head shook
  const int gimbalMinX = minPos + 20;
  const int gimbalMaxX = maxPos - 20;
  // Head nod
  const int gimbalMinY = minPos + 20;
  const int gimbalMaxY = maxPos - 20;
  // Switch instances
  bool isShoulderAavailable = false;
  bool isElbowAvailable = false;
  bool isKneeAvailable = false;
  bool isGimbalAvailable = false;
  // Switch call
  void shoulderSW() {
    isShoulderAavailable = !isShoulderAavailable;
  }
  void elbowSW() {
    isElbowAvailable = !isElbowAvailable;
  }
  void kneeSW() {
    isKneeAvailable = !isKneeAvailable;
  }
  void gimbalSW() {
    isGimbalAvailable = !isGimbalAvailable;
  }
  // Timing instances
  unsigned long lastShoulderUpdate = 0;
  unsigned long lastElbowUpdate = 0;
  unsigned long lastKneeUpdate = 0;
  unsigned long lastGimbalUpdate = 0;
};

// Struct init
LegAttachments legAttach;
WalkingConfig walkConfig;

// --------------- TROT MOVEMENT LOGIC (DON'T MODIFY, i beg u) ---------------

// Leg lengths based on document
const float L1 = 15;  // Upper leg (cm)
const float L2 = 15;  // Lower leg (cm)

// Convert radians to degrees
#define RAD_TO_DEG 57.2958

// Gait pahse
enum GaitPhase {
  LIFT,    // Raise
  SWING,   // Move
  LOWER,   // Place
  SUPPORT  // Support
};
// Modes
enum MovementMode {
  FORWARD,
  REVERSE,
  TURN_LEFT,
  TURN_RIGHT,
  STOP
};
// Types
enum ServoType {
  SHOULDER,
  ELBOW,
  KNEE,
  GIMBAL
};

// GaitController
struct GaitController {
  struct Leg {
    GaitPhase phase;
    Servo* shoulder;
    Servo* joint;
    float x, y;
    ServoType jointType;

    // Modified constructor to include joint type
    Leg(GaitPhase p, Servo* s, Servo* j, float ix, float iy, ServoType jt)
      : phase(p), shoulder(s), joint(j), x(ix), y(iy), jointType(jt) {}

    void move() {
      // Law of Cosines: cos(θ) = (a² + b² - c²) / (2ab)
      float theta2 = acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2));
      // Calculate angle θ1 using atan2 and trigonometric identities
      float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

      // Convert to degrees and add offset, then apply safe write
      int shoulderAngle = 90 + theta1 * RAD_TO_DEG;
      int jointAngle = 90 - theta2 * RAD_TO_DEG;

      // Use safe write with stored joint type
      safeServoWrite(shoulder, shoulderAngle, SHOULDER, walkConfig);
      safeServoWrite(joint, jointAngle, jointType, walkConfig);
    }

    // Utility function to constrain angle within defined limits based on servo type
    int constrainServoAngle(int angle, ServoType type, const WalkingConfig& config) {
      switch (type) {
        case SHOULDER:
          // For shoulder servos
          if (angle > config.shoulderMaxSwingF) return config.shoulderMaxSwingF;
          if (angle < config.shoulderMaxSwingB) return config.shoulderMaxSwingB;
          break;

        case ELBOW:
          // For elbow servos
          if (angle > config.elbowMaxSwingB) return config.elbowMaxSwingB;
          if (angle < config.elbowMaxSwingF) return config.elbowMaxSwingF;
          break;

        case KNEE:
          // For knee servos
          if (angle > config.kneeMaxSwingB) return config.kneeMaxSwingB;
          if (angle < config.kneeMaxSwingF) return config.kneeMaxSwingF;
          break;

        case GIMBAL:
          // X-axis
          if (angle > config.gimbalMaxX) return config.gimbalMaxX;
          if (angle < config.gimbalMinX) return config.gimbalMinX;
          // Y-axis
          if (angle > config.gimbalMaxY) return config.gimbalMaxY;
          if (angle < config.gimbalMinY) return config.gimbalMinY;
          break;
      }
      return angle;
    }

    // Safe servo write function that applies constraints
    void safeServoWrite(Servo* servo, int angle, ServoType type, const WalkingConfig& config) {
      int safeAngle = constrainServoAngle(angle, type, config);
      servo->write(safeAngle);
    }

    void update(float stepSize, float height) {
      switch (phase) {
        case LIFT:
          y = height;
          phase = SWING;
          break;
        case SWING:
          x = stepSize;
          phase = LOWER;
          break;
        case LOWER:
          y = 0;
          phase = SUPPORT;
          break;
        case SUPPORT:
          x = -stepSize;
          phase = LIFT;
          break;
      }
    }

    // Dawg chillin stance
    void standby() {
      x = 0;
      y = 0;
      move();
    }
  };

  // Instances
  Leg FL, FR, RL, RR;
  MovementMode mode;
  // Control attachments
  GaitController(LegAttachments& legs)
    : FL{ SUPPORT, legs.Shoulder[0], legs.Elbow[0], 0, 0, ELBOW },
      FR{ LIFT, legs.Shoulder[1], legs.Elbow[1], 0, 0, ELBOW },
      RL{ LIFT, legs.Shoulder[2], legs.Knee[0], 0, 0, KNEE },
      RR{ SUPPORT, legs.Shoulder[3], legs.Knee[1], 0, 0, KNEE },
      mode(STOP) {}

  // Mode switch
  void setMode(MovementMode newMode) {
    mode = newMode;
  }

  // Movement
  void move() {
    /**
      Dawg movement paramater visualized in 2D Plane for 2DOF & 2 Joint robot

                z
                |
                |
                |
                x --------- y

      #Simply 
     */
    float stepSize = 3.0;  // From x to y
    float height = 2.0;    // From x to z

    if (mode == STOP) {
      FL.standby();
      FR.standby();
      RL.standby();
      RR.standby();
      return;
    }

    switch (mode) {
      case FORWARD:
        FL.update(stepSize, height);
        FR.update(stepSize, height);
        RL.update(stepSize, height);
        RR.update(stepSize, height);
        break;

      case REVERSE:
        FL.update(-stepSize, height);
        FR.update(-stepSize, height);
        RL.update(-stepSize, height);
        RR.update(-stepSize, height);
        break;

      case TURN_LEFT:
        FL.update(stepSize * 0.5, height);
        FR.update(stepSize * 1.2, height);
        RL.update(stepSize * 0.5, height);
        RR.update(stepSize * 1.2, height);
        break;

      case TURN_RIGHT:
        FL.update(stepSize * 1.2, height);
        FR.update(stepSize * 0.5, height);
        RL.update(stepSize * 1.2, height);
        RR.update(stepSize * 0.5, height);
        break;
    }

    FL.move();
    FR.move();
    RL.move();
    RR.move();
  }
};

// Global controller instance
GaitController gait(legAttach);

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// MEGA2560
void setup() {
  DOG.begin(115200);
  // Initialization
  int pin = 20;  // Pin assignments

  // Attach shoulders and init
  for (int i = 0; i < legAttach.ShoulderCount; i++) {  // 20, 21, 22, 23
    legAttach.Shoulder[i]->attach(pin++);
    legAttach.Shoulder[i]->write(walkConfig.defaultPos);
    delay(50);

    // init
    if (i >= legAttach.ShoulderCount) { walkConfig.shoulderSW(); }
    legAttach.Shoulder[i]->write(walkConfig.shoulderRestPos);
  }

  // Attach elbows and init
  for (int i = 0; i < legAttach.ElbowCount; i++) {  // 24, 25
    legAttach.Elbow[i]->attach(pin++);
    legAttach.Elbow[i]->write(walkConfig.defaultPos);
    delay(50);

    // init
    if (i >= legAttach.ElbowCount) { walkConfig.elbowSW(); }
    legAttach.Elbow[i]->write(walkConfig.elbowRestPos);
  }

  // Attach knees and init
  for (int i = 0; i < legAttach.KneeCount; i++) {  // 26, 27
    legAttach.Knee[i]->attach(pin++);
    legAttach.Knee[i]->write(walkConfig.defaultPos);
    delay(50);

    // init
    if (i >= legAttach.KneeCount) { walkConfig.kneeSW(); }
    legAttach.Knee[i]->write(walkConfig.kneeRestpos);
  }

  // Attach gimbal servos and init
  for (int i = 0; i < legAttach.GimbalCount; i++) {  // 28, 29
    legAttach.Gimbal[i]->attach(pin++);
    legAttach.Gimbal[i]->write(walkConfig.defaultPos);
    delay(50);

    // init
    if (i >= legAttach.GimbalCount) { walkConfig.gimbalSW(); }
    legAttach.Gimbal[i]->write(walkConfig.gimbalRestPos);
  }
}

void loop() {
  unsigned long currentMillis = millis();  // When yh used

  if (DOG.available()) {
    // do sum
    /**
      gait.setMode(MODE) <- Directions
      gait.move(); <- init movement
      delay or sum
     */
    char command = DOG.read();
    switch (command) {
      // case somthig do somthing
      case DIR_FORWARD :
        gait.setMode(FORWARD);
        gait.move();
        delay(500);
      case DIR_BACKWARD :
        gait.setMode(REVERSE);
        gait.move();
        delay(500);
      // Unified left
      case DIR_LEFT :
      case DIR_FORWARD_L :
        gait.setMode(TURN_LEFT);
        gait.move();
        delay(500);
      // Unified right
      case DIR_RIGHT :
      case DIR_FORWARD_R :
        gait.setMode(TURN_RIGHT);
        gait.move();
        delay(500);
      // Unhandled modes
      // case DIR_BACKWARD_L :
      // case DIR_BACKWARD_R :
      case DIR_STOP :
        gait.setMode(STOP);
        gait.move();
        delay(500);
    }
  }
}