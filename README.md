# Quadruped Robot Dog Project

## Overview
This project implements a quadruped robot dog with inverse kinematics-based locomotion control. The robot features four legs with 2 degrees of freedom (DOF) each and a gimbal-controlled head mechanism. The control system uses a trot gait pattern and includes forward, reverse, and turning movements.

## Hardware Configuration

### Servo Layout
```
         FRONT
           +
()--()---|===|---()--()
         |   |
         |   |
         |   |
         |   |
()--()---|___|---()--()
          REAR
```

### Joint Configuration
- **Shoulders**: 4 servos (FL, FR, RL, RR)
- **Elbows**: 2 servos (Front legs)
- **Knees**: 2 servos (Rear legs)
- **Gimbal**: 2 servos (Head nod and shake)

### Physical Parameters
- Upper leg length (L1): 15cm
- Lower leg length (L2): 15cm

## Servo Configuration

### Default Positions
- Default Position: 90°
- Minimum Position: 0°
- Maximum Position: 180°

### Movement Limits
#### Shoulder
- Rest Position: 102° (90° + 12°)
- Maximum Forward Swing: 160°
- Maximum Backward Swing: 70°

#### Elbow
- Rest Position: 35° (45° - 10°)
- Maximum Forward Swing: 18°
- Maximum Backward Swing: 85°

#### Knee
- Rest Position: 35° (45° - 10°)
- Maximum Forward Swing: 20°
- Maximum Backward Swing: 80°

#### Gimbal (Head)
- Rest Position: 90°
- X-axis Range: 20° to 160°
- Y-axis Range: 20° to 160°

## Control System Implementation

### Memory Management
The project uses structured arrays to manage servo assignments:
```cpp
struct LegAttachments {
    Servo* const Shoulder[5];  // 4 shoulder servos + nullptr
    Servo* const Elbow[3];     // 2 front elbow servos + nullptr
    Servo* const Knee[3];      // 2 rear knee servos + nullptr
    Servo* const Gimbal[3];    // 2 gimbal servos + nullptr
};
```

### Movement Control
The robot implements four primary movement directions:
- Forward (F)
- Backward (B)
- Left Turn (L/G)
- Right Turn (R/I)
- Stop (S)

### Gait Implementation
The trot gait is implemented using a state machine with four phases:
1. **LIFT**: Raises the leg
2. **SWING**: Moves the leg forward/backward
3. **LOWER**: Places the leg down
4. **SUPPORT**: Maintains ground contact during movement

### Inverse Kinematics

The robot uses a 2-DOF inverse kinematics solution for leg positioning. The mathematical model is based on:

1. **Leg Position Calculation**:
   - Uses the Law of Cosines to determine joint angles
   - Implements coordinate transformation for x-y-z positioning

2. **Angle Calculations**:
```cpp
// θ2 calculation using Law of Cosines
theta2 = acos((x² + y² - L1² - L2²) / (2 * L1 * L2))

// θ1 calculation using atan2
theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))
```

## Communication Interface
- Serial communication via UART (Serial3)
- Baud rate: 115200
- Command-based control system using single character commands

## Pin Configuration
Starting from pin 20 on the MEGA2560:
- Shoulders: pins 20-23
- Elbows: pins 24-25
- Knees: pins 26-27
- Gimbal: pins 28-29

## Movement Parameters
- Step Size: 3.0 units
- Step Height: 2.0 units
- Turn Modifiers:
  - Inner leg: 0.5× step size
  - Outer leg: 1.2× step size

## Safety Features
1. **Angle Constraints**: All servo movements are constrained within safe operating limits
2. **Initialization Checks**: Systematic servo attachment verification
3. **Soft Startup**: Gradual initialization with delays between servo attachments

## Known Limitations
1. Open-loop control system without feedback
2. Limited backward turning capabilities (not implemented)
3. Fixed step size and height parameters

## Future Improvements
1. Implementation of closed-loop control with feedback
2. Addition of sensor integration for stability
3. Dynamic gait parameter adjustment
4. Implementation of additional gaits (walk, gallop)
5. Enhanced backward movement capabilities

## Usage Notes
1. Ensure all servo connections match the pin configuration
2. Initialize the system in a stable position
3. Allow for complete initialization before sending movement commands
4. Monitor servo temperatures during extended operation
5. Maintain stable power supply for consistent operation

