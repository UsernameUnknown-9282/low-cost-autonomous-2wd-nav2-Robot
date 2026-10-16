# Module 1: ROS 2 Lightweight Motor Driver Node (2WD Robot)

This module provides the ROS 2 Humble Python motor driver node for a 2WD differential drive robot driven by a **Raspberry Pi Zero 2 W**, **TB6612FNG dual H-bridge motor driver**, and **two yellow TT DC gearbox motors**.

---

## 1. Hardware Pinout & Wiring

### TB6612FNG to Raspberry Pi Zero 2 W
| TB6612FNG Pin | Raspberry Pi Zero 2 W Pin | Function / Logic |
| :--- | :--- | :--- |
| **VM** | Battery positive (5V – 9V) | Motor power supply |
| **VCC** | Pi 3.3V (Pin 1 or 17) | Logic power supply |
| **GND** | Pi GND & Battery GND | **Common Ground (Essential!)** |
| **STBY** | **GPIO 22** (Physical Pin 15) | Standby Enable (HIGH = Run, LOW = Standby) |
| **PWMA** | **GPIO 12** (Physical Pin 32) | Left Motor Speed PWM (Hardware PWM0) |
| **AIN1** | **GPIO 5**  (Physical Pin 29) | Left Motor Direction 1 |
| **AIN2** | **GPIO 6**  (Physical Pin 31) | Left Motor Direction 2 |
| **PWMB** | **GPIO 13** (Physical Pin 33) | Right Motor Speed PWM (Hardware PWM1) |
| **BIN1** | **GPIO 23** (Physical Pin 16) | Right Motor Direction 1 |
| **BIN2** | **GPIO 24** (Physical Pin 18) | Right Motor Direction 2 |
| **AO1 / AO2** | Left TT Motor Terminals | Left Motor Output |
| **BO1 / BO2** | Right TT Motor Terminals | Right Motor Output |

### Direction Logic Table
| State | IN1 (AIN1 / BIN1) | IN2 (AIN2 / BIN2) | PWM Duty | Motor Output |
| :--- | :---: | :---: | :---: | :--- |
| **Forward** | `HIGH` | `LOW` | `min_pwm` to `max_pwm` | Spins Forward |
| **Reverse** | `LOW` | `HIGH` | `min_pwm` to `max_pwm` | Spins Backward |
| **Stop / Coast** | `LOW` | `LOW` | `0.0` | Coast / Stop |
| **Standby** | Don't care | Don't care | Any (`STBY=LOW`) | Disabled / Low power |

---

## 2. Key Features

1. **Differential Drive Kinematics & Desaturation**:
   - Track width $L = 0.14\text{ m}$ (wheel separation)
   - Wheel radius $R = 0.033\text{ m}$ (diameter $\approx 0.066\text{ m}$)
   - Forward kinematics:
     $$v_L = v_x - \frac{\omega_z \cdot L}{2}, \quad v_R = v_x + \frac{\omega_z \cdot L}{2}$$
   - Inverse kinematics:
     $$v_x = \frac{v_R + v_L}{2}, \quad \omega_z = \frac{v_R - v_L}{L}$$
   - **Curvature-Preserving Velocity Desaturation**: Proportional scaling when wheel speeds exceed `max_wheel_speed`, preserving turning radius $\omega_z / v_x$.
2. **Yellow TT Motor Gearbox Static Friction (Deadband) Compensation**:
   - Cheap yellow TT gearbox motors have significant static friction and do not rotate below $25\% - 30\%$ duty cycle.
   - A deadband threshold (`deadband_speed: 0.02 m/s`) stops the motors when no motion is commanded.
   - Commands at zero speed or below the deadband stop cleanly.
   - When commanded above the threshold, PWM instantly jumps to `min_pwm` ($28\%$) and scales linearly up to $100\%$ at `max_wheel_speed` ($0.40\text{ m/s}$).
3. **Safety Watchdog Failsafe (300 ms) & NaN Command Rejection**:
   - High-frequency timer (20 Hz) monitors incoming `/cmd_vel` messages.
   - If no message is received for $\ge 300\text{ ms}$ (e.g. Wi-Fi drop, computer disconnect, planner freeze), motor PWM is immediately cut to zero to prevent runaway.
   - Immediate failsafe rejection of malformed `NaN` or `Inf` Twist commands (stops motors and logs warning instead of unhandled exceptions or runaway reverse rotation).
   - Resumes cleanly as soon as valid `/cmd_vel` commands are restored.
4. **Mock Hardware Fallback & Idempotent Teardown**:
   - Can be run anywhere (WSL2, Ubuntu PC, CI, macOS).
   - If `gpiozero` or `RPi.GPIO` are unavailable, it automatically switches to `MockTB6612Driver`, logging simulated pin states and duty cycles without crashing.
   - Protected pin state transitions (cuts PWM before toggling H-bridge direction) to prevent inrush transients on physical hardware.
   - Idempotent cleanup guarantees safe, single-pass teardown on SIGTERM / SIGINT.

---

## 3. Configuration Parameters (`motor_params.yaml`)

| Parameter | Default | Description |
| :--- | :---: | :--- |
| `track_width` | `0.14` | Distance between drive wheels in meters |
| `wheel_radius` | `0.033` | Radius of wheels in meters |
| `max_wheel_speed` | `0.40` | Maximum linear wheel speed at 100% PWM (m/s) |
| `deadband_speed` | `0.02` | Minimum wheel speed command below which motor stops |
| `min_pwm` | `0.28` | Minimum starting PWM (28%) to overcome gearbox stiction |
| `max_pwm` | `1.00` | Maximum PWM duty cycle (100%) |
| `pwm_frequency` | `1000` | PWM switching frequency in Hz |
| `watchdog_timeout_sec` | `0.300` | Safety watchdog cutoff threshold in seconds |
| `mock_hardware` | `false` | Force mock mode (auto-falls back if GPIO missing) |
| `cmd_vel_topic` | `"/cmd_vel"` | Subscribed Twist topic |
| `invert_left_motor` | `false` | Invert left motor direction if wired backwards |
| `invert_right_motor` | `false` | Invert right motor direction if wired backwards |

---

## 4. Usage & Execution

### Running directly with Python:
```bash
python3 motor_driver_node.py --ros-args --params-file motor_params.yaml
```

### Running via Launch Script:
```bash
bash launch_motor_driver.sh
```

### Running via ROS 2 Launch:
```bash
ros2 launch launch_motor_driver.py
```

### Testing with ROS 2 CLI:
In another terminal, publish a forward command:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```
Or publish an in-place counter-clockwise rotation:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

---

## 5. Automated Verification Tests

Run the full pytest suite:
```bash
python3 -m pytest -v test_motor_driver.py
```

Run the live ROS 2 integration verification script:
```bash
python3 verify_live_node.py
```
