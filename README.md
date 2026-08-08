# Closed-Loop DC Motor Speed Controller
Arduino-based PI speed controller using encoder feedback, PWM motor control, fixed-period sampling, and low-pass filtering.

## Project Overview
<img width="1215" height="546" alt="image" src="https://github.com/user-attachments/assets/5e532a93-157c-45c3-8b84-665178cc24ef" />

This project implements closed-loop speed control of a 12 V DC gear motor using Elegoo Uno, encoder feedback, and an L298N motor driver. Motor speed is measured from encoder pulses, filtered using a low-pass filter and regulated with a PI controller. The system was tested across multiple speed setpoints, including a 0-100 RPM step response.

## Hardware
Components Used:
- L298N Motor Driver Controller Board Module Stepper Motor DC Dual H-Bridge
- DC 12 V Encoder Gear Motor
- Alitove DC 12 V 5A Power Supply Adapter as the power source
- Elegoo Uno R3
- Breadboard

**Wiring:**  
All encoder wires were plugged into the breadboard because they did not fit securely enough into the motor driver and Uno.

**Motor**
- White Motor Wire → L298N OUT1
- Red Motor Wire → L298N OUT2

**Encoder**
- Yellow Encoder Wire → Uno D3
- Green Encoder Wire → Uno D2
- Blue Encoder Wire → Uno 5V
- Black Encoder Wire → Uno GND

**Motor Driver**
- L298N ENA → Uno D5
- L298N IN1 → Uno D6
- L298N IN2 → Uno D7
- L298N GND → Uno GND

**Power**
- Negative Power Source → L298N GND
- Positive Power Source → L298N +12 V

## System Design
The encoder generates pulses as the motor rotates, the Arduino counts these pulses and calculates the motor speed in RPM. A low-pass filter reduces noise in the RPM measurement and the measured RPM is compared to the target RPM to calculate speed error.
The PI controller uses this error to determine the required motor power and the Arduino outputs a PWM signal to the L298N motor driver which adjusts the motor speed. This process repeats every 10 ms, creating a closed feedback loop.

## Speed Measurement & Filtering
Motor speed is measured using the built-in encoder. An interrupt increments the encoder count whenever a rising edge is detected and every 10 ms the Arduino calculates the change in encoder position and divides it by the elapsed time to determine encoder counts per second.
```cpp
float deltaT = (float)(time - prevTime) / 1.0e6; // change in time in seconds
float deltaPos = (pos - prevPos);
float velocity = deltaPos / deltaT;

// convert encoder counts to rpm
float v1 = velocity / (425) * 60;
```
The measured encoder velocity is converted to RPM using 425 encoder counts per output-shaft revolution.
A low-pass filter is then applied to reduce rapid fluctuations in the measured speed before it is used by the controller:

```cpp
 // low-pass filter (25hz cutoff)
v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
v1Prev = v1;
```

## PI/PID Controller
The filtered motor speed is compared with the target speed to calculate the control error. The proportional term responds to the current speed error while the integral term accumulates error over time to reduce steady-state error. The coefficients for these terms were selected experimentally being Kp=2 and Ki=3.5.
```cpp
float kp=2;
float ki=3.5;
float error=target-v1Filt;
eintegral=eintegral+error*deltaT;
float u = kp*error + ki*eintegral;
```

The derivative term was also implemented to evaluate full PID control however tests with Kd=0.01 resulted in little visible improvement compared with the PI controller since the additional derivative term can also increase sensitivity to encoder measurement noise, the final controller uses PI control.

## Results
| Target Speed | Measured Speed Range | Maximum Error |
|-------------:|---------------------:|--------------:|
| 50 RPM       | 49.57-50.72          | 1.44%         |
| 100 RPM      | 99.33–101.29 RPM     | 1.29%         |
| 150 RPM      | 149.27-150.53        | 0.49%         |
| 200 RPM      | 197.79-199.58        | 1.11%         |

## Step Response
The controller was tested using a step from 0 RPM to a target of 100 RPM
<img width="1213" height="537" alt="image" src="https://github.com/user-attachments/assets/b0c742ad-7f61-4e09-98bc-9eb53bcda7db" />

## Steady-State Performance
At a target of 100 RPM, the measured speed remained between 99.33 and 101.29 RPM, as referenced from the table
<img width="972" height="428" alt="image" src="https://github.com/user-attachments/assets/9101888c-f200-4b8e-90d9-610d25fecfae" />
<img width="957" height="427" alt="image" src="https://github.com/user-attachments/assets/cf43d31a-02cb-4ce3-aab9-d987c82e48b2" />

## PI vs PID Controller
A derivative term with Kd=0.01 was tested under the same 100 RPM step conditions but produced little visible improvement in the transient response so PI control was retained for the final controller
- PI:<img width="1208" height="530" alt="image" src="https://github.com/user-attachments/assets/7556c3d8-792c-4524-9f4e-b8164b82c8a8" />
- PID: <img width="966" height="433" alt="image" src="https://github.com/user-attachments/assets/07ceb39a-ebed-435b-8ce3-358e501d88b2" />

## Limitations
One encoder channel was not functioning reliably, so motor speed was measured using a single encoder channel. This was sufficient for speed measurement in a fixed direction, but prevented full quadrature encoding and reliable direction detection from the encoder
