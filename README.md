# MSD_CONTROLER
# Features
1. Working environment:
    + DC Vol: 10-40V
    + Continous Current: 20A
2. Control method:
   + Pwm/Dir
   + Analog (Protentionmeter)/Dir
   + Uart Network
   + Rc Signal.
3. Protecttion:
   + Over Voltage, Under  Voltage , Over Temperature. 
   +  Short circuit, Over current.
4. Advanced feature:
   + Current Limit as Switch Limit (don't accept go more to that Dir when the current touch to the Limit). Setting by hardware.
   + Accept a Acceleration setting by hardware.
   + Bidirectional power supply input (I am note sure, need to test with height load)
# Related tutorials and Notice
    +RC tutorial: https://www.youtube.com/watch?v=utTi0awlUzg
# Firmware
1. Arduino
This code used for controlling multiple motors with arduino board.
The motor will run follow the signal of Pulse and Direct Pin.
- The Direct Pin indicate the rotation direction that rotate clockwise or counterclockwise.
- The motor will move a radian with per rising edge on Pulse pin.

Step 1:
Define parameter of driver.
MSD_NUMBER: Number of motor need control
MSD_PPR: Number pulse per revolution.

Step 2:
Attach Direct and Pulse pin for per motor
Use MSD_AttachPin(char motorIndex, char dirPin, char pulsePin) function.
Example:
Set up Motor index 0 with Direct pin is 2 and Pulse pin is 3
MSD_AttachPin(0, 2, 3)

Step 3:
Init pin and Init timer.
Use MSD_Init()

Step 4:
Set the value that the motor will run.
Use MSD_SetValue(double position, double velocity, double acer, char msdChanel)
position: The distance motor run (rad)
velocity: max speed motor run (rad/s)
acer: accelerate of motor (rad/s)
msdChanel: motor index.
Example
The motor rotate 100 revolution => 100*6.28319 = 628.319 rad => position = 628.319 (rad).
Max speed is 100 rad/s => velocity = 100 (rad/s).
Accelrate is 10 rad/s => acer = 10(rad/s)
Motor index is 0
=> MSD_SetValue(628.319, 100, 10, 0)

Step 5:
Run motor
Use MSD_StartAll() function if you want to run all motor.
Use MSD_StartAt(char index) function if you just want to run a motor.

Note:
- Use MSD_StopAll() or MSD_StopAt(char index) if yot want stop motor when it 's running


