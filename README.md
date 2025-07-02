🤖 ROVER (rtk_rover)

Features:

✅ ESP-NOW reception of RTCM data from the base station

✅ UART communication with LC29HDA GNSS module

✅ IMU integration (MPU6050 via I2C) for orientation

✅ PWM motor control (RC-style 1000–2000μs signals)

✅ Autonomous navigation with a waypoint system

✅ PID controllers for precise movement control

✅ Navigation algorithms (Haversine formula, bearing calculation)


Key Files:

	•	main.c – Main application with hardcoded waypoints
 
	•	esp_now_rover.c/h – ESP-NOW communication
 
	•	uart_gnss.c/h – GNSS communication and NMEA parsing
 
	•	imu.c/h – MPU6050 IMU sensor handling
 
	•	motor_control.c/h – PWM motor control
 
	•	navigation.c/h – Autonomous navigation system
 

⸻

🔧 HARDWARE CONFIGURATION

Rover pinout:

	•	UART GNSS: TX = 17, RX = 16
	•	I2C IMU: SDA = 21, SCL = 22
	•	PWM Motors: Left = 25, Right = 26

MAC Addresses (update as needed):

	•	Base Station → Rover: {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC}
	•	Rover → Base Station: {0x24, 0x6F, 0x28, 0xDD, 0xEE, 0xFF}

⸻

🗺️ NAVIGATION SYSTEM

The rover uses a sophisticated navigation algorithm:

	•	Waypoint-based navigation with hardcoded coordinates
	•	PID controllers for heading and distance control
	•	State machine: IDLE → TURNING → MOVING → ARRIVED
	•	Real-time sensor fusion (GNSS + IMU)
	•	Collision avoidance and safety checks
