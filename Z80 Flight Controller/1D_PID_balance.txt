; This program is not finished!
; To-do:
; 1 - Finish calculateOrientation function
; 2 - Make calculatePID function

; This program uses a PID controller to stabilize a quadcoptor on one axis.
; A complementary filter is used on the IMU accelerometer and gyro.
; Note: This is not tuned for any drone at the moment

; ------------------------- CONTENTS -------------------------
; 1 - Setup
; 2 - Main Loop
; 3 - Functions
; 4 - Variables

; ------------------------- Setup -------------------------
	.org $0000 ; Start of ROM
	
	; ---------- Delay functions ----------
	; (used to time the Oneshot protocol)
	#define nops4 nop \ nop \ nop \ nop
	#define nops7 nops4 \ nop \ nop \ nop
	#define nops16 nops4 \ nops4 \ nops4 \ nops4
	#define nops55 nops16 \ nops16 \ nops16 \ nops7
	#define nops64 nops16 \ nops16 \ nops16 \ nops16
	#define nops247 nops64 \ nops64 \ nops64 \ nops55
	#define nops257 nops64 \ nops64 \ nops64 \ nops64 \ nop
	#define delay988cc ld b,75 \ djnz $-0 \ ld b,0 \ nop
	
	;  ---------- Constants ----------
	#define loopFreq 500 ; Hz
	#define highLength 6 ; seconds
	#define lowLength 5 ; seconds
	#define testLength 10 ; seconds
	#define loopDelay 520 ; x 13 cc
	#define hoverThrottle 66 ; 66/255
	
	; IMU Addresses
	#define yAccAddress $3D
	#define xGyroAddress $43
	
	; Roll angle complementary filter
	#define gyroGain 0.98
	#define accGain 0.02
	
	; Test segments:
	; Segment 1: Send max throttle signal to all motors (ESC calibration)
	; Segment 2: Send min throttle signal to all motors (ESC calibration)
	; Segment 3: Use PID to balance drone
	; Segment 4: Send min throttle signal once test is completed
	#define loopCountHigh loopFreq * highLength
	#define loopCountLow loopFreq * lowLength
	#define loopCountTest loopFreq * testLength
	#define segment2Start loopCountHigh
	#define segment3Start segment2Start + loopCountLow
	#define segment4Start segment3Start + loopCountTest

	; Copy program and variables from ROM to RAM so self-modifying code works
	ld hl, RAMCopyStart
	ld de, $8000
	ld bc, RAMend - $8000
	ldir ; takes ~3ms
	
	jp programStart

RAMCopyStart:
	.org $8000 ; Start of RAM
	
; ALL CODE BELOW THIS POINT WILL BE LOADED INTO RAM
programStart:

; ------------------------- Main Loop -------------------------
mainLoop: ; total = 8613cc + 7997cc delay = 16610 = 481 Hz

	; ---------- Calculate motor throttles ----------
	
	; Increase loopCounter once per frame until it reaches 65,535
	; Then it stays at 65,535
	ld de,$0001							;	if (loopCounter < 65,535) {
	ld hl,(loopCounter)				;		loopCounter++
	add hl,de									;	}
	jp c, loopCounterAtMax
	ld hl,(loopCounter)
	inc hl
	ld (loopCounter),hl
loopCounterAtMax:

	; Find what test segment we're currently in
	ld hl,(loopCounter)
	ld de,-segment4Start				;	if (loopCounter > segment4Start) {
	add hl,de									;		goto setThrottle0
	jp c, setThrottle0					;	}
	ld hl,(loopCounter)				;	else if (loopCounter > segment3Start) {
	ld de,-segment3Start				;		goto setThrottleBalance
	add hl,de									;	}
	jp c, setThrottleBalance			;	else if  (loopCounter > segment2Start) {
	ld hl,(loopCounter)				;		goto setThrottle0
	ld de,-segment2Start				;	}
	add hl,de									;	else {
	jp c, setThrottle0					;		goto setThrottle100
	jp setThrottle100					;	}
	
	; Set all 4 motors to min throttle (cuts motors off)
setThrottle0:
	xor a
	ld (motor1Throttle),a
	ld (motor2Throttle),a
	ld (motor3Throttle),a
	ld (motor4Throttle),a
	jp endSetThrottle

	; Calculate throttle signals from PID controller
setThrottleBalance: ; motors may be swapped
	call getSensorData
	call calculateOrientation
	call calculatePID
	ld a,(rollCommand)
	ld e, hoverThrottle
	add a,e
	ld (motor1Throttle),a			;	motor1Throttle = hoverThrottle + rollCommand
	ld (motor2Throttle),a			;	motor2Throttle = hoverThrottle + rollCommand
	sub e
	neg
	add a,e
	ld (motor3Throttle),a			;	motor3Throttle = hoverThrottle - rollCommand
	ld (motor4Throttle),a			;	motor4Throttle = hoverThrottle - rollCommand
	jp endSetThrottle

	; Set all 4 motors to max throttle (used in ESC calibration)
setThrottle100:
	ld a,250
	ld (motor1Throttle),a
	ld (motor2Throttle),a
	ld (motor3Throttle),a
	ld (motor4Throttle),a
endSetThrottle:
	
	; ---------- Oneshot125 Protocol  ----------
	; This drone uses the Oneshot125 protocol to control the 4 motor ESCs.
	; The Z80 processor runs at 8 MHz, so with a 500 Hz control loop,
	; each frame is 16,000 cc (clock cycles). The Oneshot125 signal has a
	; pulse width of 125 - 250 microseconds, which is 1000 - 2000 cc on
	; the Z80. 4 Oneshot125 signals are sent each frame in series, which takes
	; 8000 cc. This means only 8000 cc is left for the rest of the program.
	
	; Temporary delay that makes sure the control loop runs at ~500hz.
	; I'll be lucky if I keep this in the final program, but it'll probably end up
	; being below 500hz.
	ld b,loopDelay - 512 ; 7997cc, doesn't work because b > 255, so added 2 more
	djnz $-0
	djnz $-0
	djnz $-0
	
	; Each Oneshot signal consists of 3 parts. First, a high signal is sent, then
	; there's a 125 us delay. After that is a buffer of 257 nop (no operation)
	; instructions, which also takes ~125 us. A "send low signal" instruction
	; is placed at a position in the buffer proportional to the throttle, which
	; allows for a pulse to be generated with any width from 125 - 250 us.
	
	; Replace the last "send low signal" instruction in the buffer with nop
	; instructions to reset buffer to original state
	ld hl,$0000 ; nop \ nop
lastMotorOut1:
	ld (Oneshot1),hl ; Oneshot1/2/3/4 are just initial values. They are self-modifying
lastMotorOut2:
	ld (Oneshot2),hl
lastMotorOut3:
	ld (Oneshot3),hl
lastMotorOut4:
	ld (Oneshot4),hl
	
	; Place "send low signal" in buffer for motor 1
	ld de, motorThrottles
	ld b,0
	ld a,(de)
	ld c,a
	ld hl, Oneshot1
	add hl,bc
	ld (lastMotorOut1+1),hl
	ld (hl),$ED ; out (c),c
	inc hl
	ld (hl),$49
	
	; Place "send low signal" in buffer for motor 2
	inc de
	ld a,(de)
	ld c,a
	ld hl, Oneshot2
	add hl,bc
	ld (lastMotorOut2+1),hl
	ld (hl),$ED
	inc hl
	ld (hl),$49
	
	; Place "send low signal" in buffer for motor 3
	inc de
	ld a,(de)
	ld c,a
	ld hl, Oneshot3
	add hl,bc
	ld (lastMotorOut3+1),hl
	ld (hl),$ED
	inc hl
	ld (hl),$49
	
	; Place "send low signal" in buffer for motor 4
	inc de
	ld a,(de)
	ld c,a
	ld hl, Oneshot4
	add hl,bc
	ld (lastMotorOut4+1),hl
	ld (hl),$ED
	inc hl
	ld (hl),$49
		
	; Do oneshot125 for all 4 motors, 8163cc = 83 + 988*4 + 255*4*4 + 12*4
	ld c,$02 ; $02 is used since bit 1 is the IMU chip select. We want it pulled
	ld a,$82 ; high when not being used.
	out (c),a
	delay988cc
Oneshot1:
	nops257 ; 257 since out (c),0 is 2 bytes, max PWM value = 250 (for now)
	ld a,$42
	out (c),a
	delay988cc
Oneshot2:
	nops257
	ld a,$22
	out (c),a
	delay988cc
Oneshot3:
	nops257
	ld a,$12
	out (c),a
	delay988cc
Oneshot4:
	nops257

	jp mainLoop

; ------------------------- Functions -------------------------

; ---------- Get Data from IMU  ----------
; Get accelerometer and gyro data from MPU-9250.

; Registers:
; A, F, B, D	= Destroyed
; H, L			= Unaffected
; C				= Y acceleration (output)
; E				= X angular rate (output)
getSensorData:
	ld d,yAccAddress + $80 ; bit 7 of IMU address means we're reading
	call readDataSPI
	ld c,e
	ld d,xGyroAddress + $80
	call readDataSPI
	ret

; ---------- Read Data from SPI Device ----------
; Reads one byte of data from an SPI device.

; Registers:
; D				= address (input)
; A, F, B		= Destroyed
; C, H, L		= Unaffected
; E				= data (output)
readDataSPI:
	; Output: bit 3 = SCL, bit 2 = MOSI, bit 1 = NCS, bit 0 = MISO
	ld a,$0A
	out (0),a ; SCL = 1, MOSI = 0, NCS = 1, MISO = N/A
	ld a,$08
	out (0),a ; SCL = 1, MOSI = 0, NCS = 0, MISO = N/A

	ld b,8
sendLoop:
	xor a
	rlc d
	rla
	rla
	rla
	out (0),a ; SCL = 0, MOSI = output, NCS = 0, MISO = N/A
	or $08
	out (0),a ; SCL = 1, MOSI = output, NCS = 0, MISO = N/A
	djnz sendLoop

	ld b,8
receiveLoop: ; Value read is stored in register e
	xor a
	out (0),a ; SCL = 0, MOSI = 0, NCS = 0, MISO = input
	in a,(0)
	rra
	rl e
	ld a,$08
	out (0),a ; SCL = 1, MOSI = 0, NCS = 0, MISO = N/A
	djnz receiveLoop

	ld a,$02
	out (0),a ; SCL = 0, MOSI = 0, NCS = 1, MISO = N/A
	ret

; ---------- Calculate Drone Orientation ----------
; Use a complementary filter on accelerometer and gyro data to find
; the roll angle of the drone

; Registers:
; C				= Y acceleration (input)
; E				= X angular rate (input)
calculateOrientation:

	; Find roll angle from gyro only
	ld hl, (rollAngle)
	ld a,e										;	rollAngleGyro = rollAngle + xGyro
	add a,a
	sbc a,a										; sign extend e into de, then add de to hl
	ld d,a										; 1 radian = 14667.7195 for rollAngle
	add hl,de									; so rollAngle can be from -128 to 128 degrees
	
	; Find roll angle from accelerometer only
	ld hl, DgainTable
   add a,l
   ld l,a
   adc a,h
   sub l
   ld h,a
   ld a,(hl)
   ld (Dval),a
	; Finish this
	ret

; ------------------------- Variables -------------------------
loopCounter: ; Increases by 1 each frame until it reaches 65,535
	.dw 0

rollAngle:
	.dw 0

motorThrottles: ; 250 = 100% power (maybe max can go up to 255)
motor1Throttle:
	.db 0
motor2Throttle:
	.db 0
motor3Throttle:
	.db 0
motor4Throttle:
	.db 0

RAMend:
; \arcsin\left(\frac{x}{64}\right)\cdot14667.7195\cdot.02
