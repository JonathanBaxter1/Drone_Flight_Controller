; This program is not finished!
; To-do:
; 1 - Make calculatePID function (P = .04, I = .0001, D = .015) P = -64->63, I = -32->31, D = -32->31

; This program uses a PID controller to stabilize a quadcoptor on one axis.
; A complementary filter is used on the IMU accelerometer and gyro.

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
	ex de,hl								;	rollError = rollReference - rollAngle
	ld hl,(rollReference)
	and a ; reset carry flag
	sbc hl,de
	call calculatePID
;	ld a,(rollCommand)
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
; A, F, D	= Destroyed
; H, L			= Unaffected
; BC				= Y acceleration (output)
; E				= X angular rate (output)
getSensorData:
	ld d,xGyroAddress + $80 ; bit 7 of IMU address means we're reading
	call readDataSPI8Bit
	ld a,$02 ; Turn off IMU
	out (0),a ; SCL = 0, MOSI = 0, NCS = 1, MISO = N/A
	ld e,c
	ld d,yAccAddress + $80
	call readDataSPI8Bit
	call readDataSPI2Bit
	ld a,$02 ; Turn off IMU
	out (0),a ; SCL = 0, MOSI = 0, NCS = 1, MISO = N/A
	ret

; ---------- Read Data from SPI Device (8 bit) ----------
; Reads 8 bits of data from an SPI device.

; Registers:
; D				= address (input)
; A, F, B		= Destroyed
; E, H, L		= Unaffected
; C				= data (output)
readDataSPI8Bit:
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
receiveLoop: ; Value read is stored in register c
	xor a
	out (0),a ; SCL = 0, MOSI = 0, NCS = 0, MISO = input
	in a,(0)
	rra
	rl c
	ld a,$08
	out (0),a ; SCL = 1, MOSI = 0, NCS = 0, MISO = N/A
	djnz receiveLoop

	ret

; ---------- Read Data from SPI Device (2 bit) ----------
; Reads 2 extra bits of data from an SPI device. Used after the address has
; already been set and the first 8 bits retrieved.

; Registers:
; A, F			= Destroyed
; D, E, H, L	= Unaffected
; BC				= data (output)

	xor a
	ld b,a
	out (0),a ; SCL = 0, MOSI = 0, NCS = 0, MISO = input
	in a,(0)
	rra
	rl c
	rl b
	ld a,$08
	out (0),a ; SCL = 1, MOSI = 0, NCS = 0, MISO = N/A
	xor a
	out (0),a ; SCL = 0, MOSI = 0, NCS = 0, MISO = input
	in a,(0)
	rra
	rl c
	rl b
	ld a,$08
	out (0),a ; SCL = 1, MOSI = 0, NCS = 0, MISO = N/A
	ret
	

; ---------- Calculate Drone Orientation ----------
; Use a complementary filter on accelerometer and gyro data to find
; the roll angle of the drone

; Registers:
; A, F, D		= Destroyed
; BC				= Y acceleration (input)
; E				= X angular rate (input)
; HL				= Roll Angle (output)
calculateOrientation:
	; I used a complementary filter for the gyroscope and accelerometer modeled after the one
	; in this video by Brian Douglas: https://www.youtube.com/watch?v=whSw42XddsU
	; Actually, I've used a lot of things from his videos lol. On each frame, the roll angle is estimated
	; using 2 techniques: using the gyroscope and using the accelerometer. These 2 values are weighted
	; then added together to find the final roll angle estimation.
	
	; ----- Step 1: Find roll angle from gyro only -----
	
	; The estimation using the gyroscope is found by getting the angular rate from the gyro
	; and adding it to the previous roll angle. xGyro is a value from -128 to 127 representing
	; the angular rate on the X axis from -250 to ~248 degrees/sec. At 500 fps, one second
	; of 1 degree/sec rotation returns a roll angle value of 256. Since the gyro roll angle estimation
	; (rollAngleGyro) is a 16 bit signed int, it can store an angle from ~ -128 to 128 degrees.
	ld hl, (rollAngle)
	ld a,e										;	rollAngleGyro = rollAngle + xGyro
	add a,a
	sbc a,a		; sign extend e into de, then add de to hl
	ld d,a
	add hl,de	
	
	; ----- Step 2: Find weighted value from gyro roll angle estimate -----
	
	; Like in the video, I multiply rollAngleGyro by .98 before it's added to the weighted
	; accelerometer measurement (multiplied by .02 so they add up to 1). This is literally
	; what the following section of code does:
	;		rollAngleGyro = rollAngleGyro*.98
	; but for some reason I wanted to use a processor from 1976 that can't do floating point
	; operations or even multiplication to make a drone, so that's why it took 32 lines of code.
	; Due to the Z80's limitations, rollAngleGyro is actually multiplied by (251/256), which is
	; pretty close to .98 (and rollAngleAcc is multiplied by (5/256) which is ~.02).
	
	; First, hl*5 is loaded into the 24 bit register group "cde"
	push bc	; save Y acceleration
	ld bc, 0									;	if (hl >= 0) {	// This is necessary to make the operation work with negative roll angles
	bit 7,h										;		bc = $0000
	jp z, HLpositive						;	else {
	dec bc										;		bc = $FFFF
HLpositive:									;	}
	ld d,h		; load hl into cde, then left bitshift twice to multiply by 4
	ld e,l
	sla e		; bitshift 1
	rl d
	rl c
	sla e		; bitshift 2
	rl d
	rl c
	ld a,e		; add hl to cde once more to make cde = hl*5
	add a,l		; ^add l and e
	ld e,a
	ld a,d		; add h and d
	adc a,h
	ld d,a
	ld a,c		; add b and c (register b is used so negative hl values work)
	adc a,b
	ld c,a
	; Then, the "cde" register group is subtracted from the "hl0" register group (which is hl*256) to get
	; hl*251. The highest 16 bits (register de) are taken from this value to get hl*(251/256)
	xor a		; sub e from 0
	sub e
	ld a,l		; sub d from l
	sbc a,d
	ld e,a
	ld a,h		; sub c from h
	sbc a,c
	ld d,a		; rollAngleGyro*.98 is returned in de
	pop bc	; retrieve Y acceleration
	
	; ----- Step 3: Find weighted roll angle from accelerometer only -----
	
	; The accelerometer estimates roll angle by detecting the direction of the gravity vector.
	; When the drone isn't  accelerating, the only force acting on it is gravity. We use the
	; component of the "acceleration" vector parallel to the drone (horizontal when it's level)
	; to determine the angle. Each axis on the accelerometer is stored as a 16 bit signed int
	; (-32768 to 32767) and represents a value from -2g to 2g. The arcsin of (the parallel
	; component of acceleration divided by 16384 (1g)) gives us the estimated roll angle in
	; radians. This is multiplied by ~14667.7 to give us the 16 bit value. It's then multiplied
	; by (5/256) for weighting. All of this is just put into a lookup table though. Only the
	; 10 highest bits of the parallel acceleration value is used.
	
	ld a,b ; parallel force is limited from -1g to 1g so asin can be calculated
	or a ; cp 0
	jp z, rollAccPositive ; 0g to 1g
	cp 1
	jp z, rollAccPositiveMax ; 1g
	cp 3
	jp z, rollAccNegative ; -1g to 0g
 rollAccNegativeMax: ; -1g
	ld hl, $FE3E ; -90 degrees
	jp addRollEstimates
rollAccPositiveMax:
	ld hl, $01C2 ; 90 degrees
	jp addRollEstimates
rollAccPositive:
	ld hl, asinTable ; use lookup table
	ld b,0
	add hl,bc
	add hl,bc
	ld a,(hl)
	inc hl
	ld h,(hl)
	ld l,a
	jp addRollEstimates
rollAccNegative:
	xor a ; invert accelerometer value
	ld b,a
	sub c
	ld c,a
	ld hl, asinTable ; use lookup table
	add hl,bc
	add hl,bc
	ld a,(hl)
	inc hl
	ld b,(hl)
	ld c,a
	and a ; reset carry flag
	ld hl,0 ; invert roll angle result
	sbc hl,bc
	
	
	; ----- Step 4: Add the weighted values to find final roll angle estimate -----
addRollEstimates:
	add hl,de
	ret


; ---------- Calculate PID ----------
; Calculates the roll throttle signal using a PID controller that takes the roll error
; as an input. A low pass filter is used on the derivative term.

; Registers:
; HL				= Roll Error (input)
; A				= Roll Command (output)

	; ----- Calculate P term -----
	ex de,hl
	ld hl,PgainTable
	ld b,0
	ld c,d
	add hl,bc
	ld a,(hl)
;save a

	; ----- Calculate I term -----
	; ----- Calculate D term -----
; max initial value = ~575 if drone is 90 degrees at start and max gyro rate, about 3 rotations per second
; divide by ~7.6 I think, so max D value after gain is ~75
; if max value is limited to 236, max D value after gain is ~31
	ld hl,(rollDerivativeSum)
	ex de,hl ; now hl = rollError and de = rollDerivSum, don't change de until next "ex de,hl"
	and a ; reset carry flag
	sbc hl,de ; hl = out
	ld a,h ; don't change a until "cp $80"
	sra h
	rr l
	ex de,hl ; now hl = rollDerivSum and de = out/2
	add hl,de
	ld (rollDerivativeSum),hl
	sra d
	rr e ; de = out/4
	cp $02
	jp c, rollDerivInRange
	cp $FE
	jp nc, rollDerivInRange
	cp $80
	jp nc, rollDerivMaxNegative
rollDerivMaxPositive:
	ld e, 127
	jp rollDerivInRange
rollDerivMaxNegative:
	ld e, -128
rollDerivInRange:
	ld d,0
	ld hl, DgainTable
	add hl,de
	ld a,(hl)
	; then add a to saved a and integral result
	

	ld a,0 ; temporary
	ret



; ------------------------- Variables -------------------------
loopCounter: ; Increases by 1 each frame until it reaches 65,535
	.dw 0

rollReference:
	.dw 0
rollAngle:
	.dw 0
rollDerivativeSum:
	.dw 0

;import math
;for i in range(0, 257):
;    print(".dw $"+format(round(math.asin(i/256)*14667.7196*(5/256)),'#06x')[2:].upper())
    
asinTable:
	.dw $0000
	.dw $0001
	.dw $0002
	.dw $0003
	.dw $0004
	.dw $0006
	.dw $0007
	.dw $0008
	.dw $0009
	.dw $000A
	.dw $000B
	.dw $000C
	.dw $000D
	.dw $000F
	.dw $0010
	.dw $0011
	.dw $0012
	.dw $0013
	.dw $0014
	.dw $0015
	.dw $0016
	.dw $0018
	.dw $0019
	.dw $001A
	.dw $001B
	.dw $001C
	.dw $001D
	.dw $001E
	.dw $001F
	.dw $0021
	.dw $0022
	.dw $0023
	.dw $0024
	.dw $0025
	.dw $0026
	.dw $0027
	.dw $0028
	.dw $002A
	.dw $002B
	.dw $002C
	.dw $002D
	.dw $002E
	.dw $002F
	.dw $0030
	.dw $0031
	.dw $0033
	.dw $0034
	.dw $0035
	.dw $0036
	.dw $0037
	.dw $0038
	.dw $0039
	.dw $003B
	.dw $003C
	.dw $003D
	.dw $003E
	.dw $003F
	.dw $0040
	.dw $0041
	.dw $0043
	.dw $0044
	.dw $0045
	.dw $0046
	.dw $0047
	.dw $0048
	.dw $004A
	.dw $004B
	.dw $004C
	.dw $004D
	.dw $004E
	.dw $004F
	.dw $0051
	.dw $0052
	.dw $0053
	.dw $0054
	.dw $0055
	.dw $0056
	.dw $0058
	.dw $0059
	.dw $005A
	.dw $005B
	.dw $005C
	.dw $005D
	.dw $005F
	.dw $0060
	.dw $0061
	.dw $0062
	.dw $0063
	.dw $0065
	.dw $0066
	.dw $0067
	.dw $0068
	.dw $0069
	.dw $006B
	.dw $006C
	.dw $006D
	.dw $006E
	.dw $006F
	.dw $0071
	.dw $0072
	.dw $0073
	.dw $0074
	.dw $0075
	.dw $0077
	.dw $0078
	.dw $0079
	.dw $007A
	.dw $007C
	.dw $007D
	.dw $007E
	.dw $007F
	.dw $0080
	.dw $0082
	.dw $0083
	.dw $0084
	.dw $0085
	.dw $0087
	.dw $0088
	.dw $0089
	.dw $008B
	.dw $008C
	.dw $008D
	.dw $008E
	.dw $0090
	.dw $0091
	.dw $0092
	.dw $0093
	.dw $0095
	.dw $0096
	.dw $0097
	.dw $0099
	.dw $009A
	.dw $009B
	.dw $009D
	.dw $009E
	.dw $009F
	.dw $00A0
	.dw $00A2
	.dw $00A3
	.dw $00A4
	.dw $00A6
	.dw $00A7
	.dw $00A8
	.dw $00AA
	.dw $00AB
	.dw $00AD
	.dw $00AE
	.dw $00AF
	.dw $00B1
	.dw $00B2
	.dw $00B3
	.dw $00B5
	.dw $00B6
	.dw $00B8
	.dw $00B9
	.dw $00BA
	.dw $00BC
	.dw $00BD
	.dw $00BF
	.dw $00C0
	.dw $00C1
	.dw $00C3
	.dw $00C4
	.dw $00C6
	.dw $00C7
	.dw $00C9
	.dw $00CA
	.dw $00CC
	.dw $00CD
	.dw $00CF
	.dw $00D0
	.dw $00D2
	.dw $00D3
	.dw $00D5
	.dw $00D6
	.dw $00D8
	.dw $00D9
	.dw $00DB
	.dw $00DC
	.dw $00DE
	.dw $00DF
	.dw $00E1
	.dw $00E3
	.dw $00E4
	.dw $00E6
	.dw $00E7
	.dw $00E9
	.dw $00EB
	.dw $00EC
	.dw $00EE
	.dw $00F0
	.dw $00F1
	.dw $00F3
	.dw $00F5
	.dw $00F6
	.dw $00F8
	.dw $00FA
	.dw $00FC
	.dw $00FD
	.dw $00FF
	.dw $0101
	.dw $0103
	.dw $0104
	.dw $0106
	.dw $0108
	.dw $010A
	.dw $010C
	.dw $010E
	.dw $0110
	.dw $0112
	.dw $0114
	.dw $0116
	.dw $0118
	.dw $011A
	.dw $011C
	.dw $011E
	.dw $0120
	.dw $0122
	.dw $0124
	.dw $0126
	.dw $0128
	.dw $012A
	.dw $012D
	.dw $012F
	.dw $0131
	.dw $0134
	.dw $0136
	.dw $0138
	.dw $013B
	.dw $013D
	.dw $0140
	.dw $0142
	.dw $0145
	.dw $0148
	.dw $014A
	.dw $014D
	.dw $0150
	.dw $0153
	.dw $0156
	.dw $0159
	.dw $015C
	.dw $015F
	.dw $0163
	.dw $0166
	.dw $016A
	.dw $016E
	.dw $0172
	.dw $0176
	.dw $017A
	.dw $017F
	.dw $0184
	.dw $0189
	.dw $018F
	.dw $0196
	.dw $019E
	.dw $01A9

PgainTable: ; make sure this takes negative numbers into account
DgainTable: ; also this

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
;import math
;# 236
;isum = 0
;for i in range(0, 200):
;    i2 = i*577 # i2 is 16 bit signed
;    out = (i2-isum) # out is 16 bit signed
;    isum += math.floor(out/2) # use sra to sign extend out/2 from 15 to 16 bits
;    #print(out*0.0654499341)
;    print(out)
