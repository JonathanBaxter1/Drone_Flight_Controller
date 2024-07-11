; This program uses a PID controller to stabilize a quadcoptor on one axis.
; A complementary filter is used on the IMU accelerometer and gyro.

; This program has not been tested yet!

; ------------------------- CONTENTS -------------------------
; 1 - Setup
; 2 - Main Loop
; 3 - Functions
; 4 - Variables

; ------------------------- SETUP -------------------------
	.org $0000 ; Start of ROM, where program is stored
	
	; ---------- Delay Macros ----------
	; (used to time the Oneshot protocol)
	#define nops4 nop \ nop \ nop \ nop
	#define nops7 nops4 \ nop \ nop \ nop
	#define nops16 nops4 \ nops4 \ nops4 \ nops4
	#define nops64 nops16 \ nops16 \ nops16 \ nops16
	#define nops257 nops64 \ nops64 \ nops64 \ nops64 \ nop
	#define delay988cc ld b,75 \ djnz $-0 \ ld b,0 \ nop
	#define delay3330cc ld b,0 \ djnz $-0 ; cc means clock cycles
	
	;  ---------- Constants ----------

	; All 4 ESCs are calibrated when the program is started. A high signal is sent
	; for a few seconds while you plug in the ESCs. A low signal is then sent for a
	; few seconds, long enough to complete the calibration. The 1D PID balance test
	; then runs for the time specified in testLength.
	#define highLength 6 ; seconds
	#define lowLength 5 ; seconds
	#define testLength 10 ; seconds

	; Main loop frequency. This only affects the calibration timing.
	; To change the actual loop frequency, change delayLoopCount.
	#define loopFreq 500 ; Hz

	; A busy loop at the end of the main loop is used to tune the main loop frequency
	#define delayLoopCount 281 ; Delay clock cycles = delayLoopCount * 24cc + 10cc

	; All motors are at hoverThrottle when balanced
	#define hoverThrottle 66 ; 66/250
	
	; If the throttle of any motor exceeds this, an error occurs and the program stops.
	#define throttleLimit hoverThrottle+16+12+9+5 ; 108/250, ~43% Throttle
	
	; IMU Addresses
	#define yAccAddress $3D
	#define xGyroAddress $43
	
	; Roll angle complementary filter gains
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

	; The rest of the program is copied from ROM to RAM
	; so the self-modifying code used in the Oneshot protocol works.
	ld hl, RAMCopyStart
	ld de, $8000
	ld bc, RAMend - $8000
	ldir ; takes a maximum of 86ms @ 8 MHz
	
	jp programStart

RAMCopyStart:
	.org $8000 ; Start of RAM
	
; ALL CODE BELOW THIS POINT WILL BE LOADED INTO RAM
programStart:

; ------------------------- MAIN LOOP -------------------------
mainLoop:

	; ---------- Calculate motor throttles ----------
	
	; Increase loopCounter once per frame until it reaches 65,535
	; Then it stays at 65,535
	ld de,$0001                 ;	if (loopCounter < 65,535) {
	ld hl,(loopCounter)         ;		loopCounter++;
	add hl,de                   ;	}
	jp c, loopCounterAtMax
	ld (loopCounter),hl
loopCounterAtMax:

	; Find what test segment we're currently in
	ld hl,(loopCounter)
	ld de,-segment4Start        ;	if (loopCounter > segment4Start) {
	add hl,de                   ;		goto setThrottle0;
	jp c, setThrottle0          ;	}
	ld hl,(loopCounter)         ;	else if (loopCounter > segment3Start) {
	ld de,-segment3Start        ;		goto setThrottleBalance;
	add hl,de                   ;	}
	jp c, setThrottleBalance    ;	else if  (loopCounter > segment2Start) {
	ld hl,(loopCounter)         ;		goto setThrottle0;
	ld de,-segment2Start        ;	}
	add hl,de                   ;	else {
	jp c, setThrottle0          ;		goto setThrottle100;
	jp setThrottle100           ;	}
	
	; Set all 4 motors to min throttle (cuts motors off)
setThrottle0:
	xor a ; ld a,0
	ld (motor1Throttle),a
	ld (motor2Throttle),a
	ld (motor3Throttle),a
	ld (motor4Throttle),a
	jp endSetThrottle

	; Calculate throttle signals from PID controller
setThrottleBalance:
	call getSensorData
	call calculateOrientation

	ex de,hl                    ;	rollError = rollReference - rollAngle
	ld hl,(rollReference)
	and a ; reset carry flag
	sbc hl,de

	call calculatePID
	
	ld e, hoverThrottle         ;	motor1Throttle = motor2Throttle = hoverThrottle + rollCommand
	add a,e                     ;	motor3Throttle = motor4Throttle = hoverThrottle - rollCommand
	call checkThrottleLimit
	ld (motor1Throttle),a       ;	if (motor1Throttle >= throttleLimit || motor2Throttle >= throttleLimit) {
	ld (motor2Throttle),a       ;		handleError();
	sub e                       ;	}
	neg
	add a,e
	call checkThrottleLimit
	ld (motor3Throttle),a
	ld (motor4Throttle),a
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
	
	; Each Oneshot signal consists of 3 parts. First, a high signal is sent, then
	; there's a 125 us delay. After that is a buffer of 257 nop (no operation)
	; instructions, which also takes ~125 us. A "send low signal" instruction
	; is placed at a position in the buffer proportional to the throttle, which
	; allows for a pulse to be generated with any width from 125 - 250 us.

	;        Min throttle (0)       ;     Max throttle (250)
	;                               ;
	; |---125us---|                 ; |--------250us---------|
	; |           |                 ; |                      |
	; |           |                 ; |                      |
	; |           |                 ; |                      |
	;_|           |______________   ;_|                      |___
	;

	; Replace the previous "send low signal" instruction in the buffer with nop
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
	ld (hl),$ED ; out (c),c ($ED49) is the "send low signal" instruction
	inc hl		; the top 4 bits of c are cleared, which sends a low
	ld (hl),$49	; signal to each ESC
	
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
	
	; The Z80 can output an 8-bit value to upto 256 ports. However, this drone
	; only uses one output port, which can be accessed by writing to any port 0-255.

	;                            OUTPUTS
	; Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
	; ------|-------|-------|-------|-------|-------|-------|-------|
	; ESC 1 | ESC 2 | ESC 3 | ESC 4 |  SCL  | MOSI  |  NCS  |  LED  |

	; The same is true for inputs. Only 8 bits are used for input and any port 0-255
	; can be used. The SPI pins in the input and output are used for the IMU.

	;                             INPUTS
	; Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0 |
	; ------|-------|-------|-------|-------|-------|-------|-------|
	;  N/A  |  N/A  |  N/A  |  N/A  |  N/A  |  N/A  |  N/A  |  MISO |

	; Do oneshot125 for all 4 motors
	ld c,$02 ; Pull IMU chip select pin high when not being used
	ld a,$82 ; Pull ESC 1 high
	out (c),a
	delay988cc ; Pull ESC 1 low at some point in this buffer
Oneshot1:
	nops257 ; 257 since out (c),0 is 2 bytes, max PWM value = 250 (for now)
	ld a,$42 ; Pull ESC 2 high
	out (c),a
	delay988cc ; Pull ESC 2 low at some point in this buffer
Oneshot2:
	nops257
	ld a,$22 ; Pull ESC 3 high
	out (c),a
	delay988cc ; Pull ESC 3 low at some point in this buffer
Oneshot3:
	nops257
	ld a,$12 ; Pull ESC 4 high
	out (c),a
	delay988cc ; Pull ESC 4 low at some point in this buffer
Oneshot4:
	nops257

	; Temporary delay that makes sure the control loop runs at ~500hz.
	; I'll be lucky if I keep this in the final program, but it'll probably end up
	; being below 500hz.
	ld de, delayLoopCount ; 10cc
delayLoop:
	dec de ; 6cc
	ld a,d ; 4cc
	or e ; 4cc
	jp nz, delayLoop ; 10cc

	jp mainLoop

; ------------------------- FUNCTIONS -------------------------

; ---------- Get Data from IMU  ----------
; Get accelerometer and gyroscope data from IMU (MPU-9250).

; Registers:
; BC        = Y acceleration (output)
; E         = X angular rate (output)
; A, F, D   = Destroyed
; H, L      = Unaffected
getSensorData:
	ld d,xGyroAddress + $80 ; bit 7 of IMU address means we're reading
	call readDataSPI8Bit ; read 8 bits from gyro
	ld a,$02 ; Turn off IMU
	out (0),a ; SCL = 0, MOSI = 0, NCS = 1, MISO = N/A
	ld e,c
	ld d,yAccAddress + $80
	call readDataSPI8Bit ; read 10 bits from accelerometer
	call readDataSPI2Bit
	ld a,$02 ; Turn off IMU
	out (0),a ; SCL = 0, MOSI = 0, NCS = 1, MISO = N/A
	ret

; ---------- Read Data from SPI Device (8 bit) ----------
; Reads 8 bits of data from an SPI device.

; Registers:
; D         = Address (input)
; C         = Data (output)
; A, F, B   = Destroyed
; E, H, L   = Unaffected
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
; C             = data (input)
; BC            = data (output)
; A, F          = Destroyed
; D, E, H, L    = Unaffected
readDataSPI2Bit:
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
; BC        = Y acceleration (input)
; E         = X angular rate (input)
; HL        = Roll Angle (output)
; A, F, D   = Destroyed
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
	ld a,e                      ;	rollAngleGyro = rollAngle + xGyro
	add a,a
	sbc a,a ; sign extend e into de, then add de to hl
	ld d,a
	add hl,de	
	
	; ----- Step 2: Find weighted value from gyro roll angle estimate -----
	
	; Like in the video, I multiply rollAngleGyro by .98 before it's added to the weighted
	; accelerometer measurement (multiplied by .02 so they add up to 1). This is literally
	; what the following section of code does:
	;		rollAngleGyro = rollAngleGyro*.98
	; but for some reason I wanted to use a processor from 1976 that can't do floating point
	; operations or even multiplication for that matter, so that's why it took 32 lines of code.
	; Due to the Z80's limitations, rollAngleGyro is actually multiplied by (251/256), which is
	; pretty close to .98 (and rollAngleAcc is multiplied by (5/256) which is ~.02).
	
	; First, hl*5 is loaded into the 24 bit register group "cde"
	push bc	; save Y acceleration
	ld bc, 0                    ;	if (hl >= 0) {	// This is necessary to make the operation work with negative roll angles
	bit 7,h                     ;		bc = $0000
	jp z, HLpositive            ;	else {
	dec bc                      ;		bc = $FFFF
HLpositive:                     ;	}
	ld d,h      ; load hl into cde, then left bitshift twice to multiply by 4
	ld e,l
	sla e       ; bitshift 1
	rl d
	rl c
	sla e       ; bitshift 2
	rl d
	rl c
	ld a,e      ; add hl to cde once more to make cde = hl*5
	add a,l     ; ^add l and e
	ld e,a
	ld a,d      ; add h and d
	adc a,h
	ld d,a
	ld a,c      ; add b and c (register b is used so negative hl values work)
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
	pop bc		; retrieve Y acceleration
	
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
	add hl,bc ; add offset twice since values are 16 bit
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
	add hl,bc ; add offset twice since values are 16 bit
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
; HL            = Roll Error (input)
; A             = Roll Command (output)
; BC, DE, F     = Destroyed

	; ----- Calculate P term ----- (Max value ~= 16)
	; P gain = .04, input is always from -90 to 90
	; Gain table multiplier = *0.17454148769

	; REGISTER B MUST BE 0 AT START OF INTEGRAL CALCULATION!!!
	ex de,hl
	ld hl,PgainTable
	ld b,0
	ld c,d
	add hl,bc
	ld a,(hl)
	ld ixl,a                    ;	rollCommand = Pterm

	; ----- Calculate I term ----- (Max value ~= 12)
	; I gain = .01, input from -128 to 127
	; Gain table multiplier = *0.0893652414
	; The integral must not change more than 65536 in a single frame for this function to work.
	; This shouldn't be a problem since only an 8 bit value is added each frame.

	; REGISTER DE MUST BE PRESERVED UNTIL DERIVATIVE CALCULATION!!!
	ld hl,(rollIntegralLow2Bytes)
	ld a,(rollIntegralHighByte)
	add hl,bc
	adc a,b ; just adds carry flag to a since b is 0
	cp $FD
	jp z,rollIntMaxNegative
	cp $02
	jp z,rollIntMaxPositive
	jp rollIntInRange
rollIntMaxNegative:
	ld a,$FE
rollIntMaxPositive:
	ld hl,0
rollIntInRange:
	ld (rollIntegralLow2Bytes),hl
	ld (rollIntegralHighByte),a
	sra a
	rr h
	sra a
	rr h
	sra a
	rr h
	ld c,h
	ld hl,IgainTable
	add hl,bc ; just adds c since b is 0
	ld a,(hl)
	ld b,ixl
	add a,b
	ld b,a                      ;	rollCommand += Iterm
	
	; ----- Calculate D term ----- (Max value = 32)
	; D gain = .015, input from -128 to 127, Max value @ 127 ~= 37, but limited to 32
	; Gain table multiplier = *0.288134375
	; max initial value = ~575 if drone is 90 degrees at start and max gyro rate, about 3 rotations per second
	ld hl,(rollDerivativeSum)
	ex de,hl ; now hl = rollError and de = rollDerivSum, DON'T CHANGE DE UNTIL NEXT "ex de,hl"
	and a ; reset carry flag
	sbc hl,de ; hl = out
	ld a,h ; DON'T CHANGE REG A UNTIL "cp $80"
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
	add a,b						;	rollCommand += Dterm
	
	ret


; ---------- Check Throttle Limit ----------
; If the throttle exceeds the specified limit, the throttle signals to the
; ESCs will be cut off (constant LOW state). According to the ESC documentation
; at https://www.lemonfpv.com/h-pd-298.html, after 0.32 seconds the ESCs will
; cut off power to the motors. The "Handle Error" function is called which
; stops the program and flashes the LED.

; Registers:
; A             = Throttle (input)
; All Others    = Unaffected (unless handleError is called)

checkThrottleLimit:
	cp throttleLimit
	call nc, handleError
	ret


; ---------- Handle Error ----------
; If an error in the program is found, the program will stop, and the
; LED will flash indicating to the user to unplug the power from the Z80
; and plug it back in.

; Registers:
; Doesn't really matter since the program gets stuck in this loop forever

handleError:
	ld a,$01
	out (0),a ; Set LED to HIGH
	ld de,1192 ; Delay 500 ms @ 8 MHz
errorLoop1:
delay3330cc
	dec de
	ld a,d
	or e
	jp nz,errorLoop1
	xor a
	out (0),a ; Set LED to LOW
	ld de,1192 ; Delay 500 ms @ 8 MHz
errorLoop2:
delay3330cc
	dec de
	ld a,d
	or e
	jp nz,errorLoop2
	jp handleError


; ------------------------- VARIABLES -------------------------
loopCounter:
	.dw 0
rollReference:
	.dw 0
rollAngle:
	.dw 0
rollIntegralLow2Bytes:
	.dw 0
rollIntegralHighByte:
	.db 0
rollDerivativeSum:
	.dw 0
motorThrottles:
motor1Throttle:
	.db 0
motor2Throttle:
	.db 0
motor3Throttle:
	.db 0
motor4Throttle:
	.db 0

asinTable:
	.dw $0000, $0001, $0002, $0003, $0004, $0006, $0007, $0008, $0009, $000A, $000B, $000C, $000D, $000F, $0010, $0011
	.dw $0012, $0013, $0014, $0015, $0016, $0018, $0019, $001A, $001B, $001C, $001D, $001E, $001F, $0021, $0022, $0023
	.dw $0024, $0025, $0026, $0027, $0028, $002A, $002B, $002C, $002D, $002E, $002F, $0030, $0031, $0033, $0034, $0035
	.dw $0036, $0037, $0038, $0039, $003B, $003C, $003D, $003E, $003F, $0040, $0041, $0043, $0044, $0045, $0046, $0047
	.dw $0048, $004A, $004B, $004C, $004D, $004E, $004F, $0051, $0052, $0053, $0054, $0055, $0056, $0058, $0059, $005A
	.dw $005B, $005C, $005D, $005F, $0060, $0061, $0062, $0063, $0065, $0066, $0067, $0068, $0069, $006B, $006C, $006D
	.dw $006E, $006F, $0071, $0072, $0073, $0074, $0075, $0077, $0078, $0079, $007A, $007C, $007D, $007E, $007F, $0080
	.dw $0082, $0083, $0084, $0085, $0087, $0088, $0089, $008B, $008C, $008D, $008E, $0090, $0091, $0092, $0093, $0095
	.dw $0096, $0097, $0099, $009A, $009B, $009D, $009E, $009F, $00A0, $00A2, $00A3, $00A4, $00A6, $00A7, $00A8, $00AA
	.dw $00AB, $00AD, $00AE, $00AF, $00B1, $00B2, $00B3, $00B5, $00B6, $00B8, $00B9, $00BA, $00BC, $00BD, $00BF, $00C0
	.dw $00C1, $00C3, $00C4, $00C6, $00C7, $00C9, $00CA, $00CC, $00CD, $00CF, $00D0, $00D2, $00D3, $00D5, $00D6, $00D8
	.dw $00D9, $00DB, $00DC, $00DE, $00DF, $00E1, $00E3, $00E4, $00E6, $00E7, $00E9, $00EB, $00EC, $00EE, $00F0, $00F1
	.dw $00F3, $00F5, $00F6, $00F8, $00FA, $00FC, $00FD, $00FF, $0101, $0103, $0104, $0106, $0108, $010A, $010C, $010E
	.dw $0110, $0112, $0114, $0116, $0118, $011A, $011C, $011E, $0120, $0122, $0124, $0126, $0128, $012A, $012D, $012F
	.dw $0131, $0134, $0136, $0138, $013B, $013D, $0140, $0142, $0145, $0148, $014A, $014D, $0150, $0153, $0156, $0159
	.dw $015C, $015F, $0163, $0166, $016A, $016E, $0172, $0176, $017A, $017F, $0184, $0189, $018F, $0196, $019E, $01A9

PgainTable:
	.db $00, $00, $00, $01, $01, $01, $01, $01, $01, $02, $02, $02, $02, $02, $02, $03
	.db $03, $03, $03, $03, $03, $04, $04, $04, $04, $04, $05, $05, $05, $05, $05, $05
	.db $06, $06, $06, $06, $06, $06, $07, $07, $07, $07, $07, $08, $08, $08, $08, $08
	.db $08, $09, $09, $09, $09, $09, $09, $0A, $0A, $0A, $0A, $0A, $0A, $0B, $0B, $0B
	.db $0B, $0B, $0C, $0C, $0C, $0C, $0C, $0C, $0D, $0D, $0D, $0D, $0D, $0D, $0E, $0E
	.db $0E, $0E, $0E, $0E, $0F, $0F, $0F, $0F, $0F, $10, $10, $10, $10, $10, $10, $10
	.db $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10
	.db $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10, $10
	.db $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0
	.db $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0
	.db $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F0, $F1, $F1, $F1, $F1, $F1, $F2, $F2, $F2
	.db $F2, $F2, $F2, $F3, $F3, $F3, $F3, $F3, $F3, $F4, $F4, $F4, $F4, $F4, $F4, $F5
	.db $F5, $F5, $F5, $F5, $F6, $F6, $F6, $F6, $F6, $F6, $F7, $F7, $F7, $F7, $F7, $F7
	.db $F8, $F8, $F8, $F8, $F8, $F8, $F9, $F9, $F9, $F9, $F9, $FA, $FA, $FA, $FA, $FA
	.db $FA, $FB, $FB, $FB, $FB, $FB, $FB, $FC, $FC, $FC, $FC, $FC, $FD, $FD, $FD, $FD
	.db $FD, $FD, $FE, $FE, $FE, $FE, $FE, $FE, $FF, $FF, $FF, $FF, $FF, $FF, $00, $00

IgainTable:
	.db $00, $00, $00, $00, $00, $00, $01, $01, $01, $01, $01, $01, $01, $01, $01, $01
	.db $01, $02, $02, $02, $02, $02, $02, $02, $02, $02, $02, $02, $03, $03, $03, $03
	.db $03, $03, $03, $03, $03, $03, $03, $03, $04, $04, $04, $04, $04, $04, $04, $04
	.db $04, $04, $04, $05, $05, $05, $05, $05, $05, $05, $05, $05, $05, $05, $06, $06
	.db $06, $06, $06, $06, $06, $06, $06, $06, $06, $07, $07, $07, $07, $07, $07, $07
	.db $07, $07, $07, $07, $08, $08, $08, $08, $08, $08, $08, $08, $08, $08, $08, $08
	.db $09, $09, $09, $09, $09, $09, $09, $09, $09, $09, $09, $0A, $0A, $0A, $0A, $0A
	.db $0A, $0A, $0A, $0A, $0A, $0A, $0B, $0B, $0B, $0B, $0B, $0B, $0B, $0B, $0B, $0B
	.db $F5, $F5, $F5, $F5, $F5, $F5, $F5, $F5, $F5, $F5, $F5, $F6, $F6, $F6, $F6, $F6
	.db $F6, $F6, $F6, $F6, $F6, $F6, $F7, $F7, $F7, $F7, $F7, $F7, $F7, $F7, $F7, $F7
	.db $F7, $F8, $F8, $F8, $F8, $F8, $F8, $F8, $F8, $F8, $F8, $F8, $F8, $F9, $F9, $F9
	.db $F9, $F9, $F9, $F9, $F9, $F9, $F9, $F9, $FA, $FA, $FA, $FA, $FA, $FA, $FA, $FA
	.db $FA, $FA, $FA, $FB, $FB, $FB, $FB, $FB, $FB, $FB, $FB, $FB, $FB, $FB, $FC, $FC
	.db $FC, $FC, $FC, $FC, $FC, $FC, $FC, $FC, $FC, $FD, $FD, $FD, $FD, $FD, $FD, $FD
	.db $FD, $FD, $FD, $FD, $FD, $FE, $FE, $FE, $FE, $FE, $FE, $FE, $FE, $FE, $FE, $FE
	.db $FF, $FF, $FF, $FF, $FF, $FF, $FF, $FF, $FF, $FF, $FF, $00, $00, $00, $00, $00

DgainTable:
	.db $00, $00, $01, $01, $01, $01, $02, $02, $02, $03, $03, $03, $03, $04, $04, $04
	.db $05, $05, $05, $05, $06, $06, $06, $07, $07, $07, $07, $08, $08, $08, $09, $09
	.db $09, $0A, $0A, $0A, $0A, $0B, $0B, $0B, $0C, $0C, $0C, $0C, $0D, $0D, $0D, $0E
	.db $0E, $0E, $0E, $0F, $0F, $0F, $10, $10, $10, $10, $11, $11, $11, $12, $12, $12
	.db $12, $13, $13, $13, $14, $14, $14, $14, $15, $15, $15, $16, $16, $16, $16, $17
	.db $17, $17, $18, $18, $18, $18, $19, $19, $19, $1A, $1A, $1A, $1B, $1B, $1B, $1B
	.db $1C, $1C, $1C, $1D, $1D, $1D, $1D, $1E, $1E, $1E, $1F, $1F, $1F, $1F, $20, $20
	.db $20, $20, $20, $20, $20, $20, $20, $20, $20, $20, $20, $20, $20, $20, $20, $20
	.db $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0, $E0
	.db $E0, $E0, $E0, $E1, $E1, $E1, $E1, $E2, $E2, $E2, $E3, $E3, $E3, $E3, $E4, $E4
	.db $E4, $E5, $E5, $E5, $E5, $E6, $E6, $E6, $E7, $E7, $E7, $E8, $E8, $E8, $E8, $E9
	.db $E9, $E9, $EA, $EA, $EA, $EA, $EB, $EB, $EB, $EC, $EC, $EC, $EC, $ED, $ED, $ED
	.db $EE, $EE, $EE, $EE, $EF, $EF, $EF, $F0, $F0, $F0, $F0, $F1, $F1, $F1, $F2, $F2
	.db $F2, $F2, $F3, $F3, $F3, $F4, $F4, $F4, $F4, $F5, $F5, $F5, $F6, $F6, $F6, $F6
	.db $F7, $F7, $F7, $F8, $F8, $F8, $F9, $F9, $F9, $F9, $FA, $FA, $FA, $FB, $FB, $FB
	.db $FB, $FC, $FC, $FC, $FD, $FD, $FD, $FD, $FE, $FE, $FE, $FF, $FF, $FF, $FF, $00

RAMend: