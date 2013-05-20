;-----------------------------------------------------------------------------
;  Copyright (C) 2006 by Takao.
;  All rights reserved.
;  CURRENT_LIMITER:	NOT AVAILABLE CURRENT_LIMITER
;
;  FILE NAME   :	BL_ESC_100A.ASM July. 30th. '05
;  FILE NAME   :	BL_ESC_100Aa.ASM Aug. 15th. '05
;  FILE NAME   :	BL_ESC_100A_LDP.ASM Aug. 25th. '05
;  FILE NAME   :	BL_ESC_100A_LDP.ASM OCt. 10th. '05 for 1mm dia. x 23T 6-teeth 8P LDP motor
;  FILE NAME   :	BL_ESC_100A_LDP.ASM Oct. 30th. '05 for 0.7mm dia. x 2s x 15T LDP motor
;  FILE NAME   :	BL_ESC_100A2.ASM Jan. 1st. '06 for 0.7mm dia. x 2s x 15T LDP motor
;  FILE NAME   :	BL_ESC_100A3.ASM Feb. 13. '06 for AXI 2820/12 motor
;  FILE NAME   :	100A series. BL_ESC_100A_307A.ASM, Apr. 21th '06
;
;		Starting BEMF on-duty is 25% at INC_STEPPING_START_ON_DUTY:
;
;		ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER checkes max rpm in BEMF mode
;		for smooth change to zero cross mode
;		BEMF_SAMPLING_COUNTER checks slow down from zero cross to BEMF mode
;		to avoid under BEMF sampling.
;
;  TARGET MCU  :	C8051F330D
;  DESCRIPTION :  	Quick power on buzzer sound
;		+ Start with Position detection + minimize the start current
;		+ BEMF for start run(BEMF_START_SPIN:)
;		+ reverse spin check(REVERSE_SPIN:)
;		+ BEMF power controlled run(BEMF_START0:)
;		+ Zero Cross switching 50%-100% power drive
;		+ Motor stop protection by WDT + Plane find motor buzzer
;		+ 5V minimum power voltage keeper + Current Limitter
;
; 	I hope that	No need to adjust SAMPLING_TIMING by measuring BEMF OFF-DUTY inductive kick time
;	I hope that	No need to adjust START_CURRENT_ON_DUTY by minimize the start current routine
;
; 		Keep run if the power on time is over REAR_EDGE_TO_ZC_POINT by back to the BEMF mode
;		Keep a few second run mode when stick off for PPM reciever
;
;
;  NOTES:		START_ON_DUTY_CONTROL: CloopX timing is chenged to 500nS as 2 times faster
;		BEMF for start run(BEMF_START_SPIN:) with drive state sequence check to avoid reverse start spin
;		BEMF<->ZeroCross control BEMF_FULL_ON is adjusted to 75% by BEMF high sampling rate at OFF_DUTY:
;		sound is just one beep for quick start.
;		The Spin current limitter function with LM339
;		5V power supply volatge keeper is added by D/A.
;
$include (c8051f330.inc)                  ; Include register definition file.
;-----------------------------------------------------------------------------
; EQUATES Parameters
;-----------------------------------------------------------------------------
REAR_EDGE_TO_ZC_POINT equ 5
		; This value concerns with zero cross max speed stability.  
		; Safty Margin of POWER on Time of the rear edge od inductive kick to zero cross point.
		; May be "7" is for low voltage CD-ROM motor drive, may be "2" is good for high speed spin as ducted fan.
		; This value is (T2) Sync out as "COULD_NOT_CATCH_CROSS_POINT" in zero cross mode.
		; The time should be less than 500uS to get until zero coross point after inductive kick rear edge.
IND_KICK_TIME_CHECK equ 3 ; 490nS x 255 x this value
		; The inductive kick time is checked by T2 high. and if the time is over this value.
		; It means could not catch the indutive kick as sync. out.  	

BEMF_Duty_Inc_Timing equ 10 ; BEMF power step up increasing timing frame on Timer 2

;Starting parameters
START_STEPPING_DUTY_FRAME equ 80	; Start duty checking Duty frame include shaft position routine 
START_CURRENT_ON_DUTY equ 20	; Init Current limit setting in on-duty setting 
INC_START_CURRENT_ON_DUTY_STEP equ 5 ; Start power increasing step up while could not spin  
STEPPING_ON_DUTY_CHOPPING_COUNT	equ 35	; Init Stepping starting current limit chopping count

END_OF_STEPPING_START_CNT equ 5	; max starting try count of STEPPING_START. This is for motor burn-up protection at starting over load.

BEMF_start_run_on_duty_time_after_stepping_drive equ 50	; Setting first BEMF spin torque
BEMF_start_run_off_duty_time_after_stepping_drive equ 250

SAMPLE_COUNT equ 6	; minimum sampling count at one phase to avoid mis-fire in BEMF mode.
SAMPLING_TIMING equ 10	; sampling frq. for default timing(max. 20) in BEMF mode. This value also concerns starting current.
BEMF_MAX_ON_DUTY equ 180	; BEMF max On duty. OFF duty= BEMF_ON_DUTY - (ON_DUTY x2)
;
SLOW_START 	equ THROTTLE_OFF+STICK_BACK_HYS	; BEMF lowest power runs with this low power duty(%).

THROTTLE_OFF equ 10	; Throttle off stick position data

MISFIRE_COUNT equ 50	; Noise filter count to ensure the reverse spin

Slowest_Spin_Speed_Check_Time equ 60	; SPIN_FREE check time value. Used Timer0 high

FIND_TIMER_DELAY equ 36	; find timer start delay after throttle off(ex.value 24: 24 x 8sec=3min)

;RX_NOISE_TIM_CNT equ 10	; the Rx noise as throttle off signal timer counter setting data
STICK_BACK_HYS equ 2 ; return back stick hyteresis to avoid RX signal jitter

MIN_OFFSET	equ 1150	; uS. minimum control pulse width offset
CTRL_MIN	equ 2347	; *MIN_OFFSET/0.49(=12/SYSCLK); number of timer counts PCA 1/12 sys.clk in 
		; capture range sample(0800h - 1065h)

ZERO_CROSS_INIT_STEPS equ 7	; initial ZC_STEPS value as stick to ou-duty time in zerocross mode. 
ZC_SAMPLING_DUTY_INC_TIMING equ 10	; power-step up increasing timing on Timer 3 in zerocross mode.
V_ERR_MAX	equ 200	; The voltage error detection value for V_ERR_CNT

MAX_BEMF_RPM_DATA_CHECK_HIGHER equ 2 ; <<<<<<<
;-----------------------------------------------------------------------------
RX_IN   	equ P0.7	; Control pulse input from Rx.

COMP_D	equ R0	; Comparator result strage
		; R1 is partially used for find buzzer timer, 
		; timing advance counter in full on mode
		; and single sample duty calc.
	;R2	; Partially used in sound and stepping start sequence, single sample duty calc. 
	;R1	; timing advance high byte data strage
	;R2	; timing advance low byte data strage
TEMP3	equ R3
RX_ERROR_COUNT equ R4	; RX throttle off signal counter to avoid Rx noise motor shut off
ON_DUTY	equ R5	; ON duty control data storage
TEMP0	equ R6	; 
		; BRAKING_TIMER is used for braking timer
;TEMP equ R7		; Just temporally register

;STACKs

IO_SETTING	equ 01ah	; P1 setting storage while motor 3ST OFF to Re-On in BEMF running
DIVIDED_L	equ 01bh	; Timer low byte storage for devide calc
DIVIDED_H	equ 01ch	; Timer high byte storage for devide calc 
P0_IO	equ 01dh	; P0 IO storage used in stepping sequence

STEPPING_ON_DUTY equ 01eh	; Used for starting on-duty timing up with current limit function
STEPPING_ON_DUTY_CHOPPING_COUNTER	equ 01fh	; Used for starting current limited on-duty timing

PCA0_CAP_L	equ 020h	; PCA low byte holding register at the rising edge of speed control input from Rx
PCA0_CAP_H	equ 021h	; PCA high byte holding register at the rising edge of speed control input from Rx
PULSE_WIDTH_L equ 022h	; measured pulse width low byte data storage(12bits resolution)
PULSE_WIDTH_H equ 023h	; measured pulse width high byte data storage(12bits resolution)
CONTROL_DATA_L equ 024h	; The data storage of checked low byte of 12bit resolution input control pulse width
CONTROL_DATA_H equ 025h	; The data storage of checked High byte of 12bit resolution input control pulse width
CONTROL_DATA equ 026h	; The data storage of 8bit resolution input control pulse width

SAMPLING_COUNTER_L equ 027h	; sampling counter at one phase drive
SAMPLING_COUNTER_H equ 028h
TEMP_MEMORY equ 029h	; Just TEMP memory
BRAKING_TIMER equ 02ah		; is used for braking timer
			; On/Off duty timing data memory in BEMF mode
RX_NOISE_FILTER_COUNTER equ 02bh; SPIN COUTER for ZC => BEMF mode Rx noise filtering  
Rx_Noise_Count equ 20	; Almost 0.1 sec noise count to avoid immediate stop for PPM noisy Rx

TMR1L equ 030h	; Storage space for when single sample to BEMF sampling back 1 phase frame timing data 
TMR1H equ 031h

INPUT_COUNTER_L equ 032h	; This counter is used for input signal loss check
INPUT_COUNTER_H equ 033h	; This counter is used for input signal loss check

ROUND_CHK_FLAG equ 034h
PREV_COMP_D	equ 035h

;T2_LOWEST_L	equ 036h	; BEMF max speed one phase spin timing as single sample min. spin timing
;T2_LOWEST_H	equ 037h	; BEMF max speed one phase spin timing as single sample min. spin timing 

BEMF_OFF_EXT_TIMER equ 038h
BEMF_FULL_ON equ 039h

SPIN_COUNTER equ 03ah		; Spin counter 

T2_ON_DUTY_LOW equ 03bh		; set on_duty timer low storage SS mode RAM  
T2_ON_DUTY_HIGH equ 03ch	; set on_duty timer high storage SS mode RAM 

STORAGE_B equ 03dh	; Stack for AccB

SPIN_MODE equ 03eh	; BEMF START mode(unstable):3
		; BEMF stable run mode(as 5 or less BEMF samples in one state):2
		; BEFM very high speed critical mode(as one BEMF sample in one state):1
		; Zero Cross mode:0

DRV_SEQUENCE_CHECK_ENABLE equ 03fh	; flag
CURRENT_LIMITER_ENABLE_FLAG equ 040h	; "1" Eanble, "0" Disable for BEMF<->Zerocross one shot rush current enable
 
ZC_STEPS	equ 041h	; zero cross stick timing resolution
COMP1_A	equ 042h	; comparator result temporaly storage
COMP1_B	equ 043h	; comparator result temporaly storage
COMP1_C	equ 044h	; comparator result temporaly storage

;ZC_BREAK_IN_COUNTER equ 045h	; BEMF Duty 50% run to stable half power before starting ZC control.

STEP_START_FLAG	equ 046h	; Stepping start flag to control small on-duty just after stepping start.

ZC_OFF_DUTY_L equ 047h
ZC_OFF_DUTY_H equ 048h	

;NEED_MORE_T_ADVANCE_FLAG equ 049h

BEMF_OFF_DUTY_TIME_L	equ 04ah	; used in BEMF power timing check
BEMF_OFF_DUTY_TIME_H	equ 04bh	; used in BEMF power timing check
BEMF_DUTY_TIME_L	equ 04ch	; used in BEMF power timing check
BEMF_DUTY_TIME_H	equ 04dh	; used in BEMF power timing check

;ADVANCE_ENABLE equ 04ch	; used for Timing advance enable flag.
BEMF_PWR_OVER_CHECK_COUNTER equ 04eh

ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER equ 04fh	; used for BEMF increasing speed up check enable flag

MISFIRE_COUNTER equ 050h	; mis-fire counter as reverse spin.

PACK	equ 051h	; 2 or 3 Li-Po Package detection result(2=2PACK, 3=3PACK)
AD_SEQUENCE equ 052h	; Voltage and temprature check ad conversion sequence flag
CONTROL_DATA_MAX equ 053h ; for decreasing stick power limit at Li-Po voltage down and/or Li-Po or CPU over heat

FIND_TIMER equ 054h	; plane find buzzer timer counter

TMR2M_H	equ 056h	; rear edge of inductive kick T2 high data storage to check the weak motor generative volage p-p level in zero cross
TMR2M_L	equ 057h	; rear edge of inductive kick T2 low data storage to check the weak motor generative volage p-p level in zero cross

LIPO_T_ERR_CNT	equ 058h	; the over heat counter
W_TERMINAL_V_CHK_FLAG	equ 059h	; check Vdd line voltage throough P-ch W-state MOSFET ON
TOP_V_ERR_CNT	equ 05ah	; the top cell voltage down counter
V2_ERR_CNT	equ 05bh	; The GND to cell2 volatge drop counter to avoid shot voltage dip.
CHK_TOP_V_SEQUENCE equ 05ch	;top cell as pack voltage check sequence counter

TOP_V_LOW equ 05dh	; Top voltage low data stored here
TOP_V_HIGH equ 05eh	; Top voltage high data stored here

V1_ERR_CNT	equ 05fh	; The bottom cell volatge drop counter to avoid shot voltage dip.
V3_ERR_CNT	equ 060h	; The Top cell volatge drop counter to avoid shot voltage dip in 3S mode.
MID_CELL_DIP_CNT equ 061h	; bottom cell to middle cell volatge drop counter to avoid shot voltage dip.

DRIVE_STATE equ 062h	; drive state storage 
START_DRIVE_STATE equ 063h	; 
PREV_DRIVE_STATE equ 064h	; Previous drive state storage 
PREV_DRV_STATE equ 065h	; previous drive state storage in stepping start control
NEXT_DRV_STATE equ 066h	; next drive state storage in stepping start control fpr reverse spin check
NEXT_NEXT_DRV_STATE equ 067h	; next next drive state storage in stepping start control fpr reverse spin check
COMP_REMEASURE_COUNTER equ 068h

COMP_PD	equ 070h	; comparator previous data strage to avoid oscillation
TEMP1	equ 071h	; Used in timing delay routine
TEMP_L	equ 072h	; 16 bit calc temporaly storage low
TEMP4	equ 074h	; Used in Rx pulse width counter in interrup routine 
TEMP5	equ 075h
TEMP_H	equ 076h	; 16 bit calc temporaly storage low
TEMP7	equ 077h	; 1st BEMF running(bit0), brake ON flag(bit1), BEMP direction check flag(bit2,3)
		; # of sampling in one phase is OK=1 flag(bit4), over mask time flag(bit5), 
		; ZERO cross timing is OK=1 fkg(bit6), BAD comparator flag(bit7)
DUTY	equ 078h	; Temp. Used in BEMF run power duty control timer routine
IO_MODE_M	equ 079h	; P1 mode setting storage while motor 3ST OFF to Re-On in BEMF running 

;-----------------------------------------------------------------------------
; RESET and INTERRUPT VECTORS
;-----------------------------------------------------------------------------
;Reset Vector
            cseg AT 0

	ljmp Main	; Locate a jump to the start of code at the reset vector.

	org	005bh
            ljmp	PCA0_PLS	; Uses about 6uS. PCA0 Mode0 plus pulse width count capture interrupt
;-----------------------------------------------------------------------------
; CODE SEGMENT
;-----------------------------------------------------------------------------
BL_ESC      segment  CODE
	rseg BL_ESC	; Switch to this code segment.
	using 0	; Specify register bank for the following program code.

Main:
;	 Disable the WDT.
            anl PCA0MD, #NOT(040h)  ; clear Watchdog Enable bit and 1/12 system clock usage for PCA

	mov OSCICN, #10000011b	; make 24.5MHz run


;-------------------------------------
INIT:	mov PULSE_WIDTH_L,#0	; Init. control pulse data
	mov PULSE_WIDTH_H,#0
	mov CONTROL_DATA, #0	; Init. Rx input pulse width data
	mov TEMP7, #0		; 1st BEMF running(bit-0) and brake ON flag(bit-1) clear	

	mov RX_ERROR_COUNT, #0
	mov ROUND_CHK_FLAG, #0	; Clear the BEMF running monitor flag

	mov TMR3RLL, #0
	mov TMR3RLH, #0

	mov BEMF_FULL_ON, #BEMF_MAX_ON_DUTY	;
	
	mov DIVIDED_H, #255
	orl TMR3CN, #00000100b	; start T3 count-up used in BEMF 50% power timing check and zero cross power increase timing
	mov TMOD, #00010001b	; Set Timer0,1 16 bit counter mode
	mov DRV_SEQUENCE_CHECK_ENABLE, #0	; starting routine uses complimentary sequence to avoid shaft spin	
	mov ZC_STEPS, #ZERO_CROSS_INIT_STEPS
	mov TEMP_L, #255
	mov TEMP_H, #255
	;mov CONTROL_DATA_MAX, #THROTTLE_OFF+2
	mov CONTROL_DATA_MAX, #255
	mov TMR1L, #255
	mov TMR1H, #255

	mov AD_SEQUENCE, #0
	mov LIPO_T_ERR_CNT, #0
	mov V1_ERR_CNT, #0
	mov V2_ERR_CNT, #0
	mov V3_ERR_CNT, #0
	mov W_TERMINAL_V_CHK_FLAG, #0
	mov CHK_TOP_V_SEQUENCE, #0
	mov CURRENT_LIMITER_ENABLE_FLAG, #1	; Enable
	mov MISFIRE_COUNTER, #0
	;mov ZC_BREAK_IN_COUNTER, #0
	;mov NEED_MORE_T_ADVANCE_FLAG, #0
	mov ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER, #127	; Checking BEMF max speed enable (avaraging) flag
	mov BEMF_PWR_OVER_CHECK_COUNTER, #0
;--------------------
	mov PCA0CPL2, #255	; wathc dog timing is 32mS interval
	orl PCA0MD, #11000000b	; PCA suspend while CPU is supended and WDT is "ON".

	;clr P2.0		; Test pin "L"
	setb P2.0		; P2.0 pin "H" with open drain

ADC_Init:
    mov  ADC0CF,    #00111000b	; 3MHz AD SAR CLOCK, right justified
    setb AD0EN		; enable ADC

Voltage_Reference_Init:
    mov  REF0CN,    #00Eh
                  
; Peripheral specific initialization functions,
Port_IO_Init:
    ; P0.0  -  Skipped,     Open-Drain, Analog
    ; P0.1  -  Skipped,     Open-Drain, D/A output Analog
    ; P0.2  -  Skipped,     Open-Drain, Analog
    ; P0.3  -  Skipped,     Open-Drain, Analog
    ; P0.4  -  Skipped,     Open-Drain, Analog
    ; P0.5  -  Skipped,     Open-Drain, Analog
    ; P0.6  -  Skipped,     Push-Pull,  Digital
    ; P0.7  -  CEX0 (PCA),  Open-Drain, Digital

    ; P1.0  -  Skipped,     Push-Pull,  Digital
    ; P1.1  -  Skipped,     Open-Drain, Analog
    ; P1.2  -  Skipped,     Open-Drain, Analog
    ; P1.3  -  Skipped,     Open-Drain, Analog
    ; P1.4  -  Unassigned,  Push-Pull,  Digital
    ; P1.5  -  Unassigned,  Push-Pull,  Digital
    ; P1.6  -  Unassigned,  Push-Pull,  Digital
    ; P1.7  -  Unassigned,  Push-Pull,  Digital

	call SPIN_FREE	; <<<<<<<<<<<<<

    mov  P0MDIN,    #0C0h
    mov  P1MDIN,    #0F1h
    mov  P0MDOUT,   #040h
    mov  P1MDOUT,   #0F1h
    mov  P2MDOUT,   #001h
    mov  P0SKIP,    #07Fh
    mov  P1SKIP,    #00Fh
    mov  XBR1,      #041h

DAC_Init:
    mov  IDA0CN,    #0F2h

;   mov XBR1, #11000001b	; no weak pull-ups, cross bar enabled on out put pins, CEX0 routed to Port pin.
			; All out puts are "L", just after RESET.

	mov CPT0CN, #10000000b	; bit7: Comp. Enable, bit6: Comp out, bit5,4: no interrupt, bit3-0: no hys.

; Timer0,1 16bit counter with pre-scaled system clock is 1/48=1960nS.<<<<<<<<<<<<<<<<
T01_SET:	orl CKCON, #00000010b

PCA0_INT:	; Programable Counter Array is used in Edge-triggered Caputure Mode for power control input from Rx.
	orl PCA0CN, #01000000b	; Run Programmable Counter 0 
	;mov	PCA0MD, #00000000b	; PCA0 works in 1/12 system clock
	orl PCA0CPM0, #00100001b	; PCA0 is captured by positive edge first on P0.7, Also generates the interrupt.
      	orl  	EIE1, #00010000b	; enable PCA0 interrupts 
	clr   	CCF0                  	; clear PCA0 module0 interrupt flag
;	orl EIP1,#00010000b	; PCA0 innterrupt set to high priority level.

	call WAKE_UP_CP	; wait for comparator wake up.
	setb EA		; enable global interrupts
;--------------------
PWR_ON_SOUND:
SOUND_1	equ 1 	; Sound frame time
;************************************
SND_ON_DUTY	equ 200	; Sound pitch
;************************************
	mov TEMP3, #SOUND_1	;
	mov TEMP5, #SND_ON_DUTY	;
	call SOUNDER

	call NO_SOUND_TIM

	mov TEMP3, #SOUND_1	;
	mov TEMP5, #SND_ON_DUTY	;
	call SOUNDER

	call NO_SOUND_TIM

	mov TEMP3, #SOUND_1	;
	mov TEMP5, #SND_ON_DUTY	;
	call SOUNDER
	call NO_SOUND_TIM
;****************************************************************
;PACK_TEST:	call LIPO_2S_3S_CHK	;  Test the P0.6 voltage for 2 or 3 Li-Po pack detection
;****************************************************************
GET_INPUT_PULSE:
	call SPIN_FREE	; make the outputs are open drain as 3-Sate open to reduce the SW noise

FIND_BUZZER_INT:
	mov TEMP0, #0		; clear T2 repeat counter as TEMP0
	;mov FIND_TIMER, #1		; TEST ONLY: T2 x TEMP0 x FIND_TIMER timer(8 sec.)
	mov FIND_TIMER, #FIND_TIMER_DELAY	; T2 x TEMP0 x FIND_TIMER timer(x 8sec.) ; BUG found on Mar. 17,
	call START_T2			; clear T2 counter 32mS
;-----------------
THROTTLE_POS_CHK:
	call WDT_OFF
	mov a, CONTROL_DATA		; Get Rx input pulse 
BRAKE_SET_CHK:	clr c
		subb a, #060h		; Almost Full throttle?
		jnc BRAKE_FLAG_ON		; Yes. Then Brake fuction ON
		jmp THROTTLE_OFF_CHK

BRAKE_FLAG_ON:
	orl TEMP7, #00000010b	; Turn on Brake_ON flag

THROTTLE_OFF_CHK:
	call WDT_OFF
	mov a, CONTROL_DATA	; Get Rx input pulse 
		jz THROTTLE_OFF_CHK	; wait for first Rx pulse 

		clr c
		subb a, #THROTTLE_OFF	; wait for throttle OFF position
		jc READY_SOUND	; Is the input pulse less than 940uS?
		jmp THROTTLE_OFF_CHK	; No.

READY_SOUND: 
SOUND_2	equ 1 
;*******************************************
SND_ON_DUTY2 equ 100
	;mov b, #pack
	mov b, #3
;*******************************************	
GO_SOUND_LOOP:
	mov TEMP3, #SOUND_2	
	mov TEMP5, #SND_ON_DUTY2
	call SOUNDER
	call NO_SOUND_TIM
	djnz b, GO_SOUND_LOOP 

;;****** Vdd check TEST ONLY
	mov R1, #0
LOOP0:	djnz R1, LOOP0
	
V_TEST:	call ALL_H
	;call CHECK_TOP_CELL_VOLTAGE
	call SPIN_FREE
	call WDT_OFF
;	jmp LOOP0

BRAKE_TIMER_INT:
		mov BRAKING_TIMER, #0			; BRAKING_TIMER is used for brake timer counter with TF2H over flow

;-----------------
THROTTLE_ON_CHK:	jnb TF2H, RX_IN_CHK
		call SPIN_FREE
		call WDT_OFF

INC_T2_REPEAT_CNT:	clr TF2H

BRAKE_TIMER_COUNT_TRIG:	inc R7
		cjne R7, #10, FIND_BUZZER_TIMER_CNT_TRIG	; 32msec x10= 320mS sec.

;BRAKE_TIMER_COUNT_TRIG:	mov a, BRAKING_TIMER
;		inc a
;		mov BRAKING_TIMER, a
;		clr c
;		subb a, #10
;		jnz FIND_BUZZER_TIMER_CNT_TRIG	; 32msec x10= 320mS sec.

		mov BRAKING_TIMER, #0
			dec BRAKING_TIMER

BRAKE_ON_CHK:	mov a, TEMP7
			anl a, #00000010b
			jz FIND_BUZZER_TIMER_CNT_TRIG	; Brake function Off mode

		call BRAKING

FIND_BUZZER_TIMER_CNT_TRIG:
		inc TEMP0
		cjne TEMP0, #0, RX_IN_CHK	; 32mS x 255 = 8.16 sec. 
		dec FIND_TIMER			; 8.16 x 22 = 3 min. 
		mov a, FIND_TIMER
		jnz RX_IN_CHK 	; 
		mov FIND_TIMER, #FIND_TIMER_DELAY	; 
			jmp FIND_BUZZER_ON
	
RX_IN_CHK:	setb EA
		call SPIN_FREE
		call WDT_OFF
		
		mov CONTROL_DATA_MAX, #255
		mov MISFIRE_COUNTER, #0
		mov a, CONTROL_DATA	; Get Rx input pulse 
			clr c
			subb a, #THROTTLE_OFF	; wait for throttle OFF position
			jc THROTTLE_ON_CHK	; the input pulse more than 940uS
			jmp THROTTLE_OFF_CHK1	; Yes.

;---------------
SOUNDER:	;clr EA
SOUNDER1:	mov TEMP0, #010h
SOUNDING:		call S_WDT_OFF	
			call STATE1_ON
			call SOUND_DRIVE_CNT
			call SPIN_FREE
			call STATE2_ON
			call SOUND_DRIVE_CNT
			call SPIN_FREE
			call STATE3_ON
			call SOUND_DRIVE_CNT
			call SPIN_FREE
			call STATE4_ON
			call SOUND_DRIVE_CNT
			call SPIN_FREE
			call STATE5_ON
			call SOUND_DRIVE_CNT
			call SPIN_FREE
			call STATE6_ON
			call SOUND_DRIVE_CNT
			call SPIN_FREE
		djnz TEMP0, SOUNDING
		mov TEMP0, #010h
	
	djnz TEMP3, SOUNDING
	;setb EA
	ret
;----------------
THROTTLE_ON_CHK_A: 
	jmp THROTTLE_ON_CHK	;long jmp help
;----------------
SOUND_DRIVE_CNT:	
SND_ON_TIM:
	mov   ON_DUTY, TEMP5
SND_ON_LOOP:
	nop
	djnz  ON_DUTY, SND_ON_LOOP

ALL_3ST:
	call SPIN_FREE
;*********************************************
	call S_WDT_OFF
SND_OFF_TIM:
	mov ON_DUTY, TEMP5 ; 1:10 as 10% power for sound
SND_OFF_LOOP:
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	djnz  ON_DUTY, SND_OFF_LOOP

	ret
;------------------
NO_SOUND_TIM:
		call SPIN_FREE

	mov TEMP3, #0
	mov TEMP5, #20
NO_SOUND1:
		call WAIT_10uS
		
		call S_WDT_OFF
		
		djnz TEMP3, NO_SOUND1
	djnz TEMP5, NO_SOUND1
	ret
;------------------
FIND_BUZZER_ON:
	mov R1, #230

BUZZER:	mov TEMP3, #1	; (Sound frame time)
	mov TEMP5, R1	; Sound pitch
	call SOUNDER1
	call SPIN_FREE

THROTTLE_ON_CHK_B:
	mov a, CONTROL_DATA	; Get Rx input pulse 
		clr c
		subb a, #THROTTLE_OFF	; check the throttle OFF position
		jnc READY_SOUND_1

	dec R1 
		cjne R1, #100, BUZZER	; highest frequency.

COOLING_MOTOR:	mov R1, #128	; 4sec off sound to avoid motor heat-up
CMTR1:		mov R2, #0
CMTR2:		mov R3, #0
CMTR3:		mov a, CONTROL_DATA	; Get Rx input pulse 
		clr c
		subb a, #THROTTLE_OFF	; check the throttle OFF position
		jnc READY_SOUND_1
		call S_WDT_OFF
		djnz R3, CMTR3
		djnz R2, CMTR2	 	
		djnz R1, CMTR1				

		jmp FIND_BUZZER_ON

;------------------
READY_SOUND_1: 	call SPIN_FREE
		;jmp READY_SOUND	; long branch help
		jmp RX_IN_CHK	; Spin immediately. '06/02/06 
;--------------
STEPPING_START:
;--------------
THROTTLE_OFF_CHK1:
	mov DRV_SEQUENCE_CHECK_ENABLE, #0	; enable drive sequence check for starting sequence by this flag
	;mov DRV_TIM, #SAMPLING_TIMING	; set default sampling timing	

	mov STEPPING_ON_DUTY_CHOPPING_COUNTER, #STEPPING_ON_DUTY_CHOPPING_COUNT
	mov STEPPING_ON_DUTY, #START_CURRENT_ON_DUTY
		
	mov a, CONTROL_DATA	; Get Rx input pulse 
	mov TEMP3, a
		clr c
		subb a, #THROTTLE_OFF	; check the throttle OFF position
		jc THROTTLE_ON_CHK_A	; the input pulse less than 940uS
;---------------
MOTOR_START:	call BRAKING	; For high site drive power
		call WAIT_10uS

SPIN_START:		call GET_START_SHAFT_POSITION1

SPIN_START1:	mov PREV_DRV_STATE, START_DRIVE_STATE 

		call GET_NEXT_DRIVE_STATE	; for reverse spin check as well
		mov NEXT_DRV_STATE, START_DRIVE_STATE
		
		call GET_NEXT_DRIVE_STATE	; 2 Step move for sure
		mov NEXT_NEXT_DRV_STATE, START_DRIVE_STATE

		mov START_DRIVE_STATE, PREV_DRV_STATE
		call SHAFT_POSITION_TRIMMING	; Spin motor with one stepping
		
		;call BRAKING
		mov b, #250		; spin responce timer
WAIT_SPIN_RESPONCE1:	call WAIT_1uS
		call WDT_OFF
		djnz b, WAIT_SPIN_RESPONCE1 

THROTTLE_OFF_CHK0:	mov a, CONTROL_DATA	; Get Rx input pulse 
			clr c
			subb a, #THROTTLE_OFF	; wait for throttle OFF position
			jc STEPPING_START	; the input pulse more than 940uS

SPIN_START_CHECK1:	;call test_pin_on
		call GET_START_SHAFT_POSITION1	; START_DRIVE_STATE = Now state+1

		;call GET_COMP_DATA
		;mov a, COMP_D
		mov a, START_DRIVE_STATE				 
		clr c
REVERSE_SPIN_CHECK:	subb a, NEXT_DRV_STATE	; Reverse spin check is here
		jz START_STEPPING_DRIVE2; one step spin with turned on_duty
		;jz NEXT_SPIN_START	
		jmp COULD_NOT_START_SPIN
;-----
START_STEPPING_DRIVE2:	;Make sure two steps spin stepping drive
		mov START_DRIVE_STATE, NEXT_DRV_STATE
		call SHAFT_POSITION_TRIMMING	; Spin motor with next one stepping

		;call BRAKING
		mov b, #250		; spin responce timer
WAIT_SPIN_RESPONCE2:	call WAIT_1uS
		call WDT_OFF
		djnz b, WAIT_SPIN_RESPONCE2

SPIN_START_CHECK2:	call GET_START_SHAFT_POSITION1	; START_DRIVE_STATE = Now state+1
		mov a, START_DRIVE_STATE
		clr c
SPIN_CHECK2:	subb a, NEXT_NEXT_DRV_STATE	; Reverse spin check is here
		jz START_STEPPING_DRIVE; two step spin with turned on_duty
		jmp COULD_NOT_START_SPIN
;-----------------------
COULD_NOT_START_SPIN:
INC_STEPPING_START_ON_DUTY:
		call BRAKING	; For high site drive power
		call WAIT_10uS
		
		mov a, STEPPING_ON_DUTY
		add a, #INC_START_CURRENT_ON_DUTY_STEP	; inc as on_duty+5
		clr c
		subb a, #START_STEPPING_DUTY_FRAME/4	; <<<<<<<<< 25% limit power duty for start
		jc ADD_MORE_START_PWR
		jmp CAN_NOT_SPIN_START
				 
ADD_MORE_START_PWR:	mov a, STEPPING_ON_DUTY
		add a, #INC_START_CURRENT_ON_DUTY_STEP	; inc as on_duty
		mov STEPPING_ON_DUTY, a
TRY_WITH_MORE_START_POWER:
		jmp SPIN_START
;-----------
CAN_NOT_SPIN_START:
	jmp START_STEPPING_DRIVE	; force Start 
;	mov STEPPING_ON_DUTY, #START_CURRENT_ON_DUTY
;GET_REVERSE_DRIVE_STATE:
;	mov a, START_DRIVE_STATE
;	clr c
;	subb a, #2	; 
;	jz SET_DRIVE_STATE_6	; START_DRIVE_STATE=2
;	jc SET_DRIVE_STATE_5	; START_DRIVE_STATE=1

;SET_REVERSE_STATE:
;	mov START_DRIVE_STATE, a
;	jmp SPIN_START1

;SET_DRIVE_STATE_6:
;	mov a, #6
;	mov START_DRIVE_STATE, a
;	jmp SPIN_START1

;SET_DRIVE_STATE_5:
;	mov a, #5
;	mov START_DRIVE_STATE, a
;	jmp SPIN_START1
;-------
START_STEPPING_DRIVE:	call STEP_START1
		mov STEP_START_FLAG, #1	; Set STEP_START_FLAG
		jmp BEMF_START_SPIN
;---------------
GET_START_SHAFT_POSITION:
STEP_START_BARKING:	call BRAKING
		mov b, #10		; Make sure the Shaft stop to get next step start state
SHAFT_BRAKE_B:		mov a, #0
SHAFT_BRAKE_A:		dec a
			call WDT_OFF
			call WAIT_10uS
			jnz SHAFT_BRAKE_A
		djnz b, SHAFT_BRAKE_B 
GET_START_SHAFT_POSITION1:
DG4:		call STATE1_ON	; U=1, V=0, W=3ST
        		call MEASURE_INDUCTIVE_KICK_LENGTH

DG1:		call STATE4_ON	; U=0, V=1, W=3ST
		call MEASURE_INDUCTIVE_KICK_LENGTH	

DG5:		call STATE2_ON	; U=1, V=3ST, W=0
		call MEASURE_INDUCTIVE_KICK_LENGTH

DG2:		call STATE5_ON	; U=0, V=3ST, W=1
		call MEASURE_INDUCTIVE_KICK_LENGTH
			
DG6:		call STATE3_ON	; U=3ST, V=1, W=0
		call MEASURE_INDUCTIVE_KICK_LENGTH

DG3:		call STATE6_ON	; U=3ST, V=0, W=1
		call MEASURE_INDUCTIVE_KICK_LENGTH

	jmp STEP_START

;--------------------------
MEASURE_INDUCTIVE_KICK_LENGTH:
	clr EA
	mov IDA0L, #00000000b		; 10mV sense as 0.1V on @10mili ohm MOSFET and divided to 1:10
	mov IDA0H, #00000001b
		
SAMPLE_ON_DUTY:	mov b, #20		; Less than 10uS, Could not measure the inductive kick. 
SAMPLE_ON_Loop:	call WAIT_1uS
		djnz b, SAMPLE_ON_Loop

		call SPIN_FREE
		mov a, DRIVE_STATE
;-------
SAMPLE_KICK1:	cjne a, #1, SAMPLE_KICK2	; U=1, V=0, W=3ST
		jmp S_KICK12
SAMPLE_KICK2:	cjne a, #2, SAMPLE_KICK3	; U=1, V=3ST, W=0
		; U=1 -> U=GND-0.7V(3ST) -> U=V=W
S_KICK12:		mov CPT0MX, #00000000b	; CP in-: P0.1(D/A), CP in+: P0.0(U) with UP slope
		
BEFORE_KICK:

KICK_COUNTER_HL:	call START_T2
		call WAIT_1uS	; Comparator responce
COUNT_KICK_LH:		mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
		jz COUNT_KICK_LH	; checking the negative edge of inductive kick.
		clr TR2	; Stop Timer2
	setb EA
		jmp GET_START_POSITION
;-------
SAMPLE_KICK3:	cjne a, #3, SAMPLE_KICK4	; U=3ST, V=1, W=0
		jmp S_KICK34
SAMPLE_KICK4:	cjne a, #4, SAMPLE_KICK5	; U=0, V=1, W=3ST
		; V=1 -> V=GND-0.7V(3ST) -> U=V=W
S_KICK34:		mov CPT0MX, #00000001b	; CP in-: P0.1(D/A), CP in+: P0.2(V) with UP slope
		jmp BEFORE_KICK
;-------
SAMPLE_KICK5:	; W=1 at stae 5 and 6
		; W=1 -> W=GND-0.7V(3ST) -> U=V=W
S_KICK56:		mov CPT0MX, #00000010b	; CP in-: P0.1(D/A), CP in+: P0.4(W) with UP slope
		jmp BEFORE_KICK

;-------
GET_START_POSITION:	call WDT_OFF
		mov a, TMR2L	; compare the previous inductive kick timing length
		clr c
		subb a, TEMP_L

		mov a, TMR2H
		subb a, TEMP_H
		
		jnc SAMPLE_OFF_DUTY	; Get minimum timing for starting

		mov TEMP_H, TMR2H	; Store the minimum inductive kick length "H" as start position
		mov TEMP_L, TMR2L	; Store the minimum inductive kick length "L"
		mov START_DRIVE_STATE, DRIVE_STATE	; Get start drive pattern

SAMPLE_OFF_DUTY:	call BRAKING	; 
		mov TEMP5, #20
SAMPLE_OFF_DUTY_LOOP:		call WDT_OFF
			mov b, #10	; <<<<<<
OFF_Loop3:			call WAIT_1uS
			djnz b, OFF_Loop3
		djnz TEMP5, SAMPLE_OFF_DUTY_LOOP
		ret
;--------
STEP_START:	mov SPIN_COUNTER, #20		; Off Timer
WAIT_TO_START:
	call SAMPLE_OFF_DUTY
	djnz SPIN_COUNTER, WAIT_TO_START

	mov TEMP_L, 0ffh	; Re-Store the minimum inductive kick length "L" for next start 
	mov TEMP_H, 0ffh	; Re-Store the minimum inductive kick length "H" for next start

GET_NEXT_DRIVE_STATE:
	mov a, START_DRIVE_STATE
	mov DRV_SEQUENCE_CHECK_ENABLE, #0	; also, diable sequence check by this flag

	inc a	; Next start drive Phase
	clr c
	subb a, #7	; If START6, Then step start1
	jz STEP_START1_

	add a, #7
	mov START_DRIVE_STATE, a
	ret

STEP_START1_:
	mov a, #1
	mov START_DRIVE_STATE, a
	ret
;---------------------------------------------------
SHAFT_POSITION_TRIMMING:
	mov TEMP5, #STEPPING_ON_DUTY_CHOPPING_COUNT

SHAFT_TRIM_START1:		
; Stepping start with duty control as current limitter.
CHOP_DRIVE1:
	call STEP_DRIVE_STATE
	call START_ON_DUTY_CONTROL
	djnz TEMP5, CHOP_DRIVE1

TRIM_DRIVE_OFF:	mov SPIN_COUNTER, #0
;		call test_pin_on
		ret
;---------------------------------------------------
STEP_DRIVE_STATE:	mov a, DRIVE_STATE
STEP_DRIVE_STATE1:	cjne a, #1, STEP_DRIVE_STATE2
		call PRE_CHARGE_STATE1_DRIVER
		call STATE1_ON
		ret

STEP_DRIVE_STATE2:	cjne a, #2, STEP_DRIVE_STATE3
		call PRE_CHARGE_STATE2_DRIVER
		call STATE2_ON
		ret

STEP_DRIVE_STATE3:	cjne a, #3, STEP_DRIVE_STATE4
		call PRE_CHARGE_STATE3_DRIVER
		call STATE3_ON
		ret

STEP_DRIVE_STATE4:	cjne a, #4, STEP_DRIVE_STATE5
		call PRE_CHARGE_STATE4_DRIVER
		call STATE4_ON
		ret

STEP_DRIVE_STATE5:	cjne a, #5, STEP_DRIVE_STATE6
		call PRE_CHARGE_STATE5_DRIVER
		call STATE5_ON
		ret

STEP_DRIVE_STATE6:	call PRE_CHARGE_STATE6_DRIVER
		call STATE6_ON
		ret
;--------------------
STEP_START1:	mov a, START_DRIVE_STATE
		cjne a, #1, STEP_START2		
			call STATE1_ON
			call STEP_DRIVE_CNT
			call STATE2_ON
			call STEP_DRIVE_CNT
			call STATE3_ON
			call STEP_DRIVE_CNT
			call STATE4_ON
			call STEP_DRIVE_CNT
			call STATE5_ON
			call STEP_DRIVE_CNT
			call STATE6_ON
			call STEP_DRIVE_CNT
			jmp STEP_TO_BEMF

STEP_START2:	cjne a, #2, STEP_START3
			call STATE2_ON
			call STEP_DRIVE_CNT			
			call STATE3_ON
			call STEP_DRIVE_CNT
			call STATE4_ON
			call STEP_DRIVE_CNT
			call STATE5_ON
			call STEP_DRIVE_CNT
			call STATE6_ON
			call STEP_DRIVE_CNT
			call STATE6_ON
			call STEP_DRIVE_CNT
			jmp STEP_TO_BEMF

STEP_START3:	cjne a, #3, STEP_START4
			call STATE3_ON
			call STEP_DRIVE_CNT
			call STATE4_ON
			call STEP_DRIVE_CNT
			call STATE5_ON
			call STEP_DRIVE_CNT
			call STATE6_ON
			call STEP_DRIVE_CNT
			call STATE1_ON
			call STEP_DRIVE_CNT			
			call STATE2_ON
			call STEP_DRIVE_CNT
			jmp STEP_TO_BEMF

STEP_START4:	cjne a, #4, STEP_START5
			call STATE4_ON
			call STEP_DRIVE_CNT
			call STATE5_ON
			call STEP_DRIVE_CNT
			call STATE6_ON
			call STEP_DRIVE_CNT
			call STATE1_ON
			call STEP_DRIVE_CNT
			call STATE2_ON
			call STEP_DRIVE_CNT			
			call STATE3_ON
			call STEP_DRIVE_CNT
			jmp STEP_TO_BEMF

STEP_START5:	cjne a, #5, STEP_START6			
			call STATE5_ON
			call STEP_DRIVE_CNT
			call STATE6_ON
			call STEP_DRIVE_CNT
			call STATE1_ON
			call STEP_DRIVE_CNT
			call STATE2_ON
			call STEP_DRIVE_CNT
			call STATE3_ON
			call STEP_DRIVE_CNT			
			call STATE4_ON
			call STEP_DRIVE_CNT
			jmp STEP_TO_BEMF

STEP_START6:		call STATE6_ON
			call STEP_DRIVE_CNT
			call STATE1_ON
			call STEP_DRIVE_CNT
			call STATE2_ON
			call STEP_DRIVE_CNT
			call STATE3_ON
			call STEP_DRIVE_CNT
			call STATE4_ON
			call STEP_DRIVE_CNT			
			call STATE5_ON
			call STEP_DRIVE_CNT

STEP_TO_BEMF:	;call SPIN_FREE	; No reverse spin check here.
		;mov SPIN_COUNTER, #0
		ret
;------------
STEP_DRIVE_CNT:	
	mov TEMP5, STEPPING_ON_DUTY_CHOPPING_COUNTER
CHOP_ON_DUTY:	
	call STEP_DRIVE_STATE
	call START_ON_DUTY_CONTROL
	djnz TEMP5, CHOP_ON_DUTY
	dec STEPPING_ON_DUTY_CHOPPING_COUNTER	; speed up for next state drive
	dec STEPPING_ON_DUTY_CHOPPING_COUNTER	; as accelaration
	dec STEPPING_ON_DUTY_CHOPPING_COUNTER
	dec STEPPING_ON_DUTY_CHOPPING_COUNTER
	ret
;------------
START_ON_DUTY_CONTROL:
ON_DUTY_TIM1:	mov b, STEPPING_ON_DUTY
CLoop1:		;call WAIT_500nS
		call WAIT_1uS
		call WAIT_1uS
		djnz b, CLoop1
	
		;*********************************************
		mov P0_IO, P0	; IO data storage for P0
		mov IO_MODE_M, P1MDOUT	; GPIO mode setting data storage
		mov IO_SETTING, P1	; IO data storage
		;*********************************************

		call SPIN_FREE	; takes a few uS for MOSFET OFF
		call WDT_OFF
OFF_DUTY_TIM1:	mov a, #START_STEPPING_DUTY_FRAME
		clr c
		subb a, STEPPING_ON_DUTY
		mov b,a

CLoop2:		call WAIT_500nS
		;call WAIT_1uS
		djnz b, CLoop2
		;*********************************************
		mov P0, P0_IO	; IO data storage for P0
		mov P1, IO_SETTING	; IO data storage
		mov P1MDOUT, IO_MODE_M 	; GPIO mode setting data storage
		;*********************************************
		ret
;---------------------
BEMF_START_SPIN:	mov STEP_START_FLAG, #1	; Set STEP_START_FLAG
		mov SPIN_COUNTER, #0	; First some spins with BEMF
		call WDT_OFF

BEMF_START_SPIN1:	mov a, #BEMF_start_run_on_duty_time_after_stepping_drive	; BEMF start run on_duty timing
		call BEMF_START_DUTY_TIMER
		
		call SPIN_FREE
		mov a, #BEMF_start_run_off_duty_time_after_stepping_drive	 ; BEMF start run off_duty timing
		call BEMF_START_DUTY_TIMER

		mov a, DRV_SEQUENCE_CHECK_ENABLE
		jnz SPIN_WITH_DRV_SEQUENCE_CHECK_BEMF_SPIN

NO_DRV_SEQUENCE_CHECK_BEMF_SPIN:
		inc SPIN_COUNTER
		mov a, SPIN_COUNTER
		clr c
		subb a, #5	; one times for first BEMF spin
		jc BEMF_START_SPIN_A	

SPIN_WITH_DRV_SEQUENCE_CHECK_BEMF_SPIN:
		mov DRV_SEQUENCE_CHECK_ENABLE, #1 ; Reverse spin check enabled
		inc SPIN_COUNTER
		mov a, SPIN_COUNTER
		jnz BEMF_START_SPIN_A		; 256 times spin		
		jmp BEMF_START0	; start spins with BEMF with PWR control

BEMF_START_SPIN_A:
	clr EA
		call GET_COMP_DATA
	setb EA	
		mov a, COMP_D
		call WDT_OFF

BEMF_START_SPIN_STATE1:	cjne a, #1, BEMF_START_SPIN_STATE2
		call STATE1
		jmp BEMF_START_SPIN1

BEMF_START_SPIN_STATE2:	cjne a, #2, BEMF_START_SPIN_STATE3
		call STATE2
		jmp BEMF_START_SPIN1

BEMF_START_SPIN_STATE3:	cjne a, #3, BEMF_START_SPIN_STATE4
		call STATE3
		jmp BEMF_START_SPIN1

BEMF_START_SPIN_STATE4:	cjne a, #4, BEMF_START_SPIN_STATE5
		call STATE4
		jmp BEMF_START_SPIN1

BEMF_START_SPIN_STATE5:	cjne a, #5, BEMF_START_SPIN_STATE6
		call STATE5
		jmp BEMF_START_SPIN1

BEMF_START_SPIN_STATE6:	call STATE6
		jmp BEMF_START_SPIN1

;---
BEMF_START_DUTY_TIMER:	call WAIT_1uS		
		dec a
		jnz BEMF_START_DUTY_TIMER
		ret
;---------------------
BEMF_START0:	mov SPIN_MODE, #1	; set to BEMF mode
		mov ON_DUTY, #SLOW_START ; BEMF slow start run duty

BEMF_START1:	call START_T0	; comparator oscillation loop reset
BEMF_RESTART:	mov CONTROL_DATA_MAX, #255	; Current Limiter initialize data
		mov MISFIRE_COUNTER, #0
		
		mov TEMP0, #0	; times of T2 over flow counter for increasing throttle timer
SET_BEMF_TIMER:
		call T2_SET		; Start T2 timer for slow BEMF control time
		call SPIN_FREE	; 
;---------------------
GET_BEMF:
BEMF_SPEED_CNT:	mov TEMP3, CONTROL_DATA	; Get Rx input pulse

THROTTLE_OFF_CHK2:	mov a, temp3	; Get Rx input pulse to check stop
			clr c
			subb a, #THROTTLE_OFF	; check the throttle OFF position. Is the input pulse less than 940uS?
			jc THROTTLE_OFF_CHK_TIMER2	; 		

		mov RX_ERROR_COUNT, #0	; clr T2(32mS interval) repeat counter
RUN_NORMALLY0:		jmp RUN_NORMALLY
;---
THROTTLE_OFF_CHK_TIMER2:
		mov FIND_TIMER, #FIND_TIMER_DELAY
			jnb TF2H, RUN_NORMALLY0	; confirm the Rx noise pulse

T160mS_INTERVAL:	clr TF2H
		call WDT_OFF
		inc RX_ERROR_COUNT
		mov a, RX_ERROR_COUNT
		clr c
		subb a, #5	; over 160mS throttle off signal come?
		jc RUN_NORMALLY

; Checking free spining before stepping fire
CHECK_FREE_SPINING:	call SPIN_FREE	
		call START_T0
		mov b, #0

CHECK_FREE_SPIN:	call WDT_OFF
		call GET_COMP_DATA
		mov a, COMP_D

SPIN_STATE1:	cjne a, #1, SPIN_STATE2
		orl b, #00000001b
		jmp CHECK_ALL_SPIN_PATTERN

SPIN_STATE2:	cjne a, #2, SPIN_STATE3
		orl b, #00000010b
		jmp CHECK_ALL_SPIN_PATTERN

SPIN_STATE3:	cjne a, #3, SPIN_STATE4
		orl b, #00000100b
		jmp CHECK_ALL_SPIN_PATTERN

SPIN_STATE4:	cjne a, #4, SPIN_STATE5
		orl b, #00001000b
		jmp CHECK_ALL_SPIN_PATTERN

SPIN_STATE5:	cjne a, #5, SPIN_STATE6
		orl b, #00010000b
		jmp CHECK_ALL_SPIN_PATTERN

SPIN_STATE6:	cjne a, #6, RE_START_STEPPING
		orl b, #00100000b
		
CHECK_ALL_SPIN_PATTERN:	mov a, b
		clr c
		subb a, #00111111b	; spin with all pattern?
		jz CHECK_FREE_SPINING	; Yes.		
;-------------------
THROTLE_RE_ON_CHECK:	mov CONTROL_DATA_MAX, #255
		mov MISFIRE_COUNTER, #0
		mov a, CONTROL_DATA		; Get Rx input pulse
		mov TEMP3, a
			clr c
			subb a, #THROTTLE_OFF	; check the throttle OFF position. Is the input pulse less than 940uS?
			jnc RUN_NORMALLY_

		mov a, TH0
		clr c
		subb a, #Slowest_Spin_Speed_Check_Time; Stiil spin a little bit to avoid reverse start spin
		jc CHECK_FREE_SPIN

THROTTLE_OFF2:	; Is the input pulse still less than 940uS?
RE_START_STEPPING:	mov RX_ERROR_COUNT, #0 ;
		;mov CONTROL_DATA_MAX, #THROTTLE_OFF+2
		jmp THROTTLE_ON_CHK	; re-start with stepping spin

RUN_NORMALLY_:	mov RX_ERROR_COUNT, #0 ;
		jmp BEMF_PWR_SETTING
RUN_NORMALLY:
;;****** Added for reverse spin check and retry routine is here
	mov DRV_SEQUENCE_CHECK_ENABLE, #1	; Spin drive with correct sequence

SPIN_SPEED_CHECK:
	mov a, TH1
	clr c		; 
	subb a, #10	; spin enough for BEMF(490nS x 4 x 255 x 10= 5mS/spin)?
	jc REVERSE_SPIN_CHECK1

	mov a, ON_DUTY
	clr c
	subb a, #THROTTLE_OFF+3	; Spin direction check work area is between THROTTLE_OFF and THROTTLE_OFF+*
	jc BEMF_PWR_SETTING
	
	mov a, ON_DUTY
	clr c
	subb a, #THROTTLE_OFF+10; Spin direction check work area is between THROTTLE_OFF and THROTTLE_OFF+*
	jnc BEMF_PWR_SETTING

REVERSE_SPIN_CHECK1:
	mov a, MISFIRE_COUNTER	; MISFIRE_COUNTER is in drive state routine
	clr c
ADJ_REVERSE_SENSE:
	subb a, #MISFIRE_COUNT	; Noise filter to ensure the reverse spin and avoid immidiate Stop motor
	jnc REVERSE_SPIN
	jmp BEMF_PWR_SETTING

REVERSE_SPIN:
	;jmp REVERSE_SPIN	; TEST ONLY
	;call BRAKING	; dangerous TEST
	mov MISFIRE_COUNTER, #0	; Just for timer use only
;	jmp CHECK_FREE_SPIN	; Unstable BUG check ONLY

	call BRK_TIM0	; <<<<<<<<

	mov MISFIRE_COUNTER, #0
	call BRAKING	;
	call BRK_TIM0

	mov MISFIRE_COUNTER, #0
	jmp STEPPING_START
;-----------
BRK_TIM0:	call start_T0
BRK_TIM1:	call WDT_OFF
	jnb TF0, BRK_TIM1	; 32mS loop 

	inc MISFIRE_COUNTER
	mov a, MISFIRE_COUNTER
	clr c
	subb a, #5	; loop as about 160mS	; <<<<<<<
	jc BRK_TIM0	
	ret
;;**********
BEMF_PWR_SETTING:	; 12uS max. from OFF_TIMER end
		setb TR2		
			cjne RX_ERROR_COUNT, #0, GET_CMP_DATA ; No duty control jmp. May be Throttle off Rx noise "<<

		mov a, TEMP3	; Get Rx input pulse
			clr c
			subb a, ON_DUTY	; DUTY >= Rx input pulse?
			jc DEC_PWR		; Yes.
			jz GET_CMP_DATA
				
INC_DUTY:		mov a, STEP_START_FLAG	; Need small on_duty to avoid MOSFET melt just after stepping starting?
		jz  INC_DUTY_ON	; No.

		mov a, SPIN_COUNTER	; SPIN_COUNTER is increased in state1 drive. 
		clr c
		subb a, #10	; Do not increase the starting power before 10 spins.
		jc SET_START_POWER	; set small on_duty to avoid MOSFET melt for starting.
		mov STEP_START_FLAG, #0	; Clear STEP_START_FLAG

INC_DUTY_ON:		mov a, TMR2H 
			clr c
			subb a, #BEMF_Duty_Inc_Timing 		
			jnc INC_ONDUTY
			jmp GET_CMP_DATA
			
INC_ONDUTY:		call START_T2
		inc ON_DUTY
		jmp GET_CMP_DATA

SET_START_POWER:	mov ON_DUTY, #SLOW_START ; BEMF slow start run duty
		mov DRV_SEQUENCE_CHECK_ENABLE, #1 ; Reverse spin check
		jmp GET_CMP_DATA

DEC_PWR:		;mov a, ON_DUTY	; Test monitor only
		dec ON_DUTY
;-----------------
GET_CMP_DATA:
	clr EA	
		call GET_COMP_DATA	; Compare the motor generative voltage while drive 3-ST timing.
	setb EA
	;13uS from OFF_TIMER end with stick midium.

			mov a, ROUND_CHK_FLAG		; 
			anl a, #11111100b		; 
			cjne a, #11111100b, COMP_STATE1	; spin with all state sequence?
			call WDT_OFF		; Yes. Spinning check by WDT for MOSFET protection	

		mov a, ROUND_CHK_FLAG		; Round check for WDT power off protection control
			anl a, #00000011b		; Clear the spinning status
			mov ROUND_CHK_FLAG, a		; 

			cjne a, #3, COMP_STATE1_3SPINS	; a few times run round in BEMF mode?

BACK_TO_BEMF:	mov ROUND_CHK_FLAG, #0		; reset the round check flag
			jmp COMP_STATE1		; 
	
COMP_STATE1_3SPINS:	mov a, ROUND_CHK_FLAG
			inc a
		mov ROUND_CHK_FLAG, a

COMP_STATE1:	cjne COMP_D, #1, COMP_STATE2
			orl ROUND_CHK_FLAG, #00000100b		; Spin check flag to make sure the starting
		call STATE1
		jmp BEMF_DRIVE_ON

COMP_STATE2:	cjne COMP_D, #2, COMP_STATE3
			orl ROUND_CHK_FLAG, #00001000b
		call STATE2
		jmp BEMF_DRIVE_ON

COMP_STATE3:	cjne COMP_D, #3, COMP_STATE4
		orl ROUND_CHK_FLAG, #00010000b
			
NORMAL_RUN:		
COMP_STATE_03:	call STATE3
			jmp BEMF_DRIVE_ON

COMP_STATE4:	cjne COMP_D, #4, COMP_STATE5
			orl ROUND_CHK_FLAG, #00100000b
		call STATE4	
		jmp BEMF_DRIVE_ON
;
COMP_STATE5:	cjne COMP_D, #5, COMP_STATE6
			orl ROUND_CHK_FLAG, #01000000b
BEMF_DRIVE_5:		call STATE5	
			jmp BEMF_DRIVE_ON
;
COMP_STATE6:	;cjne COMP_D, #6, COMP_STATE_STOP_OVER	; COMP_D =7
DRIVE_6:		orl ROUND_CHK_FLAG, #10000000b
		call STATE6	
		jmp BEMF_DRIVE_ON

;COMP_STATE_STOP_OVER:	; COMP_D =7

		;jmp GET_BEMF		; Time over when motor stops and the comparator can work properly 

BEMF_DRIVE_ON:	;26uS from OFF_TIMER end with stick midium.
						; <<<<<<<<
			mov a, SAMPLING_COUNTER_L	; inc number of sampling counts
			clr c
			add a, #1
			mov SAMPLING_COUNTER_L, a

			mov a, SAMPLING_COUNTER_H
			addc a, #0
			mov SAMPLING_COUNTER_H, a

		call ON_TIMER
		;call CHECK_TOP_CELL_VOLTAGE	; voltage down and over current check routine 

BEMF_MAX_THROTTLE_CHK1:		mov a, ON_DUTY 	; May be ON_DUTY is 100 at low voltage driving	
			clr c
			subb a, BEMF_FULL_ON		; Is it full throttle drive condition?
			jnc ZERO_CROSS_IN	; Yes, start zero cross drive 
				
		jmp GET_BEMF
;---------------------------------------------------
GET_BEMF1:		call SPIN_FREE
		mov ROUND_CHK_FLAG, #11111100b ; spin all sequence
		mov ON_DUTY, BEMF_FULL_ON
		dec ON_DUTY	; -1
		mov DRV_SEQUENCE_CHECK_ENABLE, #0	; Not sure the first spin state at here
		mov RX_NOISE_FILTER_COUNTER, #0
		mov ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER, #127	; Checking BEMF max speed
		mov SPIN_MODE, #1	; set to BEMF mode

		mov SPIN_COUNTER, #5	; check slow down with this value spins
TWICE_SAME_SAMPLING_DATA_CHECK:	;<<<<<<<<
		call START_BEMF_SAMPLING_COUNTER

		call GET_COMP_DATA
		mov TEMP_MEMORY, COMP_D
		;
		call WDT_OFF

CHECK_BEMF_SAMPLING_TIME_END:
		mov a, TMR3CN	; BEMF_SAMPLING_TIME_END?	
		anl a, #10000000b	; cheking timer3 over flow flag high
		jnz TWICE_SAME_SAMPLING_DATA_CHECK ; time over

		call GET_COMP_DATA
		mov a, COMP_D
		clr c
		subb a, TEMP_MEMORY
		jz CHECK_BEMF_SAMPLING_TIME_END	; Still same BEMF data
		
; MAke sure slow enough for sampling as BEMF spin?
SLOW_DOWN_CHECK_WITH_SOME_SPINS:
		dec SPIN_COUNTER
		mov a, SPIN_COUNTER
		jnz TWICE_SAME_SAMPLING_DATA_CHECK		

WAIT_SET_BEMF_RUN:	mov SPIN_COUNTER, #20
;setb p0.6	; MONITOR
		call START_T3
		jmp GET_BEMF
;---------------------------------------------------
ZERO_CROSS_IN:	
;setb p0.6	; MONITOR
;
BEMF_ACCELERATION_MONITOR:
		mov a, ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER
		clr c
		subb a, #127	; Cheking the acceleration average + or -
		jc THROTTLE_OFF_CHK3
;setb p0.6	; MONITOR
		jmp GET_BEMF

THROTTLE_OFF_CHK3:	
;clr p0.6	; MONITOR
		mov TEMP3, CONTROL_DATA	; Get Rx input pulse to check stop
;------------------------------
ZERO_CROSS_RUN:	mov SPIN_MODE, #0	; set to Zero Cross spin mode
		;Accelerate timer makes inc/dec power calc only for power control. 
		mov SPIN_COUNTER, #1	; stay 50% power run 254 spins at zero cross mode starting by this count
				; advance timing should be low to get near 50% power at mode changing.
		call SPIN_FREE	; turn off sysc. rectifier for GET_COMP_DATA 
STATE_CHECK_3:	
ZERO_C4:
;** IF SOMETHING HAPPENED HERE, THEN SYSTEM STOPS BY WATCH DOG TIMER!!! **
	clr EA	
		call GET_COMP_DATA
	setb EA
		cjne COMP_D, #4, ZERO_C4; Get start point of state4 

		mov PREV_DRIVE_STATE, #3
		call STATE4		; U=3ST, V=1, W=0 -> U=0, V=1, W=3ST		
		mov CPT0MX, #01010010b	; CP in-: P1.3(HALFW), CP in+: P0.4(W) with UP slope
		;call INDUCKTIVE_KICK_DET_LH ; NO INDUCTIVE KICK.Then Start T2
		call START_T2
		call ZERO_CROSS_CHK_UP ; 
		call SPIN_FREE7	; 50% power for the start
		call WDT_OFF
WAIT_T2_OVER34:	jnb TF2H, WAIT_T2_OVER34
		mov ON_DUTY, #0
		jmp ZERO_CROSS5		
;----------------
ZERO_CROSS3:	call STATE3		; U=1, V=3ST, W=0 -> U=3ST, V=1, W=0
				; P0.2 and P0.3(V), P0.4 and P0.5(W) are connected togather by solder.
		mov CPT0MX, #01000000b	; CP in-: P1.1(HALFU), P0.0(U) with DOWN slope
		call INDUCKTIVE_KICK_DET_LH ;
		call ZERO_CROSS_CHK_DOWN; 
		;call SPIN_FREE7	; TEST ONLY 50% power run TEST
		call ZERO_CROSS_OFF_DUTY

ZERO_CROSS4:	call STATE4		; U=3ST, V=1, W=0 -> U=0, V=1, W=3ST
		mov CPT0MX, #01010010b	; CP in-: P1.3(HALFW), CP in+: P0.4(W) with UP slope
		call INDUCKTIVE_KICK_DET_HL ; Inductive kick L->H and H->L detector
		call ZERO_CROSS_CHK_UP ; Inputs are reversed.
		;call SPIN_FREE7	; TEST ONLY
		call ZERO_CROSS_OFF_DUTY

ZERO_CROSS5:	call STATE5		; U=0, V=1, W=3ST -> U=0, V=3ST, W=1
		mov CPT0MX, #00010101b	; , CP in-: P0.3(V), CP in+: P1.2(HALFV) with DOWN slope
		call INDUCKTIVE_KICK_DET_HL ; Inductive kick L->H and H->L detector 
		call ZERO_CROSS_CHK_UP	;
		
		;call SPIN_FREE7	; TEST ONLY 50% power run TEST
		call ZERO_CROSS_OFF_DUTY
;-----------------
ZERO_CROSS6:	call STATE6		; U=0, V=3ST, W=1 -> U=3ST, V=0, W=1
		mov CPT0MX, #01000000b	; CP in-: P1.1(HALFU), CP in+: P0.0(U) with UP slope
		call INDUCKTIVE_KICK_DET_HL ; Inductive kick L->L and H->L detector
		call ZERO_CROSS_CHK_UP	; 		
		;call SPIN_FREE7	; TEST ONLY 50% power run TEST
		call ZERO_CROSS_OFF_DUTY

ZERO_CROSS1:	call STATE1		; U=3ST, V=0, W=1 -> U=1, V=0, W=3ST
		mov CPT0MX, #01010010b	; CP in-: P1.3(HALFW), CP in+: P0.4(W) with DOWN slope
		call INDUCKTIVE_KICK_DET_LH ; Inputs are reversed. Inductive kick H->L to L->H detector
		call ZERO_CROSS_CHK_DOWN;
		;call SPIN_FREE7	; TEST ONLY 50% power run TEST
		call ZERO_CROSS_OFF_DUTY

ZERO_CROSS2:	call STATE2		; U=1, V=0, W=3ST -> U=1, V=3ST, W=0
		mov CPT0MX, #00010101b	; CP in-: P0.3(V), CP in+: P1.2(HALFV)with UP slope
		call INDUCKTIVE_KICK_DET_LH ; Inputs are reversed. Inductive kick H->L and L->H detector
		call ZERO_CROSS_CHK_DOWN
		;call SPIN_FREE7	; TEST ONLY 50% power run TEST
		call ZERO_CROSS_OFF_DUTY
;-----------------------
ZERO_CROSS_RUN_PWR_CONTROL:
		;call SPIN_FREE	; TEST ONLY

THROTTLE_OFF_CHK5:	mov TEMP3, CONTROL_DATA	; Get Rx input pulse to check stop
		mov a, TEMP3	; Get Rx input pulse 

			add a, #STICK_BACK_HYS	; hysteresis margin for Rx input pulse jitter
			clr c
			subb a, BEMF_FULL_ON ; MAX throttle position as 50% run in BEMF mode
			jc RX_NOISE_CANCELLER

		mov RX_NOISE_FILTER_COUNTER, #0

PWR_CNTRL5:		;mov a, NEED_MORE_T_ADVANCE_FLAG
		;jz ZC_PWR_SETTING

		mov a, SPIN_COUNTER
		jnz ZC_PWR_SETTING

		;mov NEED_MORE_T_ADVANCE_FLAG, #0
		jmp ZERO_CROSS3_		; 255 spin with 37% timing advance to avoid sync. out

ZC_PWR_SETTING:		mov a, TEMP3
			clr c
			subb a, BEMF_FULL_ON ; MAX throttle position as 50% run in BEMF mode

			; jc ZERO_CROSS3_	; BUGBUG MUST cosider the stick hysteresis value.
			jnc INC_DEC5
			jmp ZERO_CROSS3_

INC_DEC5:			subb a, ON_DUTY		
			;jmp ZERO_CROSS3_	; 50% power TEST ONLY

			jz ZERO_CROSS3_
			jnc INC_PWR5
			jmp DEC_PWR5	 
;-----------------		
INC_PWR5:		mov a, TMR3CN	; T3 is used for slow increase power-up timer as same as BEMF T2 mode
		anl a, #01000000b	; Ckeck T3 low byte overflow flag 
			jnz INC_DUTY_5	; T3 overflow?
			jmp SET_ON_DUTY_TIMER	; Not yet

INC_DUTY_5:		anl TMR3CN, #00111111b	; Clear T3 overflow flags
		inc TEMP0		; Also, TEMP0 is used for 256 loops slow power-up timer counter with T3
		cjne TEMP0, #ZC_SAMPLING_DUTY_INC_TIMING, ZERO_CROSS3_
				
		mov TEMP0, #0
		inc ON_DUTY		
		jmp ZERO_CROSS3	; Still in zero cross run <<<<<<<<

ZERO_CROSS3_:	jmp ZERO_CROSS3	; Still in zero cross run

DEC_PWR5:		dec ON_DUTY
			jmp ZERO_CROSS3	; Still in zero cross run
;------------------
SET_ON_DUTY_TIMER:	clr TR2		; Stop T2
		mov a, T2_ON_DUTY_LOW	; Get previous timing data
		cpl a
		mov TMR2L, a	; set T2 low byte data
		
		mov a, T2_ON_DUTY_HIGH	; Get previous timing data	
		cpl a
		mov TMR2H, a	; set T2 high byte data

START_DUTY_ON:	clr TF2H		; clear T2 high byte over flow bit
		setb TR2		; start T2 count-up for power on duty time
		
		jmp ZERO_CROSS3	; Still in zero cross run
		;ret
;------------------
GET_BEMF_02:		jmp GET_BEMF1	; long branch help
;---------------
RX_NOISE_CANCELLER:	clr c
		mov a, RX_NOISE_FILTER_COUNTER	
			add a, #1
			clr c
			mov RX_NOISE_FILTER_COUNTER, a
			subb a, #Rx_Noise_Count	; almost 0.1 sec margin to protect Rx stick down noise
			jnc GET_BEMF_02	; get back to BEMF mode
		jmp ZERO_CROSS3	; Still in zero cross run
;------------
INDUCKTIVE_KICK_DET_LH:	call START_T2
		call WAIT_1uS	; comparator res. to avoid inductive kick front edge
		;
IND_KICK_NEG_LEVEL_CHK:
		mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
			jz IND_KICK_POS_EDGE_CHK	; The inductive kick is already done in off duty timing.

		clr TR2
		mov TMR2M_H, TMR2H
		mov TMR2M_L, TMR2L
		setb TR2
		ret

IND_KICK_POS_EDGE_CHK:	clr c
		mov a, TMR2H
			subb a, #IND_KICK_TIME_CHECK ; Sync out value in T2. Over 350uS to get zero coross point as over load.
			jnc SMALL_ZC_MARGIN_POS
		
ZC_POS_CHK:		call WAIT_1uS	; Comparator responce
		mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
			jz IND_KICK_POS_EDGE_CHK	; checking the positive edge of inductive kick.

		clr TR2
		mov TMR2M_H, TMR2H
		mov TMR2M_L, TMR2L
		setb TR2
		ret

SMALL_ZC_MARGIN_POS:	call SPIN_FREE
		mov a, ON_DUTY	; May be accelating. Power down to avoid un-sync. 
		clr c
		subb a, #1
		jc ZERO_ON_DUTY_POS
		mov ON_DUTY, a
ZERO_ON_DUTY_POS:	jmp ZC_POS_CHK
;---------------
COULD_NOT_CATCH_CROSS_POINT_: 	call SPIN_FREE
			jmp MOTOR_UNUSUAL_STOP
;---------------
ZERO_CROSS_CHK_DOWN:	call WAIT_10uS	; Avoid chatter

		clr EA
ZERO_CROSS_CHK_D:	mov a, TMR2H
			clr c
			subb a, #REAR_EDGE_TO_ZC_POINT ; Sync out value in T2. Over 500uS to get zero coross point as over load.
		jnc COULD_NOT_CATCH_CROSS_POINT_ ; System shut down to protect FETs.

		call WAIT_1uS	; comparator responce.
		mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
			jnz ZERO_CROSS_CHK_D	; checking the DOWN SLOPE until over zero cross point 

STOP_T2:		clr TR2		; stop the T2 count-up
		setb EA

OVER_ZERO_CHECK:	
;			call SPIN_FREE	; TEST ONLY!!!!!!
;check_point:		nop
		
OVER_ZERO_CHECK_H:	mov a, TMR2H	; Checking the T2 is zero
			jnz TIMING_ADVANCE	; No, It is normal
		
OVER_ZERO_CHECK_L:	mov a, TMR2L
			anl a, #11100000b	; Cut T2 low 4bits as Is the zero cross time less than 31 as 15uS?
			jnz TIMING_ADVANCE	; No, It is normal
		
SET_TF2H:		call SPIN_FREE
		setb TF2H	; some thing wrong. speed up next T2 end check routine(T2 over flow check).		
		ret
;***************
TIMING_ADVANCE:	mov R1, TMR2L
		mov R2, TMR2H
		call TIMING_ADVANCE_CALC 
		jmp T_ADV_ADJ_END
;
TIMING_ADVANCE_CALC:	mov a, R2
			clr c
			rrc a		; 1/2 as 25% timing advance
		mov R2, a

		mov a, R1
			rrc a
		mov R1, a
		;
		;mov a, NEED_MORE_T_ADVANCE_FLAG
		;jnz TWO_BIT_RIGHT_SHIFT
		ret
		
TWO_BIT_RIGHT_SHIFT:	mov b, R1
		mov TEMP_MEMORY, R2
				
		mov a, R2
			clr c
			rrc a		; 1/4 as 1/8 12.5% Timing advance(1/2 phase measuring)
		mov R2, a

		mov a, R1
			rrc a
		mov R1, a
		;		
		;ret

ADD_FOR_37_PERCENT_AD_TIMING:		; 37.5% advance for only start of zero cross control
		clr c
		add a, b
		mov R1, a

		mov a, R2		
		addc a,TEMP_MEMORY
		mov R2, a
		ret


		;
		mov a, R2
			clr c
			rrc a		; 1/8 as 1/16 6.3% Timing advance(1/2 phase measuring)
		mov R2, a

		mov a, R1
			rrc a
		mov R1, a
		
		ret

		;
		mov a, R2
		clr c
		rrc a		; 1/16 as 1/32 3% Timing advance(1/2 phase measuring)

		mov R2, a
		mov a, R1
		rrc a
		mov R1, a

END_T_ADJ:		ret
;--
T_ADV_ADJ_END:	
STORE_ADV_T_L:	mov a, TMR2L
			clr c
			subb a, R1	; Advanced time "L" is in R1
		mov R1, a	; R1 is temp

STORE_ADV_T_H:	mov a, TMR2H
			subb a, R2
;			jc NO_ADVANCE_TIME_SET
		mov R2, a		; Advanced time "H" is in R2
;---------
STICK_POWER_ADDING:	
		mov a, ON_DUTY
		jz ZERO_ON_DUTY	; no adding power. keep 50% duty
		mov b, ZC_STEPS	; zero cross power up STEPS

		mul ab
				
		mov TMR2L, a	; TMR2L is temporary data storage for T2 Time set as stick control "L"
		mov TMR2H, b	; TMR2H is temporary data storage for T2 Time set as stick control "H"		
;-----------
COMP_FULL_ON_LIMIT_CHK:	mov a, R1			; Advanced timing "L"
			clr c
			subb a, TMR2L	; 	
		mov ZC_OFF_DUTY_L, a		; set ZC power off duty control time
COMP_FULL_ON_H:	mov a, R2			; Advanced timing "H"
			subb a, TMR2H
		mov ZC_OFF_DUTY_H, a		; set ZC power off duty control time
			jc FULL_POWER_TIME_SET	; Over 100% setting?

ZC_ADD_POWER:	mov R1, TMR2L
		mov R2, TMR2H
		jmp ZC_T2_SET

FULL_POWER_TIME_SET:	mov ZC_OFF_DUTY_H, #0 	; full on power setting
		mov R2, #0
		mov ZC_OFF_DUTY_L, #1	; ZC_OFF_DUTY_L must be never zero
		mov R1, #1
ZC_T2_SET:		mov a, R1		; get low byte of T2 zero cross timing data (Also Advanced timing "L", if time set over)
			cpl a	; T2 is the count-up timer
		mov TMR2L, a	; low byte re-load data			
		mov a, R2		; get high byte of T2 zero cross timing data(Also Advanced timing "H", if time set over)
			cpl a
		mov TMR2H, a	; high byte re-load data
	
		clr TF2H
		setb TR2
		ret
;------------------
ZERO_ON_DUTY:	mov ZC_OFF_DUTY_L, R1	; keep 50% power duty
		mov ZC_OFF_DUTY_H, R2 
		jmp ZC_T2_SET
;*****************
ZERO_CROSS_OFF_DUTY:	call CURRENT_LIMITER
		;call LESS_THAN_5V_CHECK
ZC_OFF_TIME_SET:	;;;;*****************
		;call SPIN_FREE
		call SPIN_FREE8
		mov a, DRIVE_STATE
		clr c
		subb a, #6		; State 6 driving?
		jnz ZC_OFF_TIME_SET1
		;call CHECK_TOP_CELL_VOLTAGE	; voltage down and over current check routine 
		;;;;;********		
ZC_OFF_TIME_SET1:	call WDT_OFF
		clr TR2

		mov TMR2L, ZC_OFF_DUTY_L
		mov TMR2H, ZC_OFF_DUTY_H

		mov a, TMR2L	; get low byte of T2 zero cross timing data
		cpl a		; T2 is the count-up timer
		mov TMR2L, a	; low byte re-load data
				
		mov a, TMR2H	; get high byte of T2 zero cross timing data
		cpl a
		mov TMR2H, a	; high byte re-load data
	
		clr TF2H
		setb TR2
		
ZC_POWER_OFF_TIME:	jnb TF2H, ZC_POWER_OFF_TIME
		ret

MOTOR_UNUSUAL_STOP:	call SPIN_FREE
		JMP GET_BEMF1	; Keep run
;--------------
INDUCKTIVE_KICK_DET_HL:	call START_T2
		call WAIT_1uS	; Avoid inductive kick front edge

IND_KICK_POS_LEVEL_CHK:
		mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
			jnz IND_KICK_NEG_EDGE_CHK	; The inductive kick is already done in off duty timing.

		clr TR2
		mov TMR2M_H, TMR2H
		mov TMR2M_L, TMR2L
		setb TR2
		ret

IND_KICK_NEG_EDGE_CHK:	clr c
		mov a, TMR2H
			subb a, #IND_KICK_TIME_CHECK ; Sync out value in T2. Over 350uS to get zero coross point as over load.
			jnc SMALL_ZC_MARGIN_NEG

ZC_NEG_CHK:		call WAIT_1uS	; Comparator responce
		mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
			jnz IND_KICK_NEG_EDGE_CHK	; checking the negative edge of inductive kick.

		clr TR2
		mov TMR2M_H, TMR2H
		mov TMR2M_L, TMR2L
		setb TR2
		ret

SMALL_ZC_MARGIN_NEG:	call SPIN_FREE
		mov a, ON_DUTY	; May be accelating. Power down to avoid un-sync. 
		clr c
		subb a, #1
		jc ZERO_ON_DUTY_NEG
		mov ON_DUTY, a
ZERO_ON_DUTY_NEG:	jmp ZC_NEG_CHK

;---------------------
ZERO_CROSS_CHK_UP:	call WAIT_10uS	; Avoid chatter
		
		clr EA
ZERO_CROSS_CHK_U:	mov a, TMR2H
			clr c
			subb a, #REAR_EDGE_TO_ZC_POINT ; Sync out value in T2. Over 250uS to get zero coross point as over load.
			jnc COULD_NOT_CATCH_CROSS_POINT
		mov CPT0CN, #10000000b
		call WAIT_1uS	; comparator responce.
		mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
			jz ZERO_CROSS_CHK_U	; checking the UP SLOPE until over zero cross point
		setb EA
STOP_T2_2:		clr TR2		; stop the T2 count-up

CALC_TIMING_ADVANCE:	jmp OVER_ZERO_CHECK
;------------
COULD_NOT_CATCH_CROSS_POINT: 	call SPIN_FREE
			jmp MOTOR_UNUSUAL_STOP
;------------

CHECK_T2_END:
		;call WAIT_20uS
T2_END_CHECK:	jnb TF2H, T2_END_CHECK ; wait untill T2 high byte over flow flag for T2 time over.
		clr TF2H
		ret
;------------
;as well as Auto timing adjust routine
OVER_SHOOT_CHK:	call WAIT_1uS	; avoid comparator responce delay
OVER_SHOOT_CHK_:	jnb TF2H, OVER_SHOOT_CHK_
		ret

OVER_SHOOT_CHK_UD:	mov a, CPT0CN	; Get Comparator result 
		anl a, #01000000b	; Get CP0OUT bit. Checking the SLOPE until over GND or VCC.
		jnz KEEP_RUN	; No. No Problem.
		
		;call SPIN_FREE	; TEST ONLY
		
		ret		; SLOPE over GND or VCC.
			
KEEP_RUN:		jnb TF2H, OVER_SHOOT_CHK_UD ; Checking T2 high byte over flow flag for T2 time over.

		;call SPIN_FREE	; TEST ONLY


		ret		; Yes, T2 time over.
;------------------------------------------
TIMER_20uS:		call WAIT_20uS
		ret
;------------------------------------------
;-------------------	
SAME_RPM:		jmp SET_ON_DUTY_TIMER
;-------------------
;------------------------------------------
GET_COMP_DATA:
;	mov a, ON_DUTY
;	clr c
;	subb a, #SLOW_START-1	; <<<<<<<< Spin enough as BEMF enough?
;	jc SPIN_SPEED_NOT_ENOUGH
;	call COMP_MEASURE	; to get high responce
;	ret

SPIN_SPEED_NOT_ENOUGH:
	mov COMP_REMEASURE_COUNTER, #50
	call COMP_MEASURE		
	;call WDT_OFF	; Motor off and No burning if re-measure loop time is so long.

GET_FIRST_BEMF_DATA:
	mov PREV_COMP_D, COMP_D

RE_MEASURE:	dec COMP_REMEASURE_COUNTER	; BEMF Re-measure loop timing limitter
	call COMP_MEASURE	; Re-check
	mov a, COMP_D	; 
	cjne a, PREV_COMP_D, NOISE_DATA ; Same comp. result?

GOT_BEMF_DATA:
	ret		; Yes

NOISE_DATA:	mov a, COMP_REMEASURE_COUNTER	; Try 50 times to get stable result.
	jz NOISE

	mov PREV_COMP_D, COMP_D	; No
	jmp RE_MEASURE
	
NOISE:	;mov a, COMP_D		; system failer
	jmp NOISE
	jmp GOT_BEMF_DATA	
;-----------------------------------------------------------------------------
; Interrupt routine to get Rx input pulse
;-----------------------------------------------------------------------------
PCA0_PLS:	clr   CCF0		; clear PCA0 module0 interrupt flag

            push  PSW               ; Program Status Word, preserve registers
            mov STORAGE_B, b
	push  acc

Capture_Edge_check:
	mov a, PCA0CPM0
	anl a, #00100000b		;Caputure rise edge select data on PCA0, is bit 5
	jz fall_edge 
; 
rise_edge:	mov PCA0_CAP_L, PCA0CPL0	; Save 16 bit counter low byte value at positive front edge time 
            mov PCA0_CAP_H, PCA0CPH0	; Save 16 bit counter high byte value at positive front edge time 
	mov PCA0CPM0, #00010001b	; PCA0 is captured by negative edge on P0.7 first and generates the interrupt.	

	pop   acc
            mov b, STORAGE_B
	pop   PSW
            reti
;
fall_edge:	mov PCA0CPM0, #00100001b	; PCA0 is captured by positive edge on P0.7 first and generates the interrupt.
	mov a, PCA0CPL0
	clr c
	subb a, PCA0_CAP_L
	mov PULSE_WIDTH_L, a

	mov a, PCA0CPH0
	subb a, PCA0_CAP_H
	mov PULSE_WIDTH_H, a

	clr c
	mov a, PULSE_WIDTH_L
	subb a, #LOW(CTRL_MIN)		; offset control minimum pulse width(about 1mS)
	mov PULSE_WIDTH_L, a

	mov a, PULSE_WIDTH_H
	subb a, #HIGH(CTRL_MIN)		; offset control minimum pulse width
	jc ALMOST_ZERO		; Minus after offset adj.
	mov PULSE_WIDTH_H, a

CNTRL_PULSE_CHK:
	mov a, PULSE_WIDTH_H		; 00 - 09 range for normal input.
	jz MIN_WIDTH_NEGO
	jmp MAX_WIDTH_CHK
		
MIN_WIDTH_NEGO:
	mov a, PULSE_WIDTH_L		
	clr c
	subb a, #080h		; no need low nibble. Less than "8" is Zero
	jc ALMOST_ZERO

MAX_WIDTH_CHK:
	mov a, PULSE_WIDTH_H		; 00 - 09 range for normal input.
	clr c
	subb a, #10
	jnc BAD_PULSE		; range over pulse(over 10 or less than 0)	

	mov a, PULSE_WIDTH_L
	mov CONTROL_DATA_L, a
	mov a, PULSE_WIDTH_H
	mov CONTROL_DATA_H, a 
;--------
DUTY_CALC:	mov a, CONTROL_DATA_L	; The pulse width count resolution is 0000 - 0900
	anl a, #11110000b
	rr a		; no need low nibble
	rr a
	rr a
	rr a
	mov b, a

	mov a, CONTROL_DATA_H
	anl a, #00001111b
	clr c
	subb a, #9		; over 8 as 9?
	jz max_08h_hi
	jmp cut_hi_nibble_0

max_08h_hi: mov a, CONTROL_DATA_H
	dec a		
	jmp cut_hi_nibble

cut_hi_nibble_0:
	mov a, CONTROL_DATA_H
cut_hi_nibble:
	rl a		; not use in upper nibble
	rl a
	rl a
	rl a
	add a,b
	mov CONTROL_DATA, a
	
	clr c			; 
	subb a, CONTROL_DATA_MAX	; When over heat or Li-Po voltage down, Then MAX duty decreaced.
	jnc MAX_DOWN
	jmp RETURN_I

MAX_DOWN:
	mov CONTROL_DATA, CONTROL_DATA_MAX 	; Power limit at emergency condition		; 	

RETURN_I:	mov a, CONTROL_DATA		; input pulse noise filter
	clr c
	subb a, TEMP4		; Is it same as previous Rx in pulse width?
	jz RET_I			; Yes. 
	mov TEMP4, CONTROL_DATA		; No.
		
RET_I:	mov INPUT_COUNTER_L, #0	; clear input signal lose check counter
	mov INPUT_COUNTER_H, #0	; This counter is cleared in State2 on

	pop   acc
            mov b, STORAGE_B
	pop   PSW
            reti

ALMOST_ZERO: mov CONTROL_DATA, #1	; Debug on Mar. 8th '05
	jmp RETURN_I

BAD_PULSE:	jmp RETURN_I
;------------------------------------
; Timer0 16bit counter with pre-scaled clock is 24.5MHz/12=490nS.
START_T0:	mov TL0, #0	; 
	mov TH0, #0	; 
	clr TF0
	setb TR0
	ret

; Timer1 16bit counter with pre-scaled clock is 24.5MHz/12=490nS.
START_T1:	clr TR1
	mov TL1, #0	; 
	mov TH1, #0	; 
	clr TF1
	setb TR1
	ret
;
; Timer2 16bit counter with auto re-load, pre-scaled clock is 1/12=490nS.
T2_SET:	anl CKCON, #11001111b	; T2 clock source select internal OSC
	orl TMR2CN, #00000100b	; Enable Timer2
;;;;**********
	anl CKCON, #11001111b	; Timer2 with 1/12 sys. clock.
	jmp START_T2

H_T2_SET:	mov CKCON, #0	; T2 clock source select internal OSC
	orl CKCON, #00110000b	; Timer2 with sys. clock.
	orl TMR2CN, #00000100b	; Enable Timer2
;;;;**********
START_T2:	clr TR2
	mov TMR2RLL, #0h	; low byte re-load data
	mov TMR2RLH, #0h	; High byte re-load data
	mov TMR2L, #0h	; 
	mov TMR2H, #0h	; 
	clr TF2H
	clr TF2L
	setb TR2
	ret
;
START_T3:	anl TMR3CN, #00111011b	; clr TF3H, clr TF3L and clr TR3 "<<<<<<<<
	mov TMR3RLL, #0h	; low byte re-load data;
	mov TMR3RLH, #0h	; High byte re-load data
	mov TMR3L, #0h	; 
	mov TMR3H, #0h	; 
	orl TMR3CN, #00000100b	; setb TR3
	ret
;------------
; BEMF run power duty control timer routine
ON_TIMER:	
;setb P1.7
	anl TMR3CN, #11111011b	; clr TR3 to get BEMF off-duty time
	mov BEMF_OFF_DUTY_TIME_L, TMR3L	; get off duty time	
	mov BEMF_OFF_DUTY_TIME_H, TMR3H	

	call START_T3	; start to measure the on-duty time by T3
			
L1:	mov a, ON_DUTY	; on-duty timer
	add a, ON_DUTY	; x2
	;add a, ON_DUTY	; x3
	mov DUTY, a
L2:	call WAIT_500nS
	djnz  DUTY, L2	; High speed sampling 
;clr P1.7

OFF_DUTY:	anl TMR3CN, #11111011b	; stop timer3 to get BEMF on-duty time
	call SPIN_FREE	; set off-duty
	mov R1, TMR3L	; get on-duty time
	mov R2, TMR3H
	;
	mov a, BEMF_OFF_DUTY_TIME_L
	clr c
	add a, R1
	mov BEMF_DUTY_TIME_L, a

	mov a, BEMF_OFF_DUTY_TIME_H
	addc a, R2
	mov BEMF_DUTY_TIME_H, a	; Storage sampling time using for returning from ZC mode
	;
	call TIMING_ADVANCE_CALC
	 
	mov a, TMR3L	; adjust the off-duty time with timing advanced time.
	clr c
	subb a, R1	
	mov R1,a

	mov a, TMR3H
	subb a, R2
	mov R2, a		; 

	mov a, BEMF_OFF_DUTY_TIME_L	; off-duty time - on-duty time < 0 as zero-cross in power?
	clr c
	subb a, R1	
	
	mov a, BEMF_OFF_DUTY_TIME_H
	subb a, R2	
	jc OVER_BEMF_POWER
	jmp OFF_RUN

OVER_BEMF_POWER:
;setb p0.6
	inc BEMF_PWR_OVER_CHECK_COUNTER
	mov a, BEMF_PWR_OVER_CHECK_COUNTER
	clr c
	subb a, #100	; power over continusly?
	jnc TRY_TO_GO_ZC
	jmp TRIG_OFF_DUTY_MEASURING_TIMER

TRY_TO_GO_ZC:
	mov a, ON_DUTY
	clr c
	subb a, #SLOW_START	; <<<<<<<<
	jc OFF_RUN		; too small for BEMF_FULL_ON. just for safe.

	dec ON_DUTY		; <<<<<It is already over
	mov BEMF_FULL_ON, ON_DUTY ; set on_duty data include +- 1 digit as Rx jitter
	jmp TRIG_OFF_DUTY_MEASURING_TIMER

OFF_RUN:	mov BEMF_FULL_ON, #100	; stick power limit in BEMF mode
	mov BEMF_PWR_OVER_CHECK_COUNTER, #0

TRIG_OFF_DUTY_MEASURING_TIMER:
	call START_T3	; start to measure the BEMF off-duty time

OFF_TIMER:	
;clr p0.6	; MONITOR
GET_INDUCTIVE_KICK:		mov IDA0L, #00000000b		; D/A setting 10mV sense as 0.1V divided to 1:10 by R
			mov IDA0H, #00000001b
			
		mov a, DRIVE_STATE
I_KICK1:	cjne a, #1, I_KICK2	; U=1, V=0, W=3ST
		jmp I_KICK12
I_KICK2:	cjne a, #2, I_KICK3	; U=1, V=3ST, W=0
		; U=1 -> U=GND-0.7V(3ST) -> U=V=W
I_KICK12:		mov CPT0MX, #00000000b	; CP in-: P0.1(D/A), CP in+: P0.0(U) with UP slope
		
GET_I_KICK:		mov TEMP_MEMORY, #100	; as 50uS timer while speed down as no inductive kick 
I_KICK_LH:		call WAIT_500nS	; Comparator responce
		dec TEMP_MEMORY
		mov a, TEMP_MEMORY
		jz OFF_DUTY_TIME_OVER

		mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
		jz I_KICK_LH	; checking the negative edge of inductive kick.
OFF_DUTY_TIME_OVER:
	setb EA
		ret
;-------
I_KICK3:	cjne a, #3, I_KICK4	; U=3ST, V=1, W=0
		jmp I_KICK34
I_KICK4:	cjne a, #4, I_KICK5	; U=0, V=1, W=3ST
		; V=1 -> V=GND-0.7V(3ST) -> U=V=W
I_KICK34:		mov CPT0MX, #00000001b	; CP in-: P0.1(D/A), CP in+: P0.2(V) with UP slope
		jmp GET_I_KICK
;
I_KICK5:	; W=1 at stae 5 and 6
		; W=1 -> W=GND-0.7V(3ST) -> U=V=W
I_KICK56:		mov CPT0MX, #00000010b	; CP in-: P0.1(D/A), CP in+: P0.4(W) with UP slope
		jmp GET_I_KICK
;----------
START_BEMF_SAMPLING_COUNTER:
	anl TMR3CN, #00111011b	; clr TF3H, clr TF3L and clr TR3

	mov a, BEMF_DUTY_TIME_L
	cpl a
	mov TMR3RLL, a	; low byte re-load data;

	mov a, BEMF_DUTY_TIME_H
	cpl a
	mov TMR3RLH, a	; High byte re-load data

	mov TMR3L, #0ffh	; 
	mov TMR3H, #0ffh	; 

CHECK_BEMF_SAMPLING_TIME:
	;
	orl TMR3CN, #00000100b	; setb TR3 to start

WAIT_T3_RELOAD:
	mov a, TMR3CN
	anl a, #10000000b	; Checks TF3H
	jz WAIT_T3_RELOAD

	anl TMR3CN, #00111111b	; clr TF3H, clr TF3L

START_BEMF_SAMPLING_TIMING_COUNTER:
	ret
;----------
COMPARATOR_RES_DELAY:
WAIT_500nS:	mov TEMP1, #3	; consider the call instruction delay
	jmp W_10L

WAIT_1uS:	mov TEMP1, #9
	jmp W_10L

WAIT_20uS:	mov TEMP1, #190	; 95 at 24.5MHz
	jmp W_10L

WAKE_UP_CP:	orl CPT0CN, #10000000b	; bit7: Comp. Enable, bit6: Comp out, bit5,4: no interrupt, bit3-0: no hys.
	mov CPT0MD, #0	; 100nS responce	
WAIT_10uS:	mov TEMP1, #95	; 95 at 24.5MHz

W_10L:	nop		; 1 cycle clock = 40nS
	nop		; 1 cycle clock = 40nS
	djnz  TEMP1, W_10L	; 2/3 cycle clock = 26nS
	ret
;-----------------------------------------------------------------------------
COMP_MEASURE:	; P0.2 and P0.3(V), P0.4 and P0.5(W) are connected togather by solder.

COMP_A1:	mov CPT0MX, #00100000b	; CP in-: P0.5(W), CP in+: P0.0(U) 	
	call COMPARATOR_RES_DELAY
	mov a, CPT0CN	; Get Comparator result 
	anl a, #01000000b	; Get CPOUT bit
	mov COMP1_A, a	; store the result		

COMP_B1:	mov CPT0MX, #00010000b	; CP in-: P0.3(V), CP in+: P0.0(U)  
	call COMPARATOR_RES_DELAY
	mov a, CPT0CN	; Get Comparator result 
	anl a, #01000000b	; Get CPOUT bit
	mov COMP1_B, a

COMP_C1:	mov CPT0MX, #00100001b	; CP in-: P0.5(W), CP in+: P0.2(V)
	call COMPARATOR_RES_DELAY
	mov a, CPT0CN	; Get Comparator result 
	anl a, #01000000b	; Get CP0OUT bit	
	mov COMP1_C, a
;
COMP_LOGIC:	
UW:	mov a, COMP1_A	; CP in+:(U), CP in-:(W): CP in-: P0.5(W), CP in+: P0.0(U)
	jnz UV		; U>W
	jmp WV		; W>U

UV:	mov a, COMP1_B	; CP in+:(U), CP in-:(V): CP in-: P0.3(V), CP in+: P0.0(U)	
	jnz UH_VW		; U>V
	jmp VH_UM_WL	; V>U, #3

WV:	mov a, COMP1_C	; CP in+:(V), CP in-:(W): CP in-: P0.5(W), CP in+: P0.2(V)
	jnz VH_WM_UL	; V>W, #4		
	jmp WH_UV		; W>V 

UH_VW:	mov a, COMP1_C	; CP in+:(V), CP in-:(W): CP in-: P0.5(W), CP in+: P0.2(V)
	jnz UH_VM_WL	; V>W, #2 or All"1" offset check with connection V on the +input		
	jmp UH_WM_VL	; W>V, #1

WH_UV:	mov a, COMP1_B	; CP in+:(U), CP in-:(V): CP in-: P0.3(V), CP in+: P0.0(U)
	jnz WH_UM_VL	; U>V, #6
	jmp WH_VM_UL	; V>U, #5 or All"0" offset check with connection V on the +input
;-----
UH_WM_VL:		mov COMP_D, #1	; Inductive Kick is #4
		ret

UH_VM_WL:		mov COMP_D, #2	; Inductive Kick is #5
		ret

VH_UM_WL:		mov COMP_D, #3	; Inductive Kick is #6
		ret

VH_WM_UL:		mov COMP_D, #4	; Inductive Kick is #1
		ret

WH_VM_UL:		mov COMP_D, #5	; Inductive Kick is #2
		ret

WH_UM_VL:		mov COMP_D, #6	; Inductive Kick is #3
		ret
;--------------------------------------
; Motor drive each states control 
STATE1:	
;setb p0.6	; MONITOR
	mov a, DRV_SEQUENCE_CHECK_ENABLE		; Step Start routine must egnore the drive sequence 
	jz STATE1_ON_
	
	mov a, PREV_DRIVE_STATE
	cjne a, #6, SAME_DRIVE_CHK1
	jmp START_MEASURE_FRAME_TIME

SAME_DRIVE_CHK1:
	cjne a, #1, MISFIRE01
STATE1_ON_:	jmp STATE1_ON
MISFIRE01:	jmp MISFIRE1

START_MEASURE_FRAME_TIME:
		mov a, TH1
		clr c				; Noise filter if state6 to state1 drive caused twice or more.
		subb a, #MAX_BEMF_RPM_DATA_CHECK_HIGHER	; spin too fast?
		jc  STATE1_ON			; Misfired.
;
SAMPLING_SPEED_CHECK_IN_BEMF_MODE:
	mov a, SPIN_MODE
	jz RE_STORE_SPEED_DATA	; It is in Zero cross control mode as 0

SAMPLING_SPEED_CHECK:
	mov SPIN_MODE, #3
	mov a, SAMPLING_COUNTER_L	; SAMPLING_COUNTER_L is incremented in each state drive
	clr c
	subb a, #60		; Very Stable spin as more than 5 samples in one state in BEMF mode
	mov a, SAMPLING_COUNTER_H
	subb a, #0
	jc SET_BEMF2_MODE
	jmp ACCELERATION_CHECK

SET_BEMF2_MODE:
	mov SPIN_MODE, #2
;
	mov a, SAMPLING_COUNTER_L	; SAMPLING_COUNTER_L is incremented in each state drive
	clr c
	subb a, #7		; The system must be more than 1 sample spin in one state
	mov a, SAMPLING_COUNTER_H
	subb a, #0
			 
	jc OVER_SPEED_IN_BEMF_MODE
	jmp ACCELERATION_CHECK

OVER_SPEED_IN_BEMF_MODE:
	dec ON_DUTY		; Speed limit over in BEMF mode as sampling speed is not enough. 
	mov SPIN_MODE, #1	; set SPIN_MODE as Very critical spin
	call SPIN_FREE
	nop
;
;
ACCELERATION_CHECK:
	clr TR1
	mov a, TL1
	clr c
	subb a, TMR1L

	mov a, TH1
	subb a, TMR1H
	jz STABLE_SPIN
	jnc NO_MORE_ACCELERATION
	;jc ACCELERATION_IN_BEMF_MODE

ACCELERATION_IN_BEMF_MODE:
	inc ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER
	mov a, ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER
	jz AVERAGE_FLAG_OVER
	jmp CLR_SAMPLING_COUNTER

AVERAGE_FLAG_OVER:
	dec ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER
	jmp CLR_SAMPLING_COUNTER

NO_MORE_ACCELERATION:
	dec ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER
	mov a, ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER
	jz AVERAGE_FLAG_OVER
	jmp CLR_SAMPLING_COUNTER

AVERAGE_FLAG_UNDER_OVER:
	inc ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER
	jmp CLR_SAMPLING_COUNTER

STABLE_SPIN:
	mov ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER, #0

CLR_SAMPLING_COUNTER:
	mov SAMPLING_COUNTER_L, #0	; clear number of sampling counts in frame
	mov SAMPLING_COUNTER_H, #0

;ACCELERATION_MONITOR:
;	mov a, ACCELERATION_CKECK_AVERAGE_FILTER_COUNTER
;	clr c
;	subb a, #127	; Cheking the average + or - with offset(1H:2L signal ration at perfect stable codition)
;	jc UNDER
;
;OVER:	setb p0.6	; MONITOR
;	jmp RE_STORE_SPEED_DATA
;
;UNDER:	clr p0.6

RE_STORE_SPEED_DATA:
	mov TMR1L, TL1
	mov TMR1H, TH1

	call START_T1	; Timer1 always measure the one drive frame timing
	inc SPIN_COUNTER	; spin count

STATE1_ON:	setb TR1

STATE1_DRV:	mov DRIVE_STATE, #1
	call SPIN_FREE	; U=1, V=0, W=3ST
	setb P0.6		; UH(P0.6)="H"
	mov P1, #00101110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="H", WH(P1.6)="L", WL(P1.7) ="L"
	
	mov MISFIRE_COUNTER, #0
	ret

PRE_CHARGE_STATE1_DRIVER:	; for all N-ch MODFET boot strap driver circuit
PRE_CHARGE_STATE2_DRIVER:
	call SPIN_FREE	; 
	setb P0.6		; UH(P0.6)="H"
	mov P1, #00001110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="L"
			; charge up U-high-site drive power
	call WAIT_10uS
	ret

MISFIRE1:	call SPIN_FREE
	inc MISFIRE_COUNTER
	ret
;***************************
BRAKING:	call SPIN_FREE7	; U=0, V=0, W=0 <<<<<<	
	clr P0.6		; UH(P0.6)="L"	
	mov P1, #10101111b	; UL(P1.0)="H", VH(P1.4)="L", VL(P1.5)="H", WH(P1.6)="L", WL(P1.7) ="H"	
	ret
;***************************
STATE2:	
;clr p0.6	; MONITOR
	mov a, DRV_SEQUENCE_CHECK_ENABLE	; Step Start routine must egnore the drive sequence 
	jz STATE2_ON
	
	mov a, PREV_DRIVE_STATE
	cjne a, #1, SAME_DRIVE_CHK2
	jmp STATE2_ON
	
SAME_DRIVE_CHK2:
	cjne a, #2, MISFIRE2

STATE2_ON:	mov DRIVE_STATE, #2
	call SPIN_FREE
	setb P0.6		; UH(P0.6)="H"	
	mov P1, #10001110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="H"	
	
	mov a, INPUT_COUNTER_L	; inc input signal loss check timing counter 
	clr c
	add a, #1
	mov INPUT_COUNTER_L,a
	jc INC_INPUT_COUNTER_H
	ret

INC_INPUT_COUNTER_H:
	inc INPUT_COUNTER_H
	mov a, INPUT_COUNTER_H
	clr c
	subb a, #10
	jnc INPUT_SIGNAL_LOSS
	ret
	
INPUT_SIGNAL_LOSS:
	mov CONTROL_DATA, #0	;System Shut Off	
	call SPIN_FREE
	ret

MISFIRE2:	call SPIN_FREE
	inc MISFIRE_COUNTER
	ret
;-	
STATE3:	mov a, DRV_SEQUENCE_CHECK_ENABLE		; Step Start routine must egnore the drive sequence 
	jz STATE3_ON
	
	mov a, PREV_DRIVE_STATE
	cjne a, #2, SAME_DRIVE_CHK3
	jmp STATE3_ON
		
SAME_DRIVE_CHK3:
	cjne a, #3, MISFIRE3
	
STATE3_ON:	mov DRIVE_STATE, #3
	call SPIN_FREE
STATE3_DRV:
	clr P0.6		; UH(P0.6)="L"
	mov P1, #10011110b	; UL(P1.0)="L", VH(P1.4)="H", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="H"	
	ret

PRE_CHARGE_STATE3_DRIVER:
PRE_CHARGE_STATE4_DRIVER:
	call SPIN_FREE	; 
	clr P0.6		; UH(P0.6)="L"
	mov P1, #00011110b	; UL(P1.0)="L", VH(P1.4)="H", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="H"
			; UD=3ST, VD=1, WD=3ST	; charge up V-high-site drive power
	call WAIT_10uS
	ret

MISFIRE3:	call SPIN_FREE
	inc MISFIRE_COUNTER
	ret	
;-
STATE4:	mov a, DRV_SEQUENCE_CHECK_ENABLE		; Step Start routine must egnore the drive sequence 
	jz STATE4_ON

	mov a, PREV_DRIVE_STATE
	cjne a, #3, SAME_DRIVE_CHK4
	jmp STATE4_ON
	
SAME_DRIVE_CHK4:
	cjne a, #4, MISFIRE4
	
STATE4_ON:	mov DRIVE_STATE, #4
	call SPIN_FREE
STATE4_DRV:
	clr P0.6		; UH(P0.6)="L"	
	mov P1, #00011111b	; UL(P1.0)="H", VH(P1.4)="H", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="L"	
	ret

MISFIRE4:	call SPIN_FREE
	inc MISFIRE_COUNTER
	ret
;-
STATE5:	mov a, DRV_SEQUENCE_CHECK_ENABLE		; Step Start routine must egnore the drive sequence 
	jz STATE5_ON

	mov a, PREV_DRIVE_STATE
	cjne a, #4, SAME_DRIVE_CHK5
	jmp STATE5_ON
	
SAME_DRIVE_CHK5:
	cjne a, #5, MISFIRE5
	
STATE5_ON:	mov DRIVE_STATE, #5
	call SPIN_FREE
STATE5_DRV:
	clr P0.6		; UH(P0.6)="L"	
	mov P1, #01001111b	; UL(P1.0)="H", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="H", WL(P1.7) ="L"
	ret

PRE_CHARGE_STATE5_DRIVER:
PRE_CHARGE_STATE6_DRIVER:
	call SPIN_FREE	; 
	clr P0.6		; UH(P0.6)="L"	
	mov P1, #01001110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="H", WL(P1.7) ="L"	
			; UD=3ST, VD=3ST, WD=1	; charge up W-high-site drive power
	call WAIT_10uS
	ret

MISFIRE5:	call SPIN_FREE
	inc MISFIRE_COUNTER
	ret
;-
STATE6:	mov a, DRV_SEQUENCE_CHECK_ENABLE		; Step Start routine must egnore the drive sequence 
	jz STATE6_ON
	
	mov a, PREV_DRIVE_STATE
	cjne a, #5, SAME_DRIVE_CHK6
	jmp STATE6_ON
	
SAME_DRIVE_CHK6:
	cjne a, #6, MISFIRE6
	
STATE6_ON:	mov DRIVE_STATE, #6
	call SPIN_FREE
STATE6_DRV:
	clr P0.6		; UH(P0.6)="L"	
	mov P1, #01101110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="H", WH(P1.6)="H", WL(P1.7) ="L"

;;;;*****	call CHECK_TOP_CELL_VOLTAGE	; voltage down and over current check routine 
	ret

MISFIRE6:	call SPIN_FREE
	inc MISFIRE_COUNTER
	ret
;--------------------
SPIN_FREE:	
SPIN_FREE7:; U, V, W=3ST
	mov PREV_DRIVE_STATE, DRIVE_STATE ; store the previous state drive status
	clr P0.6
	anl P1, #00001110b	; UL(P1.0), VH(P1.4), VL(P1.5), WH(P1.6), WL(P1.7) ="L"	
MOS_WAIT:	call WAIT_1uS	; MOSFET gate capacitance delay + threw rate of OP amp
	call WAIT_1uS
	call WAIT_1uS
	;call WAIT_1uS
	;call WAIT_1uS
	;call WAIT_1uS
	;call WAIT_1uS
	ret

SPIN_FREE8:; U, V, W=3ST
	clr P0.6
	anl P1, #00001110b	; UL(P1.0), VH(P1.4), VL(P1.5), WH(P1.6), WL(P1.7) ="L"	
	call WAIT_1uS
	call WAIT_1uS
	call WAIT_1uS
	;call WAIT_1uS
	;call WAIT_1uS
	ret

ALL_H:	orl P0, #01000000b	; UH(P0.6)="H"
	mov P1, #01011110b	; UL(P1.0)="L", VH(P1.4)="H", VL(P1.5)="L", WH(P1.6)="H", WL(P1.7) ="L"
	ret 

TEST_PIN_ON:
	setb P2.0
	clr P2.0
	ret
;-----------------------------
CURRENT_LIMITER:	ret	; NOT AVAILABLE THE CURRENT_LIMITER
		mov a, CURRENT_LIMITER_ENABLE_FLAG
		jnz SET_DA
		ret		
		;jmp LESS_THAN_5V_CHECK	; TEST ONLY
		;checking current over at the drain voltage of Nch. MOSFET by LM339
SET_DA:		mov a, DRIVE_STATE

CURRENT_LIMIT_CHECK4:		cjne a, #4, CURRENT_LIMIT_CHECK5	; U=0, V=1, W=3ST
			jmp CURRENT_LIMIT_CHECK45
CURRENT_LIMIT_CHECK5:		cjne a, #5, CURRENT_LIMIT_CHECK6	; U=0, V=3ST, W=1
			; U=0
CURRENT_LIMIT_CHECK45:	mov a, P1	; P1.4 is Over Current V drive sensing port
		anl a, #00010000b
		jnz LESS_THAN_5V_CHECK	; OK.
		;call SPIN_FREE
		jmp I_OVER	; Current over
;-------
CURRENT_LIMIT_CHECK6:		cjne a, #6, CURRENT_LIMIT_CHECK1	; U=3ST, V=0, W=1
			jmp CURRENT_LIMIT_CHECK61
CURRENT_LIMIT_CHECK1:		cjne a, #1, CURRENT_LIMIT_CHECK2 	; U=1, V=0, W=3ST
			; V=0
CURRENT_LIMIT_CHECK61:	mov a, P2	; P2.0 is Over Current V drive sensing port
		anl a, #00000001b
		jnz LESS_THAN_5V_CHECK	; OK.
		;call SPIN_FREE
		jmp I_OVER	; Current over
;-------
CURRENT_LIMIT_CHECK2:		; U=1, V=3ST, W=0
CURRENT_LIMIT_CHECK3:		; U=3ST, V=1, W=0
			; W=0
CURRENT_LIMIT_CHECK23:	mov a, P1	; P1.3 is Over Current W drive sensing port
		anl a, #00001000b
CHECK_NCH_DRAIN_V_UP:	jnz LESS_THAN_5V_CHECK	; OK.
		;call SPIN_FREE
		jmp I_OVER	; Current over
;-----------------------------------------------
LESS_THAN_5V_CHECK:	;checking Less than 5V voltage drop or over current.
SET_D_A_1V:		mov IDA0H, #39	; 3.3mV/digit. 10bits, high byte at 0.5V on P0.1 and R is divided to 1:10
		mov a, DRIVE_STATE

LESS_THAN_5V_CHECK1:		cjne a, #1, LESS_THAN_5V_CHECK2 	; U=1, V=0, W=3ST
		jmp LESS_THAN_5V_CHECK12
LESS_THAN_5V_CHECK2:		cjne a, #2, LESS_THAN_5V_CHECK3	; U=1, V=3ST, W=0
		; U=1
LESS_THAN_5V_CHECK12:	mov CPT0MX, #00000000b	; CP in-: P0.1(D/A), CP in+: P0.0(U) W
			jmp CHECK_PCH_DRAIN_V_DROP
;-------
LESS_THAN_5V_CHECK3:		cjne a, #3, LESS_THAN_5V_CHECK4	; U=3ST, V=1, W=0
		jmp LESS_THAN_5V_CHECK34
LESS_THAN_5V_CHECK4:		cjne a, #4, LESS_THAN_5V_CHECK5	; U=0, V=1, W=3ST
		; V=1
LESS_THAN_5V_CHECK34:	mov CPT0MX, #00000001b	; CP in-: P0.1(D/A), CP in+: P0.2(V) 
			jmp CHECK_PCH_DRAIN_V_DROP
;-------
LESS_THAN_5V_CHECK5:	; W=1 at state 5 and 6
		; W=1
LESS_THAN_5V_CHECK56:	mov CPT0MX, #00000010b	; CP in-: P0.1(D/A), CP in+: P0.4(W)
CHECK_PCH_DRAIN_V_DROP:
			call WAIT_1uS	; Comparator and D/A settling time 5uS
			call WAIT_1uS	; Comparator and D/A settling time
			call WAIT_1uS	; Comparator and D/A settling time
			call WAIT_1uS	; Comparator and D/A settling time
			call WAIT_1uS	; Comparator and D/A settling time
			mov a, CPT0CN	; Get Comparator result 
			anl a, #01000000b	; Get CP0OUT bit
		jnz OVER_Pch_OUT_5V	; OK. 

LESS_THAN_5V:	;call TEST_PIN_ON	; Less than 5V voltage drop or over current.
		;call SPIN_FREE	; TEST ONLY
		;mov a, DRIVE_STATE	; Monitor Only
		mov a, CONTROL_DATA
		clr c
		subb a, #2
		jc LO_VOL
		mov CONTROL_DATA_MAX, a	; Power limitter
LO_VOL:		ret

OVER_Pch_OUT_5V:	mov a, CONTROL_DATA_MAX	; No volatge drop or over current
		clr c
		subb a, #100
		jnc MAX_LIMIT
		
		mov a, CONTROL_DATA
		inc a
		inc a
		;inc a		; two more "inc" means for stick feeling better
		mov CONTROL_DATA_MAX, a
MAX_LIMIT:		ret

I_OVER:		;call TEST_PIN_ON	; Less than 5V voltage drop or over current.
		;call SPIN_FREE	; TEST ONLY
		;mov a, DRIVE_STATE	; Monitor Only
		mov a, CONTROL_DATA
		clr c
		subb a, #1
		jz I_LIMIT
		mov CONTROL_DATA_MAX, a	; Power limitter
I_LIMIT:		ret	
;---------------------------------------
S_WDT_OFF:	mov PCA0CPH2, #0	; Clear the watch dog timer in sound mode
	ret

WDT_OFF:	mov PCA0CPH2, #0	; Clear the watch dog timer in spin mode
	ret		; No Li-Po protection function.

;---------------
AD_START:	nop
	nop
	nop
	nop
	nop	; wait 200nS

	clr AD0INT
	setb AD0BUSY	; Start the AD conversion.
	nop	; No reason. Need for monitoring. Why??????
	ret

AD_END_CHK:	jnb AD0INT, AD_END_CHK	; AD complete?
	ret

; End of file.
END