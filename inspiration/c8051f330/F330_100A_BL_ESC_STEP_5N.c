//-----------------------------------------------------------------------------
// F330_100A_BL_ESC_STEP_5N.c "BEMF + Zero cross control" with Pull-down while PWM off
//-----------------------------------------------------------------------------
// Copyright (C) 2007 Takao
//
// AUTH: Takao
// DATE: Jan 25th, 2007
//
// Target:         C8051F330
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <c8051f330.h>
// The differences between F41x are
// PCA0CPH5 -> PCA0CPH2 (Watch dog timer)
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for C8051Fxxx
//-----------------------------------------------------------------------------
sfr16 TMR2RL	= 0xca; // TMR2RLL = 0xca, TMR2RLH = 0xcb. Timer2 reload register
sfr16 TMR2		= 0xcc; // TMR2L = 0xcc, TMR2H = 0xcd. Timer2 counter
sfr16 TMR3RL	= 0x92; // TMR3RLL = 0x92, TMR3RLH = 0x93. Timer3 reload register
sfr16 TMR3		= 0x94;	// TMR3L = 0x94, TMR3H = 0x95. Timer3 counter
//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
// Starting parameters
#define STEPPING_ON_DUTY_CHOPPING_COUNT 30 //Init Stepping starting current limit chopping count
#define STEPPING_MAX_SPEED_CHOPPING_COUNT 3
#define START_STEPPING_DUTY_FRAME 1500	//*24.5/12MHz clock=490nS  Start duty checking Duty frame
#define STEPPING_ON_DUTY_RATIO 30	// x:1. STEPPING_ON_DUTY = START_STEPPING_DUTY_FRAME/STEPPING_ON_DUTY_RATIO
									// Init Current limit setting in start step drive as on-duty setting 
// Propo-input data parameters 
#define CTRL_MIN 2040 	//1mS. MIN_OFFSET/0.49(=12/SYSCLK); number of timer counts PCA 1/12 sys.clk in 
						//capture range sample(0800h - 1065h)
#define THROTTLE_OFF 8	// CONTROL_DATA

//BEMF+ ZC spin parameters
#define BEMF_DUTY_INC_TIMING 200	// BEMF power step up increasing timing frame on Timer 2
#define PWM_FRAME_1kHz 2040	// 1kHz sampling at starting
#define PWM_FRAME_2kHz 1020	// 2kHz sampling at starting
#define PWM_FRAME_3kHz 680	// 3kHz sampling at starting
#define PWM_FRAME_5kHz 408	// 5kHz sampling at starting
#define PWM_FRAME_8kHz 255	// 8kHz sampling at starting
#define PWM_FRAME_13kHz 157	// 13kHz sampling at starting
#define PWM_FRAME_20kHz 102	// 20kHz sampling at starting

#define CHECK_ZC_AT_PWM_ON_DUTY 50
		// Change to zero cross sensing point to the end of ON duty.
		// At slow rpm, The ZC sampling point is the end of OFF duty.

#define STICK_RESOLUTION 100
		// PWM resolution vs stick control range 
//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void Step_Drive_Count (void);
void Step_Start (void);
void Stepping_Drive_On_Timer (void);
void Stepping_Off_Timer (void);

void Wait_1250nS (COUNTS);	//1.25uS at (0) timer
void ON_Sound (void);	// 
void Spin_Free (void);	// 	
void State1_On (void);	// 
void State2_On (void);	// 
void State3_On (void);	// 
void State4_On (void);	// 
void State5_On (void);	// 
void State6_On (void);	// 
void Braking (void);	//
void Spin_Free_N (void);

void PCA_ISR (void);	// PCA Interrupt Service Routine for Rx in
void T1_ISR (void);

void BEMF_Drive (void);
void Power_Up_Timer (void);
void T1_PWM_OFF_DUTY (void);
void T1_PWM_ON_DUTY (void);
void Get_Comp_Data (void);	//
void Comp_Measure (void);	//
void Check_Free_Spinning (void);
void Start_T0 (void);

void Set_comparator_input_for_ZC_detect (void);
void Zero_Cross_Drive (void);
void Set_to_next_drive (void);
//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
unsigned char L1;	// Used in ON_Sound (),
unsigned char L2;	// Used in ON_Sound (), Step_Drive_Count (), Get_Comp_Data ()
unsigned int TEMP0;	// Used in ON_Sound (), Stepping_Drive_On_Timer (), Comp_Measure ()
unsigned int TEMP1;	// Used in ON_Sound (), 
unsigned char TEMP2;	// Used in ON_Sound (), Stepping_Drive_On_Timer ()
unsigned char TEMP3;	// Used in Check_Free_Spinning ()
unsigned int TEMP16;	// Used in Stepping_Drive_On_Timer (), T1_PWM_ON_DUTY (), T1_PWM_OFF_DUTY (
unsigned int TEMP16S;

unsigned int COUNTS; //
unsigned int DELAY_COUNTS;
unsigned char START_DRIVE_STATE;
unsigned int  STEPPING_ON_DUTY;

unsigned char PCA0_CAP_L;
unsigned char PCA0_CAP_H;
unsigned int PCA0_CAP;
signed int PULSE_WIDTH;
unsigned char CONTROL_DATA_0;
unsigned char CONTROL_DATA;
unsigned char CONTROL_DATA_MAX;
unsigned char PROPO_PULSE_IN_COUNTER;

unsigned char ON_DUTY;
unsigned int PWM_FRAME;
unsigned char DRIVE_STATE;
unsigned char PREV_DRIVE_STATE;

unsigned char NUMBER_OF_SAMPLING;
unsigned int PHASE_AVERAGE_T;
unsigned char SAMPLING_COUNT;

unsigned int ZC_OFF_DUTY;

bit LOST_ZC;	// could not catch the zero cross point by may be too much rpm
bit SPINNING;
bit T1_Op_Flag;
bit ZERO_CROSS_RUN;
bit OVER_ZERO_CROSS;
bit STATE_CHANGES; 
//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------
void main (void) 
{
	PCA0MD &= ~0x40;		// WDTE = 0 disable watchdog timer

// Initialize crossbar and GPIO
    // P0.0  -  Skipped,     Open-Drain, Analog
    // P0.1  -  Skipped,     Open-Drain, Analog
    // P0.2  -  Skipped,     Open-Drain, Analog
    // P0.3  -  Skipped,     Open-Drain, Analog
    // P0.4  -  Skipped,     Open-Drain, Analog
    // P0.5  -  Skipped,     Open-Drain, Analog
    // P0.6  -  Skipped,     Push-Pull,  Digital
    // P0.7  -  CEX0 (PCA),  Open-Drain, Digital

    // P1.0  -  Skipped,     Push-Pull,  Digital
    // P1.1  -  Skipped,     Open-Drain, Analog
    // P1.2  -  Skipped,     Open-Drain, Analog
    // P1.3  -  Skipped,     Open-Drain, Analog
    // P1.4  -  Unassigned,  Push-Pull,  Digital
    // P1.5  -  Unassigned,  Push-Pull,  Digital
    // P1.6  -  Unassigned,  Push-Pull,  Digital
    // P1.7  -  Unassigned,  Push-Pull,  Digital

    P0MDIN    = 0xC0;
    P1MDIN    = 0xF1;
    P0MDOUT   = 0x40;
    P1MDOUT   = 0xF1;
    P2MDOUT   = 0x01;
    P0SKIP    = 0x7F;
    P1SKIP    = 0x0F;
    XBR1      = 0x41;

	OSCICN = 0x83;			// Configure 24.5MHz internal oscillator

// PCA0_INT:	; Programable Counter Array is used in Edge-triggered Caputure Mode for power control input from Rx.
	PCA0MD = 0;			//#00000000b	; PCA0 works in 1/12 system clock
	PCA0CN |= 0x40;		//#01000000b	; Run Programmable Counter 0 
	PCA0CN &= 0xfc;		//#11111100b	; Clear PCA Module 0 and 1 Capture/Compare Flag for acceleration meter
	PCA0CPM0 |=0x21;	//#00100001b	; PCA0 is captured by positive edge first on P0.7, Also generates the interrupt.
	EIE1 |= 0x10;		//#00010000b	; enable PCA0 interrupts 
   	CCF0 = 0;			//             	; clear PCA0 module0 interrupt flag
//	EIP1 |= 0x10;		//#00010000b	; PCA0 innterrupt set to high priority level.
	PCA0CPM1 |= 0x48;	//#01001000b	; PCA Module 1 Capture/Compare Flag enable for acceleration meter offset adj. timing
//
	TMOD = 0x11;		// #00010001b	; Set Timer0,1 16 bit counter mode
//

	PCA0CPL2 = 32;		// The watch dog timing is 4mS interval
	PCA0MD |= 0xc0; 	//	#11000000b	; PCA suspend while CPU is supended and WDT is "ON".
//
//	Wake_Up_comparator:
	CPT0CN |= 0x80;		//	#10000000b	; bit7: Comp. Enable, bit6: Comp out, bit5,4: no interrupt, bit3-0: no hys.
	CPT0MD = 0;			// 100nS responce
//
//PWR ON_Sound:
	TEMP0 = 40;			// 10% On duty
	TEMP1 = 360;		// Off duty
	TEMP2 = 45;			// frame counts 
	ON_Sound();
//
	CONTROL_DATA = 0;
	CONTROL_DATA_MAX = 255; 
	PROPO_PULSE_IN_COUNTER = 0;	// Propo signal-in counter to make sure the initial control data
	EA = 1;            // Enable global interrupts
//
	while ( PROPO_PULSE_IN_COUNTER < 10 )	// make sure the stick off position
		{
		PCA0CPH2 = 0;	// Watch dog timer off. 
		}

//Propo Signal-in Sound:
	TEMP0 = 20;			// 20% On duty
	TEMP1 = 80;			// Off duty
	TEMP2 = 90;			// frame counts 
	EA = 0;				// Disable global interrupts for clear sound
	ON_Sound();
	EA = 1;				// Enable global interrupts		
//
	CKCON &= 0xbf; 		//anl CKCON, #10111111b	; Timer0,1,3 works with 1/12 system clock
	P2 = 0; // Monitor ONLY
//;--------------
STEPPING_START:
//;--------------
	PCA0CPL2 = 32;		// The watch dog timing is 4mS interval
	ET1 = 0 ;	// Disable Timer1 interrupt for TEMP16 PWM control use 

	while ( CONTROL_DATA <= THROTTLE_OFF )		//Wait until stick ON
		{
		PCA0CPH2 = 0;	// Watch dog timer off. 
		}
//
	Check_Free_Spinning ();
	if ( SPINNING == 1 )
		{
		goto BEMF_SPIN;
		}
// Energize the stepping drive
	//START_DRIVE_STATE = 1;
	Get_Comp_Data ();	// for smooth start after BEMF run
	TEMP2 = STEPPING_ON_DUTY_CHOPPING_COUNT;	
	TEMP16 = START_STEPPING_DUTY_FRAME;
	TEMP0 = STEPPING_ON_DUTY_RATIO;
	//
	while( TEMP2 > STEPPING_MAX_SPEED_CHOPPING_COUNT ) // Spin until Stepping max speed
		{ 
		START_DRIVE_STATE ++;
		if ( START_DRIVE_STATE == 7 )
			{
			START_DRIVE_STATE = 1;
			}
		Step_Drive_Count ();
		//
		if ( CONTROL_DATA > 10 )	// If the stick is between 8-10, then very slow step spins
			{
			TEMP2 --;	// acceleration for starting speed
			if ( TEMP0 >= 10 )	// max 10% starting power duty
				{
				TEMP0 --;	// acceleration with power-up
				}
			}
		//
		if ( CONTROL_DATA < THROTTLE_OFF )	// Stick off?
			{
			break;
			}
		}
//	
	Spin_Free ();
//
	if ( CONTROL_DATA < THROTTLE_OFF )	// Stick off?
		{
		goto STEPPING_START;
		}
//-----------------
BEMF_SPIN: // Also run ZERO-Cross spin  
//-----------------
	CKCON &= 0xcc;	//#11001100b	; Timer0,1,2 with 1/12 sys. clock.
	ET1 = 1 ;	// Eanble Timer 1 interrupt PWM power control 
	T1_Op_Flag = 0;
	PWM_FRAME = PWM_FRAME_1kHz; // 1kHz sampling at starting
	ON_DUTY = THROTTLE_OFF+2;
	SAMPLING_COUNT = 0;
	T1_Op_Flag = 0;	// on 
	T1_PWM_OFF_DUTY ();
	Start_T0 ();	// start Timer0 for duty increase delay control
	ON_DUTY = THROTTLE_OFF+2;
	ZERO_CROSS_RUN = 0;
//
	while ( CONTROL_DATA >= THROTTLE_OFF ) // Wait untill throttle stick off
		{
		if ( ZERO_CROSS_RUN == 0 )
			{
			BEMF_Drive ();
			}
		else
			{
			Step_Start ();	// zero cross spin				
			}
		//
		if ( PREV_DRIVE_STATE == DRIVE_STATE )
			{
			SAMPLING_COUNT ++;		
			}
		else
			{
/////////////////////////////////////////////////////////////////
			if ( DRIVE_STATE == 3 ) // Scope trigger
				{
				P2=1; P2=0;
				}
///////////////////////////////////////////////////////////////*/
			STATE_CHANGES = 1;
			SAMPLING_COUNT = 1;
			NUMBER_OF_SAMPLING = 0;
			}
		//
		Set_comparator_input_for_ZC_detect ();
		Power_Up_Timer ();
		//
		if ( ON_DUTY == THROTTLE_OFF + 4 )
			{
			PWM_FRAME = PWM_FRAME_5kHz;
			}
		if ( ON_DUTY >= THROTTLE_OFF + 10 )
			{
			if ( PWM_FRAME > PWM_FRAME_13kHz)
				{
				L1++;	// For Smooth PWM freq change
				if ( L1 > 3 )
					{
					PWM_FRAME --;				
					T1_PWM_ON_DUTY ();
					L1 = 0;
					}
				}
			}
		//
		if ( PWM_FRAME <= PWM_FRAME_8kHz )
			{
			ZERO_CROSS_RUN = 1;
			CPT0CN |= 0x0a;	// 10mV Hysteresis
			}
		if ( PWM_FRAME > PWM_FRAME_8kHz )
			{
			ZERO_CROSS_RUN = 0;	// BEMF_RUN
			CPT0CN &= 0xf0;	// 0mV Hysteresis			
			}
		//
		if ( CONTROL_DATA < THROTTLE_OFF )	// Stick off?
			{
			Spin_Free ();	// for BEMF sensin
			break;
			}
		PCA0CPH2 = 0;	// Watchdog off
		//
	//ON_TIMER ---------------------------------		
		while ( T1_Op_Flag == 1);	// on timer
		if ( ON_DUTY > CHECK_ZC_AT_PWM_ON_DUTY )
			{
			Zero_Cross_Drive ();
			}
		Spin_Free_N ();	// set power off to get BEMF signal with pull-up high site MOSFET
	// Off Timer --------------------------------
		if ( T1_Op_Flag == 0 )
			{
			while ( T1_Op_Flag == 0 );
			}
		//else 	// Already Off time is finished by top speed power setting
		//
		if ( ZERO_CROSS_RUN == 0 )
			{	
			Spin_Free ();	// for BEMF sensing	
			}
		//
		if ( ON_DUTY <= CHECK_ZC_AT_PWM_ON_DUTY )
			{
			Zero_Cross_Drive ();
			}
		//			
		if ( OVER_ZERO_CROSS == 1 & STATE_CHANGES == 1 & SAMPLING_COUNT >= 1 )
			{
			NUMBER_OF_SAMPLING = SAMPLING_COUNT;
			SAMPLING_COUNT = 1;	// timing advance
			STATE_CHANGES = 0;
			}
		//
		PREV_DRIVE_STATE = DRIVE_STATE;
		// Zero Cross state change control with 33% timing advance
		if ( SAMPLING_COUNT >= NUMBER_OF_SAMPLING*1/3 & NUMBER_OF_SAMPLING > 0 )
			{ 
			Set_to_next_drive ();
			}
		}
	goto STEPPING_START;
}
//-----------------------------------------------------------------------------
void Zero_Cross_Drive (void)
{
	switch ( DRIVE_STATE )	// get zero cross point by comparator
		{
		case 1:
		case 2:
		case 3:
			{
			if ( CPT0CN & 0x40 )	// #01000000b Get CPOUT bit to ZERO_CROSS_CHK_DOWN
				{
				//P2=1;
				OVER_ZERO_CROSS = 0;
				}
			else
				{
				//P2=0;
				OVER_ZERO_CROSS = 1;
				}
			}
		break;
		//
		case 4:
		case 5:
		case 6:
			{
			if ( ~CPT0CN & 0x40 )	// #01000000b Get CPOUT bit to ZERO_CROSS_CHK_UP
				{
				//P2=1; // Monitor ONLY
				OVER_ZERO_CROSS = 0;
				}
			else
				{
				//P2=0; // Monitor ONLY
				OVER_ZERO_CROSS = 1;
				}			
			}
		break;
		}
}
//------------------------
void Set_comparator_input_for_ZC_detect (void)	// configure the comparator pin asign
{
	switch ( START_DRIVE_STATE ) // assign comparator input pin 
		{
		case 1:
			{
			CPT0MX = 0x52;	//#01010010b	; CP in-: P1.3(HALFW), CP in+: P0.4(W) with DOWN slope
			}
		break;
		//
		case 2:
			{
			CPT0MX = 0x15;	//#00010101b	; CP in-: P0.3(V), CP in+: P1.2(HALFV)with UP slope
			}
		break;
		//
		case 3:
			{
			CPT0MX = 0x40;	//#01000000b	; CP in-: P1.1(HALFU), P0.0(U) with DOWN slope
			}
		break;
	//
		case 4:
			{
			CPT0MX = 0x52;	//#01010010b	; CP in-: P1.3(HALFW), CP in+: P0.4(W) with UP slope
			}
		break;
	//
		case 5:
			{
			CPT0MX = 0x15;	//#00010101b	; , CP in-: P0.3(V), CP in+: P1.2(HALFV) with DOWN slope
			}
		break;
	//
		case 6:
			{
			CPT0MX = 0x40;	//#01000000b	; CP in-: P1.1(HALFU), CP in+: P0.0(U) with UP slope
			}
		break;
	//
	}
}
//-----------------------------------------------------------------------------
void BEMF_Drive (void)
{
	Get_Comp_Data ();
	Step_Start ();
}
//
void Set_to_next_drive (void)
{
	DRIVE_STATE ++;
	if ( DRIVE_STATE == 7 )
		{
		START_DRIVE_STATE = 1;
		DRIVE_STATE = 1;
		}
	else
		{
		START_DRIVE_STATE = DRIVE_STATE;	
		}
}
//-----------------------------------------------------------------------------
// Checking free spinning before stepping re-start drive
void Check_Free_Spinning (void)
{
	Spin_Free();	// consider the leak courrent while slow spin as DC drift	

	CKCON |= 0x02;	//#00000010
	CKCON &= 0xfe;	//#11111110 // Timer0,1 uses 1/48 sysclock

	CPT0CN |= 0x0f;	//00001111b 20mV hysteresis

	Start_T0 ();
	TEMP3 = 0; 		// spinning state flags	
	//
	//	while ( TF0 == 0)	// CHECK_FREE_SPIN in 128mS
		while ( TH0 < 128 )	// CHECK_FREE_SPIN in 64mS
		{
		Get_Comp_Data ();
		PCA0CPH2 = 0;	// Watchdog off
		switch ( START_DRIVE_STATE )
			{
			case 1:
			TEMP3 |= 0x01;	// #00000001b
			break;
			//
			case 2:
			TEMP3 |= 0x02;	// #00000010b
			break;
			//
			case 3:
			TEMP3 |= 0x04;	// #00000100b
			break;
			//
			case 4:
			TEMP3 |= 0x08;	// #00001000b
			break;
			//
			case 5:
			TEMP3 |= 0x10;	// #00010000b
			break;
			//
			case 6:
			TEMP3 |= 0x20;	// #00100000b
			break;
			}
		}
//CHECK_ALL_SPIN_PATTERN:
	if ( TEMP3 == 0x3f ) // #00111111b	; spin with all pattern?
		{
		SPINNING = 1;
		}
	else
		{
		SPINNING = 0;
		}			

	CKCON &= 0xfc;	//#11111100 // Timer0,1 uses 1/12 sysclock
	CPT0CN &= 0xf0;	// 0mV hysteresis
}
//---------
void Get_Comp_Data (void)
{	
	L2 = 0;		// never "0" or "7" while getting the comparator data
	START_DRIVE_STATE = 7;	// never "0" or "7" while getting the comparator data

	while (START_DRIVE_STATE != L2) // Wait untill stable comparator output data
	{
	Comp_Measure ();
	L2 = START_DRIVE_STATE;	
	Comp_Measure ();	// RE_CHECK the comparator result data
	}
}
//
void Comp_Measure (void)	// P0.2 and P0.3(V), P0.4 and P0.5(W) are connected togather by solder.
{
//COMP_A1:
	CPT0MX = 0x20;	//#00100000b	; CP in-: P0.5(W), CP in+: P0.0(U)
	Wait_1250nS (1);
	TEMP0 = CPT0CN & 0x40;	//#01000000b	; Get CPOUT bit

//COMP_B1:
	CPT0MX = 0x10;	//#00010000b	; CP in-: P0.3(V), CP in+: P0.0(U) 
	Wait_1250nS (1);
	TEMP1 = CPT0CN & 0x40;	//#01000000b	; Get CPOUT bit

//COMP_C1:
	CPT0MX = 0x21; 	//#00100001b	; CP in-: P0.5(W), CP in+: P0.2(V)
	Wait_1250nS (1);
	TEMP2 = CPT0CN & 0x40;	//#01000000b	; Get CPOUT bit

//COMP_LOGIC://UW	
	if (TEMP0 == 0)	// CP in+:(U), CP in-:(W):
	{//	jmp WV		; W>U
		if (TEMP2 == 0)	// WV:
		{//	jmp WH_UV		; W>V 
			if (TEMP1 == 0) //WH_UV
			{//	jmp WH_VM_UL	; V>U, #5 or All"0" offset check with connection V on the +input
			START_DRIVE_STATE = 5; //Inductive Kick is #2
			}
			else
			{//	jnz WH_UM_VL	; U>V, #6
			START_DRIVE_STATE = 6;	// Inductive Kick is #3
			}
		}
		else
		{START_DRIVE_STATE = 4;	// Inductive Kick is #1
		}
	}
	else
	{//	jnz UV		; U>W
		if (TEMP1 == 0) // UV
		{//	jmp VH_UM_WL	; V>U, #3	
		START_DRIVE_STATE = 3;	// Inductive Kick is #6
		}
		else
		{//	jnz UH_VW		; U>V	
			if (TEMP2 == 0) // UH_VW
			{//	jmp UH_WM_VL	; W>V, #1
			START_DRIVE_STATE = 1;	// Inductive Kick is #4
			}
			else
			{//	jnz UH_VM_WL	; V>W, #2 or All"1" offset check with connection V on the +input
			START_DRIVE_STATE = 2;	// V>W, #2 or All"1" offset check with connection V on the +input. Inductive Kick is #5
			}		
		}
	}
}
//-----------------------------------------------------------------------------
// Timing Controls
//-----------------------------------------------------------------------------
void Wait_1250nS (COUNTS)
{
	for (DELAY_COUNTS = 1; DELAY_COUNTS <= COUNTS; DELAY_COUNTS++)
		{	// 1 time loop delay is 1.25uS include call function
		}
}
//
void Start_T0 (void)	// no re-load function in. And can NOT use TMR0 sfr
// Timer0 16bit counter with pre-scaled clock is 24.5MHz/12=490nS.
{
	TR0 = 0;		// Stop T0
	TL0 = 0;		// Clear T0
	TH0 = 0;
	TF0 = 0;		// Clear over flow flag
	TR0 = 1;		// Start T0
}
//
void T1_PWM_OFF_DUTY (void)	// no re-load function in T1
//; Timer1 16bit counter with pre-scaled clock is 24.5MHz/12=490nS.
{
	TR1 = 0;		// Stop T1
	TEMP16S = ~(PWM_FRAME - ~TEMP16);	// set T1
	TL1 = TEMP16S & 0x00ff;
	TH1 = TEMP16S / 256;
	TF1 = 0;		// Clear over flow flag
	TR1 = 1;		// Start T1
}
void T1_PWM_ON_DUTY (void)	// no re-load function in T1
//; Timer1 16bit counter with pre-scaled clock is 24.5MHz/12=490nS.
{
	TR1 = 0;		// Stop T1
	if ( ON_DUTY <= STICK_RESOLUTION )
		{
		TEMP16 = ~(PWM_FRAME*ON_DUTY/STICK_RESOLUTION);	// set T1
		}
	TL1 = TEMP16 & 0x00ff;
	TH1 = TEMP16 / 256;	//& 0xff00;
	TF1 = 0;		// Clear over flow flag
	TR1 = 1;		// Start T1
}
//
/*
void Start_T2 (void)	// zero cross detect timer
{
	TR2 = 0;	// Stop T2
	TMR2RL = 0xffff;	// re-load data
	TMR2 = 0;	// Clear T2 
	TF2H = 0;	// High byte flag off
	TF2L = 0;	// Low byte flag off
	TR2 = 1;	//Start T2
}

void Start_T3 (void)	//
{
	TMR3CN &= 0x08;	// 00001000b.  Stop T3 and clear all flags with 16 bit operation 
	TMR3RL = 0xffff;	// re-load data
	TMR3 = 0;	// Clear T3 
	TMR3CN |= 0x04;	// 00000100b Start T3
}
*/
//;-----------------------
void Power_Up_Timer (void)
{
	if ( TH0 > BEMF_DUTY_INC_TIMING )	// get slow responce control for stability by T0
		{
		Start_T0 ();	// re-start Timer0
		//
		if ( CONTROL_DATA > ON_DUTY )	// DUTY > Rx input pulse?	
			{
			ON_DUTY ++;	// increase ON_DUTY
			}
		}
	//
	if ( CONTROL_DATA < ON_DUTY )
		{
		if ( ON_DUTY > THROTTLE_OFF)
			{
			ON_DUTY --;	// decrease ON_DUTY
			}
		}
}
//;-----------------------
void Step_Drive_Count (void)
{
	for ( L2=1; L2 < TEMP2; L2++ ) // TEMP2 is STEPPING_ON_DUTY_CHOPPING_COUNTER
		{
		Step_Start ();
		Stepping_Drive_On_Timer ();	//STEPPING On timer
		Stepping_Off_Timer ();
		}
}
//;------
void Step_Start (void)
{
		switch (START_DRIVE_STATE) 
		{
		case 1:
			State1_On ();	//U=1, V=0, W=3ST
		break;

		case 2:
			State2_On ();	// U=1, V=3ST, W=0	
		break;

		case 3:
			State3_On ();	// U=3ST, V=1, W=0
		break;

		case 4:
			State4_On ();	// U=3ST, V=1, W=0
		break;

		case 5:
			State5_On ();	// W=1
		break;

		case 6:
			State6_On ();	// W=1
		break;
		}
}
//
//Stepping_Drive_On_Timer:
void Stepping_Drive_On_Timer (void)
{
	TMR3CN &= 0x3b;	//anl TMR3CN, #00111011b	; clr TF3H, clr TF3L and clr TR3
	STEPPING_ON_DUTY = TEMP16/TEMP0;
//CALC_STEPPING_OFF_DUTY_TIME:
	TMR3RL = ~(START_STEPPING_DUTY_FRAME - STEPPING_ON_DUTY);

//CALC_STEPPING_ON_DUTY_TIME:
	TMR3 = ~STEPPING_ON_DUTY;

	TMR3CN |= 0x04; 	// orl TMR3CN, #00000100b	; setb TR3 to start T3

//SAMPLE_ON_Loop:
	while (~TMR3CN & 0x80)	// get bit7 as TF3H T3 High byte over Flow
	{
	PCA0CPH2 = 0;
	}	// Watchdog off} 

	TMR3CN &= 0x3f;	// anl #00111111b	; clr TF3H, TF3L 
}
//------------			
void Stepping_Off_Timer (void)
{	
	Braking ();
	//Spin_Free();
	while (~TMR3CN & 0x80);	// get TF3H T3 High byte over Flow
//	Get_Start_Drive_State ();
	TMR3CN &= 0x3f;	// anl #00111111b	; clr TF3H, TF3L 
	PCA0CPH2 = 0; 	// Watchdog off
}
//
// Sound Generator
void ON_Sound (void)
{
for (L1=1 ; L1<= 3;L1++)	// 3 times sound
	{
	for (L2=1; L2<TEMP2; L2++)
		{
		PCA0CPH2 = 0;	// watch dog reset
		State1_On();  
		Wait_1250nS (TEMP0);	// On duty Timer
		Spin_Free ();
		Wait_1250nS (TEMP1);	// Off duty Timer
		//
		State4_On ();  
		Wait_1250nS (TEMP0);	// On duty Timer
		Spin_Free ();
		Wait_1250nS (TEMP1);	// Off duty Timer
		//
		State3_On ();  
		Wait_1250nS (TEMP0);	// On duty Timer
		Spin_Free ();
		Wait_1250nS (TEMP1);	// Off duty Timer
		//
		State6_On ();  
		Wait_1250nS (TEMP0);	// On duty Timer
		Spin_Free ();
		Wait_1250nS (TEMP1);	// Off duty Timer
		//
		State5_On ();  
		Wait_1250nS (TEMP0);	// On duty Timer
		Spin_Free ();
		Wait_1250nS (TEMP1);	// Off duty Timer
		//
		State2_On ();  
		Wait_1250nS (TEMP0);	// On duty Timer
		Spin_Free ();
		Wait_1250nS (TEMP1);	// Off duty Timer
		}
//
	for (L2=1; L2<=50;L2++)
		{
		PCA0CPH2 = 0;	// watch dog reset
		Wait_1250nS (2000);	// 2.5 S Timer
		}
	}
}
//--------------------------------------
// Motor drive each state control 
//--------------------------------------
void State1_On (void)	
{
	Spin_Free (); // All MOSFTEs off
	P0 |= 0x40;	//mov P0, #01000000 	; UH(P0.6)="H"
	P1 = 0x2e;	//mov P1, #00101110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="H", WH(P1.6)="L", WL(P1.7) ="L"
	DRIVE_STATE = 1; //
}
//
void State2_On (void)	
{				
	Spin_Free (); // All MOSFTEs off
	P0 |= 0x40;	//mov P0, #01000000 	; UH(P0.6)="H"	
	P1 = 0x8e;	//mov P1, #10001110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="H"	
	DRIVE_STATE = 2; 
}
//
void State3_On (void)	
{				
	Spin_Free (); // All MOSFTEs off
	P0 &= 0xbf;	// #101111111b	; UH(P0.6)="L"
	P1 = 0x9e;	//mov P1, #10011110b	; UL(P1.0)="L", VH(P1.4)="H", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="H"	
	DRIVE_STATE = 3; 
}
//	
void State4_On (void)	
{				
	Spin_Free (); // All MOSFTEs off
	P0 &= 0xbf;	// UH(P0.6)="L"	
	P1 = 0x1f;	// mov P1, #00011111b	; UL(P1.0)="H", VH(P1.4)="H", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="L"	
	DRIVE_STATE = 4; 
}
//
void State5_On (void)	
{				
	Spin_Free (); // All MOSFTEs off
	P0 &= 0xbf;	//clr P0.6		; UH(P0.6)="L"	
//	P1 = 0x4f;	//mov P1, #01001111b	; UL(P1.0)="H", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="H", WL(P1.7) ="L"
	P1 = 0x4f;	//mov P1, #01001111b	; UL(P1.0)="H", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="H", WL(P1.7) ="L"
	DRIVE_STATE = 5; 
}
//
void State6_On (void)	
{				
	Spin_Free (); // All MOSFTEs off
	P0 &= 0xbf;	// clr P0.6		; UH(P0.6)="L"	
	P1 = 0x6e;	// mov P1, #01101110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="H", WH(P1.6)="H", WL(P1.7) ="L"
	DRIVE_STATE = 6; 
}
//
void Spin_Free (void) // All output MOSFETs off
{
	P0 &= 0xbf;	// clr P0.6
	P1 &= 0x0e;	// anl P1, #00001110b	; UL(P1.0), VH(P1.4), VL(P1.5), WH(P1.6), WL(P1.7) ="L"	
	Wait_1250nS(1);	// MOSFETs off time
}
//
void Spin_Free_N (void) //
{
	switch ( DRIVE_STATE )
	{
		case 1:
		case 6:	//VL
		P0 &= 0xbf;	// #10111111 	; //UH = "L"
		P1 &= 0x0e;	// #00101110	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="H", WH(P1.6)="L", WL(P1.7) ="L"
		break;

		case 2:
		case 3:	//WL
		P0 &= 0xbf;	// VH	
		P1 &= 0x8e;	// #10001110b	; UL(P1.0)="L", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="H"	
		break;

		case 4:
		case 5:	//UL	
		P0 &= 0xbf;	// 	
		P1 &= 0x0f;	// #00001111b	; UL(P1.0)="H", VH(P1.4)="L", VL(P1.5)="L", WH(P1.6)="L", WL(P1.7) ="L"
		break;
	}
}

//
void Braking (void)	
{				
	Spin_Free (); // All MOSFTEs off
	P0 &= 0xbf;	// clr P0.6		; UH(P0.6)="L"	
	P1 = 0xaf;	//mov P1, #10101111b	; UL(P1.0)="H", VH(P1.4)="L", VL(P1.5)="H", WH(P1.6)="L", WL(P1.7) ="H"	
}

//-----------------------------------------------------------------------------
// Interrupt Service Routines to get Rx input pulse width
//-----------------------------------------------------------------------------
void PCA_ISR (void) interrupt 11
{
	CCF0 = 0;	// clear PCA0 module0 interrupt flag

//Capture_Edge_check:
	if ((PCA0CPM0 & 0x20) == 0)
		{	// fall edge
		PCA0CPM0 = 0x21;	// #00100001b
							// Next PCA0 will be captured by positive edge on P0.6 first and generates the interrupt.
		PULSE_WIDTH = PCA0CPH0<<8;	// set to high byte
		PULSE_WIDTH += PCA0CPL0; // set to 16bit data 

		PCA0_CAP = PCA0_CAP_H<<8;
		PCA0_CAP += PCA0_CAP_L;

		PULSE_WIDTH -= PCA0_CAP;
		PULSE_WIDTH -= CTRL_MIN;

		if ( PULSE_WIDTH < 0 )
			{
			CONTROL_DATA = 0;			
			PULSE_WIDTH = 0;
			}
		//
 		if (PULSE_WIDTH > 2450) // over 2.2mS? as check max. pulse width.
			{
			// keep previous control data as noise
			}	
		else 
			{
			if ( CONTROL_DATA_0 == PULSE_WIDTH/16 )
				{
				CONTROL_DATA = CONTROL_DATA_0;	// W-check OK.				
				}
			else
				{
				CONTROL_DATA_0 = PULSE_WIDTH/16; // re-check as W-check
				}
			//
			if ( CONTROL_DATA < THROTTLE_OFF ) //  min. pulse width check. The resolution is 490uS.
				{
				CONTROL_DATA = 1;	//ALMOST_ZERO
				PROPO_PULSE_IN_COUNTER ++;
				}
			else // Normal pulse
				{          	
				if ( CONTROL_DATA >= CONTROL_DATA_MAX )
					{
					CONTROL_DATA = CONTROL_DATA_MAX; // max power limiter
					}
				}
			}
		}
	//
	else 
		{	// Rise edge
   		PCA0_CAP_L = PCA0CPL0;	// Get front edge time
		PCA0_CAP_H = PCA0CPH0;
		PCA0CPM0 = 0x11;	//#00010001b	; PCA0 wll be captured by negative edge on P0.6 first and generates the interrupt.
		}
}
//-----------------------------------------------------------------------------
void T1_ISR (void) interrupt 3	// T1 interrupt
{
	TF1 = 0;	// clear int flag.
	if ( T1_Op_Flag == 1 )
		{
		T1_PWM_OFF_DUTY (); // Timer-1 PWM zero cross control
		T1_Op_Flag = 0;
		//P2 = 0;  // Monitor ONLY
		}
	//
	else
		{
		T1_PWM_ON_DUTY ();
		T1_Op_Flag = 1;
		//P2 = 1; // Monitor ONLY
		}	
}	 
//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------