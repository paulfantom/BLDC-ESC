// Date: Sep 24th, 2011
// Author: Takao
// This is JETI burned board as for "Arduino ATMega328 chip" with IR2103 MOSFET driver SKETCH program with all
// N-ch MOSFET diver.
// Copyright (C) 2011 Takao.
// Program Description:
// This program is for xxxx spin77 board R/C brushless-sensorless motor control.
// 16mS watchdog version.
//
// 1) A/D (is used for power voltage check only) is not used for spinning.
// 2) Added minimum propo offset calculation.
// 3) Using the minimum inductive kick finding start method as minimum power start.
// 4) No Audino timer mode to avoid mysterious interrupt while hi-speed running. 
//
// How To Test:
// 1) Download code to Arduino board(115kbps UART).
// 2) Use 12V, 20A Power supply to protect short circuit condition for debugging.
// 3) CONNECT Hyperion ZS3020-08 MOTOR + APC 9x6 prop. R/C Rx is needed.
// 4) Run this program. if spin speed is veryfast. Use 20MHz Arduino. The 16MHz sys. clock is not enought.
// Tool chain:  Arduino all free softwares
// Command Line:   None
// Release 0.1a
//-------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
// Propo-input data parameters 
#define Control_Min 950	// 950uS. MIN_OFFSET to keep about 15uS min. PWM ON spin power
#define Control_Max 2300  // 2300uS. Max Propo-in pulse width
#define Throttle_Off 12	// CONTROL_DATA
#define Stick_Hys 4	// Stick CONTROL_DATA 
#define Throttle_Offset_Drift 2	// CONTROL_DATA 
#define Minimum_Spin_Power Throttle_Off+Throttle_Offset_Drift	// CONTROL_DATA 
#define PWM_Frame_1kHz 1000	// 1kHz as 1mS PWM sampling timing
#define PWM_Frame_2kHz 500	// 2kHz sampling 
#define PWM_Frame_4kHz 250	// 4kHz sampling
#define PWM_Frame_8kHz 125	// 8kHz sampling 
#define PWM_Frame_13kHz 80	// 13kHz sampling
#define PWM_Frame_16kHz 64	// 16kHz sampling 
#define Start_PWM_ON_time 1
#define Start_Drive_State_On_Time 10	 
#define Start_Power_Up_Retry_Counts 150	// start power-up try limit
#define Minimum_One_Spin_Time 50	// re-start spinning or halt decision timing value
#define Init_PACK 4
#define Start_Power_ON_Time (PWM_Frame_8kHz>> 8) // 1/32 ON duty
#define Stick_Responce_Timing 40000 // 09/19 10mS step timing with 500nS T1 clock
//#define Zero_Cross_In_Start_Spin_Count 100
#define Active_Stop_Power_ON_Time 30;
#define Motor_L 5 // ******* as Minimum_IK_Meauring_PWM_Timing(increse automatically the measuring ON time value until finding the mini. I.K.
#define Restart_Free_Spin_Time_Count_Setting 150
//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
boolean OVER_ZERO_CROSS;
boolean TEMP_OVER_ZERO_CROSS;
boolean RE_START = 0;
boolean SPINNING;
boolean ZC_PWM_FLAG = 0;
boolean TICK_TACK_SOUND_FLAG;
boolean FISRT_ZC_PWM_FLAG;
boolean END_OF_IK_TIMING;
boolean PWM_ON_LOOP_FLAG;
boolean RE_START_FLAG;
boolean PWM_OFF_EDGE_OVER_ZERO_CROSS_FLAG;
boolean Spin_Free_Routine_TEST_FLAG_COUNTER;
boolean FULL_POWER_HYS;
boolean T2_STATE_CHANGE_CHECK_FLAG;
byte SOUND_WAVE_COUNTER;	// Used in ON_Sound ()
byte FREE_SPIN_CODE;
//byte ACSR_ORIG = ACSR;
byte START_drive_CODE;
byte IK_PWM_ON_PWR; // 6/25 Used in Start_PWM_On() for drive power ON duty timing(70uS). need adj. minimum spin power
byte START_SPIN_COUNTER;
byte LOOP;
byte LOOP1;
byte LOOP2;
byte ADD_STARTING_PWR;
byte START_drive_STATE;
byte TEMP = 0;
byte SPIN_CODE_BITS = 0;
byte TEMP_boolean; // Auduino BUG?
byte CONTROL_DATA_RAW;
byte CONTROL_DATA;
byte LINER_ON_DUTY = Throttle_Off;
byte ON_DUTY;
byte Rx_PULSE_IN_STATUS = 0;	// boolean is not affect in the interrput routine
byte MV_POINTER = 0;	// as Moving average pointer
byte ON_TIME;
byte OFF_TIME;

byte START_ON_TIME;
byte PACK = Init_PACK;
byte RX_CNT_DATA [8]; // Moving average of Rx data storage area

byte TEMP_START_drive_STATE;
byte NO_MISSING_COUNTER;
byte MINIMUM_INDUCTIVE_KICK_STATE;

byte OSC_COUNTER;
//byte TEMP1_ACSR;
//byte TEMP2_ACSR;
byte PWM_FRAME;
byte FIRST_CATCH_CODE;
byte SECOND_CATCH_CODE;
byte ERROR_CODE_52_COUNTER;
byte TEMP_NOISE_STATE[2];
byte TEMP_START_SPIN_COUNTER[2];
byte POWER_UP_TIME_COUNTER = 0;
byte SECOND_INDUCTIVE_KICK_STATE;
//byte * data xread;	// Reverse Exp. Table read pointer addressing

unsigned int SOUND_ON_DUTY_TIMING;	// Used in ON_Sound ()
unsigned int SOUND_OFF_DUTY_TIMING;	// Used in Sound gen.
unsigned int DELAY_uS_TIME;
unsigned int START_PWM_ON_LOOP2;
unsigned int FIND_BUZZER_COUNTER;
unsigned int COUNTS = 0;
unsigned int PULSE_WIDTH;
unsigned int PROPO_PULSE_IN_COUNTER = 0;
unsigned int Rx_SIGNAL_DETECT_SPIN_COUNTER;
unsigned int ZC_SPIN_COUNTER = 0;
unsigned int MV_TOTAL = 0;	// Moving average Total for the control input pulse width noise filter
unsigned int TIME1;
unsigned int TIME2;
unsigned int NEXT_POWER_UP_DOWN_TIME;
unsigned int START_TCNT1;
unsigned int ZC_TIME = -1;
unsigned int FREE_SPIN_DRIVE_TIME;
unsigned int TEMP_TNT1;
//unsigned int TEMP_IK; // for big L motor
byte TEMP_IK; // 09/18 for small L motor
//unsigned int INDUCTIVE_KICK_TIME_LENGTH; // for big L motor
byte TEMP_IK_TIMING; //  09/18 for small L motor
//unsigned int TEMP_IK_TIMING;
byte INDUCTIVE_KICK_TIME_LENGTH; //  09/18 for small L motor
//unsigned int EACH_INDUCTIVE_KICK_TIME_LENGTH [7]; // for big L motor
byte EACH_INDUCTIVE_KICK_TIME_LENGTH [7]; // 09/18 for small L motor
unsigned int RESTART_FREE_SPIN_TIME_COUNT;
unsigned int ZC_MISSING_COUNTER_A;

#include <avr/wdt.h>  // watchdog timer header
int ledPin = 13;                 // LED connected to digital pin 13
void setup()
{
        Spin_Free ();
        //       pinMode(D5_digital_monitor, OUTPUT);  // sets the digital pin as output  
        int Propo_in = 2;  // digital propo interrupt input-pin PD2
        int COMP_Hys_Pin = 8; // PB0
        int WL_N = 9; // PB1 
        int WH_P = 14; // PB4
        int VH_P = 3; // PD3
        int VL_N = 4; // PD4
        int UL_N = 5; // PD5
        int UH_P = 7; // PD7
        pinMode(Propo_in, INPUT);  // sets the pin as input
        attachInterrupt(0, Propo_Pulse_Width_Counting_L, FALLING);  // Propo reversed input signal pin2
        Serial.begin(115200);
        //DDRD = 0x28;  // D5&D3 push-pull output
        DDRD = 0xb8;  // 1011 1xxx PortD7,5,4,3 push-pull output
        DDRB = 0xfb;  // PORTB PB2 is the summing junction DC level down output at measuring I.K length. athers are used for push-pull output on portB0,1,3,4,5,6,7 
        //
        TCCR2A = 0; // simple timer for frame time easy adusting
        TIMSK0 = 0;        // no T0 overflow interrupt
        TCCR0B = 2;        // T0 runs 1/8 clock(2MHz)
        TIMSK1 = 0;        // T1, no overflow interrupt
        TCCR1A = 0;        // T1, normal operation simple count-up timer mode
        TCCR1B = 2;        // T1 runs 1/8 clock(2MHz)
        TCCR2B = 0;        // stop T2 clock for PWM
        //        pinMode(ledPin, OUTPUT);      // sets the digital pin as output
        PORTB &= 0xfe; // scope monitor "L"
}
//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void loop()
{
/*
        // OUTPUT wave form cheking TEST ONLY
         while(1)
         {
         for (START_drive_STATE=1; START_drive_STATE <= 6; START_drive_STATE ++)
         //START_drive_STATE = 1;
         			{
               	                Serial.println ("START_drive_STATE");
                                Serial.println (START_drive_STATE,DEC);		
                                Braking ();
         			Delay_1mS ();
         			for (SOUND_OFF_DUTY_TIMING= 1; SOUND_OFF_DUTY_TIMING<=2500; SOUND_OFF_DUTY_TIMING ++)	// monitor only
         				{
         PORTB |= 0x01; //monitor only		
         				Step_Start ();
                                        PORTB |= 0x01; // PB0.0 is monitor
         				DELAY_uS_TIME = 50;
                                        Delay_Microseconds ();	// switching delay
         PORTB &= 0xfe; //monitor only
         				Spin_Free ();
         				DELAY_uS_TIME = 50;
                                        Delay_Microseconds ();	// switching delay
              				Spin_Free_N ();
         				Delay_1mS ();
         				Delay_1mS ();
         				}
         			}
         }
         //OUTPUT wave form cheking TEST ONLY
*/
        Watchdog_Setting ();        // 16mS interval       
        PWR_On_Sound ();

        while ((PROPO_PULSE_IN_COUNTER< 20) || (CONTROL_DATA > Throttle_Off))
        {
                Routine_Work_Loop ();
                /*  Serial.print ("PULSE_WIDTH:");
                 Serial.println (PULSE_WIDTH,DEC);
                 Serial.print ("PROPO_PULSE_IN_COUNTER:");
                 Serial.println (PROPO_PULSE_IN_COUNTER,DEC);*/
                //  Serial.print ("CONTROL_DATA:");
                //  Serial.println (CONTROL_DATA,DEC);

                if (CONTROL_DATA > Throttle_Off)
                {
                        Serial.println ("Stick DOWN!");
                        for (COUNTS=0; COUNTS<50000; COUNTS++)
                        {
                                Routine_Work_Loop ();
                        }                         
                        TICK_TACK_SOUND_FLAG = 1;
                        Chirp_Down_Sound (); // TickTack sound before "stick down"
                } 
                TICK_TACK_SOUND_FLAG = 0;
                //     LINER_ON_DUTY = Minimum_Spin_Power;        // halt motor at the power ON.
        }
        Set_Analog_Comparator ();
        Go_Sound ();
        //;--------------
        //STEPPING_START:
        //;--------------
        while (1)
        {
                Routine_Work_Loop ();
                FIND_BUZZER_COUNTER = 0;
                Spin_Free();	// for Check_Free_Spinning ();
                //  Serial.print("CONTROL_DATA:");
                //  Serial.println(CONTROL_DATA,DEC);
                COUNTS = 0; 
                while ( ON_DUTY < Minimum_Spin_Power + Stick_Hys )	//Wait until stick ON with hys.
                {
                        Routine_Work_Loop ();
                        Delay_1mS ();  // 1mS timer for making one min. in this loop
                        COUNTS ++;
                        if ( COUNTS >= 100 )	// 100mS Count
                        {
                                COUNTS = 0;
                                FIND_BUZZER_COUNTER ++ ;
                                Serial.println (FIND_BUZZER_COUNTER,DEC); 
                                Serial.print ("PULSE_WIDTH:");
                                Serial.println (PULSE_WIDTH,DEC);
                                Serial.print("CONTROL_DATA:");
                                Serial.println(CONTROL_DATA,DEC);
                                /*      Serial.print("ON_DUTY:");
                                 Serial.println(ON_DUTY,DEC);
                                 Serial.print ("TCNT1_RUNNING = ");
                                 Serial.println (TCNT1,HEX);*/
                        }
                        //
                        if ( FIND_BUZZER_COUNTER >= 600 )	// about 1min. first start sound.
                        {
                                FIND_BUZZER_COUNTER = 400;	// every 20 sec. sounds
                                Chirp_Up_Sound ();
                                Chirp_Down_Sound ();
                        }
                        //  Serial.println (CONTROL_DATA,DEC);  
                }
                Spin_Free ();
                for (COUNTS=0; COUNTS<20; COUNTS++)// wait the baby spin(the sounds drives spin of the shaft)
                {
                        Routine_Work_Loop ();
                } 
                //
                //   Serial.println ("Spinning Checking.");
                NEXT_POWER_UP_DOWN_TIME = 0;
                Check_Free_Spinning ();
                //  FREE_SPIN_CODE=0; // test only
                //  PORTB |= 0x01;            
                if (FREE_SPIN_CODE == 0 )      //
                {       
                        RE_START_FLAG = 1;
                        TIMSK2 = 0; // disable T2 overflow interrupt
                        TCCR2B = 0; // stop T2
                        TIFR2 = 0x07; // clear interruprt flags by writing "1" for each flags
                        Brake_Time ();
                        Force_Pause_Start_State_Finder ();        // Active braking at very slow spin
                        Serial.println ("SPINNING == 0");
                        SPINNING = 0;
                        FULL_POWER_HYS = 0;
                        ADD_STARTING_PWR = Start_Power_ON_Time;
                        //Serial.print("ON_DUTY: ");
                        //Serial.println (ON_DUTY,DEC);
                        SREG |= 0x80;  //Global interrupt enable
                        PWM_FRAME = PWM_Frame_4kHz; // start PWM frame.
                        //  Serial.println ("Starter ON");
                        Starter ();
                        TIMSK2 = 0; // disable T2 interrupt
                        TCCR2B = 0; // stop T2 clock
                }
                Routine_Work_Loop ();
                
//PORTB &= 0xfe; // PB0 is scope monitor
                LINER_ON_DUTY = Minimum_Spin_Power;	// set minimum start power to reduce the starting battery current
                ON_DUTY = LINER_ON_DUTY;
                Rx_SIGNAL_DETECT_SPIN_COUNTER = 0; // reduse PWM duty at restart
                ZC_SPIN_COUNTER = 0;
                FISRT_ZC_PWM_FLAG = 1;
                //-----------------
                // ZeroCross spin  
                //-----------------
                while ( ON_DUTY >= Minimum_Spin_Power ) // Wait untill throttle stick off
                        {
                        //
                        ZC_SPIN_COUNTER ++;
                        if ( ZC_SPIN_COUNTER > 50) // 09/19 reduce the ZC start shock
                                {
                                ZC_SPIN_COUNTER = 0;       
                                if ( PWM_FRAME >= PWM_Frame_16kHz) // start PWM frame.
                                        {
                                        PWM_FRAME --;
                                        }
                                }        
                        //
                        PWM_ON_LOOP_FLAG = 0;
                        END_OF_IK_TIMING = 0;
                        if ( FISRT_ZC_PWM_FLAG == 1)
                                {
                                FISRT_ZC_PWM_FLAG = 0;
                                LINER_ON_DUTY = Minimum_Spin_Power;
                                Routine_Work_Loop ();
                                ZC_PWM_FLAG = 0;
                                END_OF_IK_TIMING = 1;        // the first spin does not have the inductive kick
                                TCNT2 = ~OFF_TIME; // The first PWM ON small pulse soon( without ZC_Detector ())
                                TIFR2 = 0x07; // clear interrupt flags by writing "1" for each flags
                                GTCCR &= 0xfd;  // clear T2 prescaler
                                TIMSK2 = 1; // enable PWM T2 interrupt
                                TCCR2B = 2; // enable T2 timer run with 1/8 sys. clk
                                }
                        ZC_full_power_control ();
                }
                //  Serial.println ("ZC_spin end");
                TIMSK2 = 0; // disable T2 interrupt
                TCCR2B = 0; // stop T2 clock
                Spin_Free (); // no longer Spin_Free_N () for the spinning check
                Routine_Work_Loop ();
                Delay_1mS ();
        }
}
//--------------------------------------------------
void ZC_full_power_control (void) //z
{
        #define No_ZC_Signal_Missing_Time_Count 450
	unsigned int temp;
	unsigned int zc_ignor_time;
        unsigned int temp_tcnt1;
        unsigned int temp_tcnt1_A;
        unsigned int temp_tcnt1_B;
    //    byte cnt2;
        byte state_change_flag = 0; // 09/19
        
	ZC_MISSING_COUNTER_A = 0; // to protect over current of MOSFETs
	if ( RE_START_FLAG == 1 ) // the first ZC control just after Start_Spin() is too much timing advance
		{
//PORTB |= 0x01; //monitor only;	// monitor only	
        	T2_STATE_CHANGE_CHECK_FLAG = 1;
                RE_START_FLAG = 0;
		}
	else
		{
//PORTB |= 0x01; //monitor only;	// monitor only		
//PORTB &= 0xfe; // PB0 is scope monitor
                Set_to_next_drive ();
                state_change_flag = 1;
                }
	//
        SREG &= 0x7f;  //Global interrupt disable;
        //TCCR2B = 0;
        if ( ZC_PWM_FLAG == 1 )	// drive state changes in PWM ON( need changing to full power control) 
		{
		Step_Start ();	//unsync. drive state change		
		}
        //TCCR2B = 2;	// T2 runs with 1/8 sys. clk
        SREG |= 0x80;  //Global interrupt enable
        temp_tcnt1_A = TCNT1;
//----------------------------------------------------------------------------------------------
	if ((ON_DUTY >= 64 ) && (FULL_POWER_HYS == 0)) // 09/23 full throttle?
		{
                        FULL_POWER_HYS = 1; // 09/19                
                        TCCR2B = 0; // Disable Timer2 interrupt for full power
	        	ZC_PWM_FLAG = 1;
                        T2_STATE_CHANGE_CHECK_FLAG = 1; // 09/22

                 }
	else 
		{
                        {
                        if (FULL_POWER_HYS == 1)  // 09/19
                                {
		                if (ON_DUTY < 62 )
                                        {
                                        FULL_POWER_HYS = 0;
                                        TCCR2B = 2;	// T2 runs with 1/8 sys. clk
                                        }
                                }        
                        else
                                {
                                TCCR2B = 2;	// T2 runs with 1/8 sys. clk
                                }
                        }
                }
//----------------------------------------------------------------------------------------------
        while ((TCNT1 - temp_tcnt1_A) < (PWM_Frame_16kHz)); // 09/24 inducktive kick ignor timing for high speed motor
PORTB |= 0x01; //monitor only;	// monitor only
        Set_comparator_inputs_for_ZC_over_detect ();	
        OVER_ZERO_CROSS = 0;	// Init. the flag.
        while ( OVER_ZERO_CROSS == 0 )	// while loop waiting for ZC point
                {
                if (ZC_PWM_FLAG == 1)
                        {
//PORTB |= 0x01; //monitor only;	// monitor only
                         Zero_Cross_Over_Check ();
                        /* CPU is too slow
                          if ((OVER_ZERO_CROSS ==1)&&(ZC_PWM_FLAG == 1)) // 09/22 w-check
                                 {
                                 DELAY_uS_TIME = 1; // 09/22 ringing delay
                                 Delay_Microseconds (); 
                                 Zero_Cross_Over_Check ();
                                 }
                         */
                         if ((ZC_PWM_FLAG == 0) || (T2_STATE_CHANGE_CHECK_FLAG == 0))
                                {
                                if (state_change_flag == 1) // 09/19
                                        {
                                        state_change_flag = 0; // 09/19
                                        OVER_ZERO_CROSS = 0;
                                        }
                                else        
                                        {
                                        OVER_ZERO_CROSS = PWM_OFF_EDGE_OVER_ZERO_CROSS_FLAG; // the latest compartor result as falling edge data is in the T2 PWM interrupt routine
                                        }
                                }
                        }
                else
                        {
                        OVER_ZERO_CROSS = 0;
                        }       
                }
PORTB &= 0xfe; // PB0 is scope monitor
        /* CPU speed in not enough
                ZC_MISSING_COUNTER_A ++;
				{
				if (ZC_MISSING_COUNTER_A > No_ZC_Signal_Missing_Time_Count) // 08/14 avoid burning ESC
					{
                                        TCCR2B = 0; // stop T2 clock
                                        TIMSK2 = 0;
                                        Spin_Free ();
                                	LINER_ON_DUTY = 0; // 09/13
	                                ON_DUTY = 0; // 09/13;
					}
				return; // 09/13  
                                }        
        */
        temp_tcnt1_B = TCNT1;
        temp = temp_tcnt1_B - temp_tcnt1_A; // T1 2MHz clock counts the time until Z.C.. 
        //
        ZC_TIME = (temp>>1);        // 25% timing advance. 
                do      
                { 
                        Rx_Signal_Lose_Off_Detector ();
                        Routine_Work_Loop (); // takes 15uS
                }        
                while ((TCNT1 - temp_tcnt1_B) < ZC_TIME); // 25% timing advance.        
}
//--------------------------------------------------------------------------------------------
// Sub Routines
//--------------------------------------------------------------------------------------------
void Delay_1mS ()
{
        START_TCNT1 = TCNT1;
        while ( TCNT1- START_TCNT1 < 2000); // 500nS steps
}
//
void Delay_Microseconds ()
{
        START_TCNT1 = TCNT1;
        while ( TCNT1- START_TCNT1 < (DELAY_uS_TIME<<1)); // T1 with 1/8 clock(2MHz,500nS) 
}
//
void ZC_Halt_Check ()
{
        unsigned int temp_tcnt1;
        boolean temp_zc_pwm_flag;
        temp_tcnt1 = TCNT1;
        temp_zc_pwm_flag =  ZC_PWM_FLAG;
        PORTD &= 0xf7;        // digitalWrite (D3_digital_monitor,0)
        //  Braking ();
        Spin_Free ();
        TCCR2B = 0; // stop T2 clock
        TIMSK2 = 0;
        Serial.print ("ON_TIME: ");
        Serial.println (ON_TIME,DEC); 
        Serial.println ("START_drive_STATE");
        Serial.println (START_drive_STATE,DEC);
        Serial.print ("FIRST_CATCH_CODE:");
        Serial.println (FIRST_CATCH_CODE,DEC);
        Serial.print ("SECOND_CATCH_CODE:");
        Serial.println (SECOND_CATCH_CODE,DEC);
        Serial.print ("~TCNT2: ");
        //  Serial.println (0xff-temp_tcnt1_B,DEC);
        //  Delay_1mS ();
        //  Serial.print ("ON_TIME: ");
        //  Serial.println (ON_TIME,DEC);
        Serial.print ("ZC_PWM_FLAG: ");
        Serial.println (temp_zc_pwm_flag,HEX);       
        //  Serial.print ("ZERO_CROSS_PWM_COUNTER: ");
        //  Serial.println (ZERO_CROSS_PWM_COUNTER,DEC);
        Serial.print ("OVER_ZERO_CROSS: ");
        Serial.println (OVER_ZERO_CROSS,HEX); 
        //        Serial.print ("PREV_OVER_ZC_FLAG: ");
        //        Serial.println (PREV_OVER_ZC_FLAG,HEX);
        ON_DUTY = Minimum_Spin_Power -1;
        for (COUNTS=0; COUNTS<3000; COUNTS++)
        {
                Delay_1mS ();
        }
        while (1);
}
//
void Force_Pause_Start_State_Finder ()	// Adjust the best starting phase by the slowest spin
{
        //  Serial.println ("void Force_Pause_Start_State_Finder ()");
        if ( START_drive_STATE > 0 ) 
        {
                for (COUNTS=0; COUNTS<250; COUNTS++)	// <<<<<<<<< Need adj.
                {
                        Spin_Free_N (); // charge up high side driver's cap.
                        DELAY_uS_TIME = OFF_TIME;
                        Delay_Microseconds ();
                        //
                        START_ON_TIME = Active_Stop_Power_ON_Time;
                        Step_Start ();                                
                        DELAY_uS_TIME = START_ON_TIME;
                        Delay_Microseconds ();
                        //
                        Spin_Free_N ();
                        Routine_Work_Loop ();
                }
        }
}
//
void Watchdog_Setting ()
{  // WDTON = 1 by the fuse setting.
        WDTCSR = 
                (0<<WDIF) | // Bit 7 ? WDIF: Watchdog Interrupt Flag
        (0<<WDIE) | // Bit 6 ? WDIE: Watchdog Interrupt Enable
        (0<< WDP3) | // Bit 5 ? WDP3: Watchdog diverder bit3
        (1<<WDCE) | // Bit 4 ? WDCE: Watchdog Change Enable
        (1<<WDE) | // Bit 3 ? WDE: Watchdog System Reset Enable
        (0<<WDP2) | (0<<WDP1) | (0<<WDP0); //bit2,1&0 ? WDP: Watchdog diverder as minimum 2mS.
}
//--------------------------------------
// Motor drive each state control 
//--------------------------------------
void State1_On ()	// U=H, V=L, W open	
{
        Spin_Free ();
        PORTD |= 0xa0;   // 1010 0000 UH(D7,PD7)="H",  UL-(D5,PD5)="H"
        PORTD &= 0xe7;   // 1110 0111 VL-(D4,PD4) ="L", VH(D3,PD3)="L"
}
//
void State2_On (void)	// U=H, V open, W=L
{				
        Spin_Free ();
        PORTB &= 0xe7;	// 1110 0111 WH(D12,PB4)="L",WL-(D11,PB3)="L"
        PORTD |= 0xa0;  // 1010 0000 UH(D7,PD7)="H",  UL-(D5,PD5)="H"       
}
//
void State3_On (void)	// U open, V=H, W=L
{				
        Spin_Free ();
        PORTB &= 0xe7;	// 1110 0111 WH(D12,PB4)="L",WL-(D11,PB3)="L"
        PORTD |= 0x18;  // 0001 1000  VL-(D4,PD4) ="H", VH(D3,PD3)="H"
}
//	
void State4_On (void)	// U=L, V=H, W open
{				
        Spin_Free ();
        PORTD &= 0x5f;   // 0101 1111 UH(D7,PD7)="L",  UL-(D5,PD5)="L"
        PORTD |= 0x18;   // 0001 1000 VL-(D4,PD4) ="H", VH(D3,PD3)="H"
}
//
void State5_On (void)	// U=L, V open, W=H
{				
        Spin_Free ();
        PORTB |= 0x18;	// 0001 1000 WH(D12,PB4)="H",WL-(D11,PB3)="H"
        PORTD &= 0x5f;   // 0101 1111 UH(D7,PD7)="L",  UL-(D5,PD5)="L"
}
//
void State6_On (void)	// U open, V=L, W=H
{				
        Spin_Free ();
        PORTB |= 0x18;	// 0001 1000 WH(D12,PB4)="H",WL-(D11,PB3)="H"
        PORTD &= 0xe7;   // 1110 0111 VL-(D4,PD4) ="L", VH(D3,PD3)="L"
}
//
void Spin_Free () // All output MOSFETs off
{
        PORTB &= 0xef;
        PORTB |= 0x08;	// xx00 1xxx WH(D12,PB4)="L",WL-(D11,PB3)="H"
        Nop (); // Wx OFF switching delay
        PORTD = 0x30  ;   // 0011 0000 UH(D7,PD7)="L",  UL-(D5,PD5)="H", VL-(D4,PD4) ="H", VH(D3,PD3)="L
        wdt_reset ();	// Watch dog timer off.
}
//
void Nop ()
{
}
//
void Braking ()	
{				
        Spin_Free ();
        DELAY_uS_TIME = 2; //2uS
        Delay_Microseconds (); // settling 2uS delay
        PORTB &= 0xe7;	// 1110 0111 WH(D12,PB4)="L",WL-(D11,PB3)="L"
        PORTD &= 0x47;   // 0100 0111 UH(D7,PD7)="L",  UL-(D5,PD5)="L", VL-(D4,PD4) ="L", VH(D3,PD3)="L
}
//
void Spin_Free_N ()
{
        Spin_Free (); // All MOSFTEs off
        switch (START_drive_STATE) 
        {
        case 1:
        case 6:
                {
                        //U=3ST, V=0, W=3ST
                        PORTD &= 0xe7;   // 1110 0111 VL-(D4,PD4) ="L", VH(D3,PD3)="L"
                        break;
                }
        case 2:
        case 3:
                {
                        // U=3ST, V=3ST, W=0	
                        PORTB &= 0xe7;	// 1110 0111 WH(D12,PB4)="L",WL-(D11,PB3)="L"
                        break;
                }

        case 4:
        case 5:
                {
                        // U=0, V=3ST, W=3st
                        PORTD &= 0x5f;   // 0101 1111 UH(D7,PD7)="L",  UL-(D5,PD5)="L"
                }
        }
}
//
void Free_Spin_Decorder ()
{
        SPIN_CODE_BITS = 0; // init.
        ADMUX = 2; 	// CP in-: AD2(Uin), CP in+: AN0(ZCV)
        Start_Sensing_Zero_Cross_Over_Check ();
        if ( OVER_ZERO_CROSS == 1 ) // START_drive_STATE 6123
        {
                SPIN_CODE_BITS |= 0x01;	// #00000001b
        }
        else
        {
                SPIN_CODE_BITS &= 0xfe;	// #11111110b
        }
        //
        ADMUX = 3;  // CP in-: AD3(Vin), CP in+: AN0(ZCV)
        Start_Sensing_Zero_Cross_Over_Check ();
        if ( OVER_ZERO_CROSS == 1 ) // START_drive_STATE 23345
        {
                SPIN_CODE_BITS |= 0x02;	
        }
        else
        {
                SPIN_CODE_BITS &= 0xfb;	// #11111101b
        }
        //
        ADMUX = 4;  // CP in-: AD4(Win), CP in+: AN0(ZCV)
        Start_Sensing_Zero_Cross_Over_Check ();
        if ( OVER_ZERO_CROSS == 1 ) // START_drive_STATE 4561
        {
                SPIN_CODE_BITS |= 0x04;
        }
        else
        {
                SPIN_CODE_BITS &= 0xfd;	// #11111011b
        }
        //
        FREE_SPIN_CODE = 0;
        switch ( SPIN_CODE_BITS )	// get zero cross point by comparator. +- input are reverse connected
        {
        case 1: // U=1, V=0, W=0 free spin check
                {
                        FREE_SPIN_CODE = 0x12;
                        break;
                }
        case 2: //  U=0, V=1, W=0 free spin check
                {
                        FREE_SPIN_CODE = 0x34;
                        break;
                }
        case 3: //  U=1, V=1, W=0 free spin check
                {
                        FREE_SPIN_CODE = 0x23;
                        break;
                }
        case 4: // U=0, V=0, W=1 free spin check
                {
                        FREE_SPIN_CODE = 0x56;
                        break;
                }
        case 5: //  U=1, V=0, W=1 free spin check
                {
                        FREE_SPIN_CODE = 0x61;
                        break;
                }
        case 6: //  U=0, V=1, W=1 free spin check
                {
                        FREE_SPIN_CODE = 0x45;
                }
        }                           
}
//
void Set_Analog_Comparator ()
{
        //Serial.println("Set_Analog_Comparator ()"); 
        ACSR = 
                (0<<ACD) | // Analog Comparator: Enabled. bit7
        (0<<ACBG) | // Analog Comparator Bandgap Select: AIN0 input as ZC ref.
        (0<<ACI) | // Analog Comparator Interrupt Flag: Clear Pending Interrupt. bit4
        (0<<ACIE) | // Analog Comparator Interrupt: disabled. bit3
        (0<<ACIC); // Analog Comparator Input Capture: Disabled.bit2
        ADCSRB |= 0x40;  // ACME bit = 1
        // 
        DIDR1 |= 0x03; // AIN0=D6,AIN1=D7 are for analog input ONLY
        ADCSRA &= 0x7f;  // Bit 7 ADEN: ADC Disable
}
//
void Start_Sensing_Zero_Cross_Over_Check ()
{
        DELAY_uS_TIME = 4; //4uS
        Delay_Microseconds (); // settling 4uS delay                
        //  
        if (ACSR & 0x20)	// the internal comparator's +-inputs are reversed
        {
                OVER_ZERO_CROSS = 0;
                //  PORTD |= 0x08;        // digitalWrite (D3_COMP_Hys_Pin,1);
                //  Serial.println (ACSR,HEX);
        }
        else
        {
                OVER_ZERO_CROSS = 1;
                //  PORTD &= 0xf7;        // digitalWrite (D3_COMP_Hys_Pin,0);
        }
}
//
void Starter ()// 6/25
{
        //#define start_spin_power 3
        //Serial.println ("void Starter ()");
        while (1)	// TEST ONLY. if the motor shaft is turned by Static_Start_State_Finder ()
        {
                START_SPIN_COUNTER = 0;
                Braking ();
                Get_minimum_Inductive_Kick_State (); // 9/15
                TEMP_START_drive_STATE = START_drive_STATE;
                Braking ();
                Get_minimum_Inductive_Kick_State (); // 9/15 w-check
                if (TEMP_START_drive_STATE == START_drive_STATE)
                         {
                         break;
                         }        
        }
        Serial.println ("///////////////////////////////////////////////////////////////////////");
        Serial.print ("START_drive_STATE= ");
        Serial.println (START_drive_STATE,DEC);
        Serial.println ("///////////////////////////////////////////////////////////////////////");
        START_SPIN_COUNTER = 0;
        ERROR_CODE_52_COUNTER = 0;
        START_ON_TIME = Start_Power_ON_Time;
        //ON_TIME = 50; // The first ON_TIME only 50uS               
        while (SPINNING == 0)
                {
                        //  Serial.print("Start_Spin:");
                        Start_Spin ();
                        Set_to_next_drive ();
                        Spin_Free_N ();
                        Routine_Work_Loop ();
                }
}
//
void Get_minimum_Inductive_Kick_State (void)
{
        IK_PWM_ON_PWR = Motor_L;
        MINIMUM_INDUCTIVE_KICK_STATE = 0;
        do
        {
                if ( IK_PWM_ON_PWR < 25 )	// 09/16 limiter value for the inductive kick measurement pulse minimum ON_DUTY to protect MOSFETs 
                        {
                        IK_PWM_ON_PWR ++;
                        }
                else
                        {
                        Spin_Free ();	// Could not find the start spin position
                        SREG &= 0x7f;  //Global interrupt disable
                        Serial.println ("///////////////////////////////////////////////////////////////////////");
                        for (START_drive_STATE= 1; START_drive_STATE<=6; START_drive_STATE ++)
                                {
                                wdt_reset ();	// Watchdog off
                                Serial.print ("The inductick time length at the state ");
                                Serial.print (START_drive_STATE,DEC);
                                Serial.print (" = ");
                                Serial.println (EACH_INDUCTIVE_KICK_TIME_LENGTH[START_drive_STATE],DEC);
                                //                
                                if (START_drive_STATE == 1)
                                        {
                                        TEMP_IK = EACH_INDUCTIVE_KICK_TIME_LENGTH[1];        //initial setting #1
                                        }
                                TEMP_IK_TIMING = EACH_INDUCTIVE_KICK_TIME_LENGTH[START_drive_STATE];	
                                if (TEMP_IK_TIMING <= TEMP_IK)      // 09/02 Get the minimum inducktive kick state include 1 digit tolerance
                                        {
                                        SECOND_INDUCTIVE_KICK_STATE = MINIMUM_INDUCTIVE_KICK_STATE; // 08/31
                                        TEMP_IK = EACH_INDUCTIVE_KICK_TIME_LENGTH[START_drive_STATE]; 
                                        MINIMUM_INDUCTIVE_KICK_STATE = START_drive_STATE;
                                        }
                                }
                        Serial.print ("The minimum inductick time legth at the state ");
                        Serial.print (MINIMUM_INDUCTIVE_KICK_STATE,DEC);
                        Serial.print (" = ");
                        Serial.println (EACH_INDUCTIVE_KICK_TIME_LENGTH[MINIMUM_INDUCTIVE_KICK_STATE],DEC);                                               
                        Serial.println ("///////////////////////////////////////////////////////////////////////");      
                        while (1)	// spin forever for debugging
                                {
                                wdt_reset ();  // Reset the watchdog timer
                                }
                        }
                Braking (); 	// avoid the resonance
                DELAY_uS_TIME = 100; 
                Delay_Microseconds ();  // Charge up high side driver
                wdt_reset ();  // Reset the watchdog timer 
                START_drive_STATE= 6;      // inertial state1 spin for low inductance coreless motor while the inductive kick mesuring.
                Start_Measure_IK_PWM_ON ();
                Routine_Work_Loop (); 
                for (START_drive_STATE= 1; START_drive_STATE<=6; START_drive_STATE ++)
                {
                        Start_Measure_IK_PWM_ON ();
                        Serial.print ("The inductick time legth [");
                        Serial.print (START_drive_STATE,DEC);
                        Serial.print ("] = ");
                        Serial.println (INDUCTIVE_KICK_TIME_LENGTH,DEC);
                        
                        Routine_Work_Loop (); 
                        if ( INDUCTIVE_KICK_TIME_LENGTH < 10 ) // 08/15 missing meaurement by mosfet switching delay?
                        {
                                MINIMUM_INDUCTIVE_KICK_STATE = 0;
                                break;
                        }
                        EACH_INDUCTIVE_KICK_TIME_LENGTH[START_drive_STATE] = INDUCTIVE_KICK_TIME_LENGTH; // for monitor only 
                        if (START_drive_STATE == 1)
                        {
                                TEMP_IK = EACH_INDUCTIVE_KICK_TIME_LENGTH[1];        //initial setting #1
                                MINIMUM_INDUCTIVE_KICK_STATE = 1;	// 09/10 just tempolary
                                SECOND_INDUCTIVE_KICK_STATE = 2;	// 09/18 just tempolary
                        }
                        if (INDUCTIVE_KICK_TIME_LENGTH <= TEMP_IK)      // 09/02 Get the minimum inducktive kick state include 1 digit tolerance
                        {
                                SECOND_INDUCTIVE_KICK_STATE = MINIMUM_INDUCTIVE_KICK_STATE; // 08/31
                                TEMP_IK = INDUCTIVE_KICK_TIME_LENGTH; 
                                MINIMUM_INDUCTIVE_KICK_STATE = START_drive_STATE;
                        }
                }
                // 09/15 include 1 digit torellance
                if (((EACH_INDUCTIVE_KICK_TIME_LENGTH[MINIMUM_INDUCTIVE_KICK_STATE]+3) <= EACH_INDUCTIVE_KICK_TIME_LENGTH[SECOND_INDUCTIVE_KICK_STATE])
                        || (EACH_INDUCTIVE_KICK_TIME_LENGTH[MINIMUM_INDUCTIVE_KICK_STATE] == EACH_INDUCTIVE_KICK_TIME_LENGTH[SECOND_INDUCTIVE_KICK_STATE]))
                {
                        START_drive_STATE = MINIMUM_INDUCTIVE_KICK_STATE;
                }
                else
                {
                        MINIMUM_INDUCTIVE_KICK_STATE = 0; // avoid START_drive_STATE = 7 in this for loop
                }
        }
        while (MINIMUM_INDUCTIVE_KICK_STATE == 0);
}
//
void Start_Measure_IK_PWM_ON () // 6/25
{
//        START_drive_STATE = 4;
//        while (1){
// PORTB &= 0xfe; // PB0 is scope monitor
        Routine_Work_Loop ();
        //Serial.print("Setting the comparator inputs for the drive state of ");
        //Serial.println("START_drive_STATE,DEC");
        Set_The_Comparator_to_Measure_Inductive_Kick_Length ();
        //Serial.println("Charging up all of the hi-side driver's power line capacitors by low-sides ON as braking.");
        //PORTB |= 0x01;        // scope trigger pin PB0.0;
        Braking ();	// avoid the resonance & set to hi-side drive
        Delay_1mS ();
        /*
        Serial.print ("The drive state : ");
        Serial.print (START_drive_STATE,DEC);
        Serial.println (" , mesuring the inductive kick length.");
        */
        SREG &= 0x7f;  //Global interrupt disable

        Routine_Work_Loop ();       
        Step_Start ();
        for (LOOP1=0; LOOP1<IK_PWM_ON_PWR; LOOP1++)  // IK measurinON time/ Need adj.
        {
                DELAY_uS_TIME = 6; // I.K. measuring resolution
                Delay_Microseconds ();
        }
        MCUCR |= 0x10; // 09/24 no pull-up R 
        PORTB &= 0xfb;  // 09/24 PB2=0, ZCV voltage level down to measure the inductive kick time length      
        DDRB |= 0x04; // 09/24 PB2 is now output
        
        Spin_Free ();
        DELAY_uS_TIME = 2; // 2uS MOSFET switching delay minimum
        Delay_Microseconds ();       
        
        while (!(ACSR & 0x20)); // 09/24 MOSFET switching speed canceller 
  PORTB |= 0x01; // PB0 is scope monitor

        INDUCTIVE_KICK_TIME_LENGTH = 0;
        while ((ACSR & 0x20) && (INDUCTIVE_KICK_TIME_LENGTH <= 0xff)) // 09/24 need more speed CPU to catch the the MOSFET ringing(for a while under ZC ref.)
        {
                INDUCTIVE_KICK_TIME_LENGTH++;
        }
  PORTB &= 0xfe; // PB0 is scope monitor
        SREG |= 0x80;  //Global interrupt enable 
        
        DDRB &= 0xfb; // 09/24 PB2 is input without pull up R
        PORTB |= 0x04;  // 09/24 PB2 = 1
        if ((INDUCTIVE_KICK_TIME_LENGTH>200) ||(INDUCTIVE_KICK_TIME_LENGTH < 3 ))   // Could not catch the inductve kick
        {
                PACK = START_drive_STATE;
                ON_Sound ();
                PACK = Init_PACK;
                Serial.print ("MINIMUM_INDUCTIVE_KICK_STATE = ");
                Serial.println (MINIMUM_INDUCTIVE_KICK_STATE,DEC);
                Serial.print ("SECOND_INDUCTIVE_KICK_STATE = ");
                Serial.println (SECOND_INDUCTIVE_KICK_STATE,DEC);
                Serial.print ("The IK_PWM_ON_PWR = ");
                Serial.println (IK_PWM_ON_PWR,DEC);
                Serial.print ("The inductick time legth = ");
                Serial.println (INDUCTIVE_KICK_TIME_LENGTH,DEC);
                Serial.print ("The bad START_drive_STATE = ");
                Serial.println (START_drive_STATE,DEC);
                for (COUNTS=0; COUNTS<50000; COUNTS++)
                {
                        Routine_Work_Loop ();
                }
                /*
                while (1)	// spin forever for debugging
                       {
                       wdt_reset ();  // Reset the watchdog timer
                       }
                */
                } 
}
//
void Start_Power_In () //x 6/25
{
        SREG &= 0x7f;  // disable Global interrupt
//        PORTB &= 0xfe;        // monitor
        //
        Spin_Free_N ();
        OFF_TIME = PWM_FRAME - START_ON_TIME;
        //
        DELAY_uS_TIME = OFF_TIME; 
        Delay_Microseconds ();
        Set_comparator_inputs_for_ZC_over_detect ();
//        PORTB |= 0x01;        // scope trigger pin PB0.0;  
        Step_Start (); // 
        DELAY_uS_TIME = START_ON_TIME; 
        Delay_Microseconds ();
        Start_Sensing_Zero_Cross_Over_Check (); // +8uS settling as ringing delay ON
        TEMP_OVER_ZERO_CROSS = OVER_ZERO_CROSS;
        Spin_Free_N (); // charge up high side driver's cap.        
        //  PORTB &= 0xfe;        // monitor
        SREG |= 0x80;  //Global interrupt enable
        START_PWM_ON_LOOP2 ++;
}  
//
void Spin_State_Check ()
{
        switch ( START_drive_STATE )
        {
        case 1:
                SPIN_CODE_BITS |= 0x01;
                break;
                //
        case 2:
                SPIN_CODE_BITS |= 0x02;	// #00000010b
                break;
                //
        case 3:
                SPIN_CODE_BITS |= 0x04;	// #00000100b
        }
}
//
void Set_The_Comparator_to_Measure_Inductive_Kick_Length ()
{
        switch ( START_drive_STATE )
        {
        case 1:
        case 2:
                {
                        ADMUX &= 0xf0;
                        ADMUX |= 0x02; 	// 09/24 CP in-: ADC2(Uin), CP in+: AN0(ZC ref)
                     //   ADMUX |= 0x03; 	// CP in-: ADC3(Vin), CP in+: AN0(ZC ref)
                        break;		
                }
                //
        case 3:
        case 4:
                {
                        ADMUX &= 0xf0;
                      //  ADMUX |= 0x04;	// 09/24 CP in-: ADC4(Win), CP in+: AN0(ZC ref)
                         ADMUX |= 0x03;	// CP in-: ADC2(Vin), CP in+: AN0(ZC ref)
                        break;			
                }
                //
        case 5:
        case 6:
                {
                        ADMUX &= 0xf0;
                      //  ADMUX |= 0x02;	// 09/2CP in-: ADC2(Uin), CP in+: AN0(ZC ref)
                        ADMUX |= 0x04;	// CP in-: ADC4(Win), CP in+: AN0(ZC ref)
                }
        }
}
//
void Set_comparator_inputs_for_ZC_over_detect ()
{
        switch ( START_drive_STATE )	// get zero cross point by comparator. +- input are reverse connected
        {
        case 1:
        case 4:
                {
                        ADMUX = 0x04;  // CP in-: AD4(Win), CP in+: AN0(ZCV)
                        break;
                }
        case 2:
        case 5:
                {
                        ADMUX = 0x03;  // CP in-: AD3(Vin), CP in+: AN0(ZCV)
                        break;
                }
        case 3:
        case 6:
                {
                        ADMUX = 0x02; 	// CP in-: AD2(Uin), CP in+: AN0(ZCV)
                }
        }
}
//
void Zero_Cross_Over_Check ()
{
        DELAY_uS_TIME = 1; // 09/22 ringing delay
        Delay_Microseconds ();       
        OVER_ZERO_CROSS = 0;
        switch ( START_drive_STATE )	// get zero cross point by comparator. +- input are reverse connected
        {
        case 1:	// Check the slope down
        case 3:
        case 5:
                {
                        if (ACSR & 0x20)   // get the comparator output bit
                        {
                                OVER_ZERO_CROSS = 1;
                        }
                        break;				
                }
        case 2:	// Check the slope up
        case 4:
        case 6:
                {
                        if (~ACSR & 0x20)  // get the comparator output bit	
                        {
                                OVER_ZERO_CROSS = 1;
                        }
                }
        }
}
//
void Start_Spin ()       
{
        Spin_Free_N ();
        START_PWM_ON_LOOP2 = 0;
        ZC_PWM_Timing_Control_Calc ();        // The map calc. takes 50uS at SPINNING == 0. !!!! could not use map calc.
        Routine_Work_Loop ();
        //************************************************************************        
        Routine_Work_Loop (); 
        switch (START_drive_STATE)
        {
        case 1:
        case 3:        
        case 5:
                {
                        TEMP_boolean = 0;
                        break;
                }
        case 2:
        case 4:       
        case 6:
                {
                        TEMP_boolean = 1;
                }
        }
        OVER_ZERO_CROSS = TEMP_boolean;
        //************************************************************************	 
        while (OVER_ZERO_CROSS == TEMP_boolean ) 
        {
                if (START_PWM_ON_LOOP2 > 1000)// Could not spin? Need adjust timing to avoid reverse resonance 
                {
                        START_SPIN_COUNTER = 0;
                        START_PWM_ON_LOOP2 = 0;
                        Serial.println ("Hard Start");
                        if (START_ON_TIME < PWM_FRAME>>3) // 09/17 1/8 ON duty max
                        {
                                START_ON_TIME ++;
                        }
                        return; // same as TEMP_boolean = !TEMP_boolean; & Set_to_next_drive ();
                }
                Start_Power_In ();
                //
                if ((START_PWM_ON_LOOP2 <= 3 ) && (OVER_ZERO_CROSS != TEMP_boolean))  // ERROR_CODE_BITS_5_OR_2 filter
                { 
                        START_drive_STATE -= 1;
                        if (START_drive_STATE == 0)
                        {
                                START_drive_STATE = 6;        // back to now drive state
                        }
                        TEMP_NOISE_STATE[ERROR_CODE_52_COUNTER]=START_drive_STATE;
                        TEMP_START_SPIN_COUNTER[ERROR_CODE_52_COUNTER]= START_SPIN_COUNTER;
                        ERROR_CODE_52_COUNTER ++;
                        START_PWM_ON_LOOP2 = 0;
                }
                if ((ERROR_CODE_52_COUNTER == 2) && (TEMP_START_SPIN_COUNTER[1]-TEMP_START_SPIN_COUNTER[0] == 1))        // continuas as short period noise?
                {
                        START_drive_STATE = TEMP_NOISE_STATE[0];
                        OVER_ZERO_CROSS = TEMP_boolean;
                        ERROR_CODE_52_COUNTER = 0;
                        Serial.println ("");
                        Serial.println ("ERROR_CODE_BITS_5_OR_2 is filtered");
                        START_SPIN_COUNTER = 0;
                }      
                else
                {
                        if (ERROR_CODE_52_COUNTER == 2)
                        {
                                ERROR_CODE_52_COUNTER = 0;
                        }
                }
        }
        //
        Routine_Work_Loop ();
        if ( LINER_ON_DUTY < Minimum_Spin_Power)
        {
                LINER_ON_DUTY = 0;
                ON_DUTY = LINER_ON_DUTY;
                SPINNING = 1;
                return;
        }
        //
        //START_ON_TIME = Start_Power_ON_Time;
        //  Serial.print ("F");
        //  Serial.print ("SPC: ");
        //  Serial.println (START_SPIN_COUNTER,DEC);
        //  Serial.print ("SPW: ");
        //  Serial.print (START_PWM_ON_LOOP2,DEC);
        //  Serial.print (", ");
        //  START_REVESE_SPIN_FLAG = 0;
        START_SPIN_COUNTER ++; 
        if ((START_SPIN_COUNTER > 70) && (ERROR_CODE_52_COUNTER <= 1) || (START_SPIN_COUNTER > 140))	//   Need adj. spin count enough?
        {
                if (START_PWM_ON_LOOP2 < 25)        // spin speed enough?
                {
                        Spin_Free ();
                        SPINNING = 1;	
                        //  PORTB |= 0x01;  
                        //  scope trigger pin PB0.0;
                        //  Serial.println("");
                        //  Serial.print ("START_PWM_ON_LOOP2: ");
                        //  Serial.println (START_PWM_ON_LOOP2,DEC);
                }
                else
                {
                        if (START_ON_TIME < PWM_FRAME>>3) // 1/8 ON duty max
                        {
                                START_SPIN_COUNTER -=5; // smooth speed-up
                                START_ON_TIME ++;
                        }
                }       
        }
}	
// Checking free spinning before stepping re-start drive
void Check_Free_Spinning () // 7/10
{
        unsigned int missing_code_counter = 0;  
        unsigned char temp;
        unsigned int temp_time_0;
        unsigned int temp_time_1;
        unsigned char n = 0;

        wdt_reset ();	// Watchdog off
        missing_code_counter = 0;
        Spin_Free ();
        //
        if ( RE_START_FLAG == 0 )
        {      
                RESTART_FREE_SPIN_TIME_COUNT = (Restart_Free_Spin_Time_Count_Setting>>1);	// 09/10 set the restart spin speed hys.
        }
        else
        {
                RESTART_FREE_SPIN_TIME_COUNT = Restart_Free_Spin_Time_Count_Setting;	// set the restart spin speed hys.
        }
        //
        do // 
        {
                missing_code_counter ++;
                if (missing_code_counter > 50)        // free spin check time limit
                {
                        FREE_SPIN_CODE = 0;
                        return;
                }        
                wdt_reset ();	// Watchdog off
                Free_Spin_Decorder ();
                TEMP = FREE_SPIN_CODE;
                DELAY_uS_TIME = 1; // 
                Delay_Microseconds ();
                Free_Spin_Decorder ();
                if ((TEMP == FREE_SPIN_CODE) && (FREE_SPIN_CODE != 0))
                {
                        n++;
                }
                else
                {
                        n=0;
                }
                //
                if (n > 10) // continus same FREE_SPIN_CODE?
                {
                        break;
                }
        }
        while ((FREE_SPIN_CODE == 0) || (TEMP == FREE_SPIN_CODE)); // get the first edge
        //
        n = 0;
        missing_code_counter = 0;
        do
        {
                missing_code_counter ++;
                if (missing_code_counter > RESTART_FREE_SPIN_TIME_COUNT) // free spin check time limit
                {

                        FREE_SPIN_CODE = 0;
                        return;
                }
                wdt_reset ();	// Watchdog off
                Free_Spin_Decorder ();
                temp = FREE_SPIN_CODE;
                DELAY_uS_TIME = 1; 
                Delay_Microseconds ();
                Free_Spin_Decorder (); // w-check
                if ((temp == FREE_SPIN_CODE) && (temp != TEMP) && (FREE_SPIN_CODE != 0))
                {
                        n++;
                }
                else
                {
                        n=0;
                }
                //
                if (n > 10) // continus same FREE_SPIN_CODE?
                {
                        break;
                }
        }
        while ((FREE_SPIN_CODE == 0) || (temp == FREE_SPIN_CODE) || (TEMP == FREE_SPIN_CODE)); // get the first ZC edge
        //	
        FIRST_CATCH_CODE = FREE_SPIN_CODE;
        /* Serial.print ("missing_code_counter = ");
         Serial.print (missing_code_counter,DEC); 
         Serial.print (", FREE_SPIN_CODE = ");
         Serial.println (FREE_SPIN_CODE,HEX);*/
        //
        missing_code_counter = 0;
        while (1) 
        {           
                n = 0;
                START_TCNT1 = TCNT1;
                do
                {
                        missing_code_counter ++;
                        if (missing_code_counter > RESTART_FREE_SPIN_TIME_COUNT)
                        {
                                FREE_SPIN_CODE = 0;
                                /* Serial.print ("missing_code_counter = ");
                                 Serial.print (missing_code_counter,DEC);
                                 Serial.print ("SECOND_CATCH_CODE = ");
                                 Serial.println (FREE_SPIN_CODE,HEX);*/
                                return;
                        }
                        wdt_reset ();	// Watchdog off
                        Free_Spin_Decorder ();
                        TEMP = FREE_SPIN_CODE;
                        DELAY_uS_TIME = 1; 
                        Delay_Microseconds ();
                        Free_Spin_Decorder ();
                        if ((TEMP == FREE_SPIN_CODE) && (FIRST_CATCH_CODE != FREE_SPIN_CODE) && (FREE_SPIN_CODE != 0))
                        {
                                n++;
                        } 
                        else
                        {
                                n=0;
                        }
                        //
                        if (n > 20) // continus same FREE_SPIN_CODE?
                        {
                                break;
                        }
                }
                while ((FREE_SPIN_CODE == 0) || (TEMP == FREE_SPIN_CODE) || (FIRST_CATCH_CODE == FREE_SPIN_CODE)); // get the second edge
                //
                FIRST_CATCH_CODE = FIRST_CATCH_CODE & 0x0f; 
                SECOND_CATCH_CODE = (FREE_SPIN_CODE & 0xf0)>>4; // shift to low nibble
                if (FIRST_CATCH_CODE == SECOND_CATCH_CODE)         // detected the drive state overlaped?
                {
                        FREE_SPIN_DRIVE_TIME = (TCNT1 - START_TCNT1)>>1; // without timing advance. 500nS resolution
                        while ((TCNT1 - START_TCNT1) < FREE_SPIN_DRIVE_TIME);        // wait next ZC start drive timing as phase adj.
                        /*  Serial.print ("missing_code_counter = ");
                         Serial.print (missing_code_counter,DEC);
                         Serial.print (",  FIRST_CATCH_CODEE = ");
                         Serial.print (FIRST_CATCH_CODE,HEX);
                         Serial.print (",  FREE_SPIN_CODE = ");
                         Serial.println (FREE_SPIN_CODE,HEX);*/
                        // Serial.print (",  ON_DUTY = ");
                        //  Serial.println (ON_DUTY,HEX);
                        START_drive_STATE = (FREE_SPIN_CODE & 0x0f);
                        return;
                }
                else
                {
                        /*   Serial.println ("ZC Starts error free spin code detection");
                         Serial.print ("FIRST_CATCH_CODE = ");
                         Serial.println (TEMP,HEX);
                         Serial.print ("SECOND_CATCH_CODE = ");
                         Serial.println (FREE_SPIN_CODE,HEX); */
                        FREE_SPIN_CODE = 0;
                        // return;
                        FIRST_CATCH_CODE = TEMP;
                }
        }       
}
//
void Rx_Signal_Lose_Off_Detector ()	// No control signal?
{
        Rx_SIGNAL_DETECT_SPIN_COUNTER ++;
        if ( Rx_SIGNAL_DETECT_SPIN_COUNTER >= 10000 )	// may be min. 700mS interval
        {
                if ( PROPO_PULSE_IN_COUNTER == 0 )
                {
                        if ( CONTROL_DATA > Minimum_Spin_Power)	//Need adj
                        {
                                CONTROL_DATA --;		// throttle down step by step 700mS slowly
                        }
                }
                else
                {
                        PROPO_PULSE_IN_COUNTER = 0;
                        Rx_SIGNAL_DETECT_SPIN_COUNTER = 5000;
                }
        }
}
//-----------------------------------------------------------------------------
void Routine_Work_Loop () // 17uS
{
        //  PORTB |= 0x01;
        Rx_Interrupt_Calc ();
        Power_Up_Timer ();
        ZC_PWM_Timing_Control_Calc ();
        wdt_reset ();	// Watch dog timer off.
        //  PORTB &= 0xfe;
}
// Sound Generator
void Go_Sound ()
{
        SOUND_ON_DUTY_TIMING = 25;			// 20% On duty
        SOUND_OFF_DUTY_TIMING = 120;			// Off duty
        SOUND_WAVE_COUNTER = 90;			// frame counts 
        ON_Sound ();
}
//
void PWR_On_Sound ()
{
        SOUND_ON_DUTY_TIMING = 50;			// 10% On duty
        SOUND_OFF_DUTY_TIMING = 450;		// Off duty
        SOUND_WAVE_COUNTER = 30;			// frame counts
        ON_Sound ();
}
//
void ON_Sound ()
{
        //PACK = 3; // 3 cell Li-po
        for (COUNTS=1 ; COUNTS<= PACK; COUNTS++)	// 3 times sound
        {
                for (LOOP2=1; LOOP2<SOUND_WAVE_COUNTER; LOOP2++)
                {
                        Sounder ();
                }
                for (LOOP=0; LOOP<30; LOOP++)  // 30mS delay
                {
                        Delay_1mS ();  // 
                        wdt_reset ();  // Reset the watchdog timer 
                }
        }
}
//
void Chirp_Up_Sound (void)
{
        int sound_freq;

        for ( sound_freq = 800; sound_freq > 300; sound_freq-- )
        {
                SOUND_ON_DUTY_TIMING = sound_freq/10;			// On duty
                SOUND_OFF_DUTY_TIMING = sound_freq*7/10;		// Off duty
                Sounder ();
                if ( CONTROL_DATA > Minimum_Spin_Power )
                {
                        return;
                }
        }
}
//
void Chirp_Down_Sound (void)
{
        int sound_freq;

        for ( sound_freq = 300; sound_freq < 800; sound_freq++ )
        {
                SOUND_ON_DUTY_TIMING = sound_freq/10;			// On duty
                SOUND_OFF_DUTY_TIMING = sound_freq*7/10;		// Off duty
                Sounder ();
                if  ((TICK_TACK_SOUND_FLAG == 1) || (CONTROL_DATA > Minimum_Spin_Power))
                {
                        return;
                }
        }
}
//
void Sounder (void)
{
        State1_On();  
        Sound_Duty_Control ();
        //
        State4_On ();  
        Sound_Duty_Control ();
        //
        State3_On ();  
        Sound_Duty_Control ();
        //
        State6_On ();
        Sound_Duty_Control ();
        //
        State5_On ();  
        Sound_Duty_Control ();
        //
        State2_On ();  
        Sound_Duty_Control ();
}

void Sound_Duty_Control ()
{
        DELAY_uS_TIME = SOUND_ON_DUTY_TIMING; 
        Delay_Microseconds ();
        //
        Braking ();  // bytege-up minus bias power of IC
        //
        DELAY_uS_TIME = SOUND_OFF_DUTY_TIMING; 
        Delay_Microseconds (); // On duty Timer
        //
        Routine_Work_Loop ();
}
//
void Step_Start ()
{
        switch (START_drive_STATE) 
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
                State4_On ();	// U=0, V=1, W=3st
                break;

        case 5:
                State5_On ();	// U=0, V=3st, W=1
                break;

        case 6:
                State6_On ();	// U=3st, V=0, W=1
        }
}
//
void Set_to_next_drive () 
{
        START_drive_STATE ++;
        if ( START_drive_STATE == 7 )
        {
                START_drive_STATE = 1;
        }
}
//
void Brake_Time () // 09/20 avoid nocking braking by PWM
{
        Braking ();
	for (LOOP1=2;LOOP1<0xff;LOOP1++);
		{
		Braking ();
		for (SOUND_ON_DUTY_TIMING=0;SOUND_ON_DUTY_TIMING<LOOP1;SOUND_ON_DUTY_TIMING++);	//simple timer
		wdt_reset (); 	// Watchdog off
		Spin_Free_N ();
		for (SOUND_ON_DUTY_TIMING=0;SOUND_ON_DUTY_TIMING<(0xff-LOOP1);SOUND_ON_DUTY_TIMING++);	//simple timer
		; 	// Watchdog off
		}	
}
//
void Rx_Interrupt_Calc ()
{
        if ( Rx_PULSE_IN_STATUS == 2 ) // 0->1->0 takes 1mS min. propo pulse
        {
                // Rx pulse width check
                if (( PULSE_WIDTH > Control_Min ) && ( PULSE_WIDTH < Control_Max )) // over 2.3mS? as check max.
                {
                        PROPO_PULSE_IN_COUNTER ++;
                        //  Serial.println(PROPO_PULSE_IN_COUNTER,DEC);
                        CONTROL_DATA_RAW = (PULSE_WIDTH-Control_Min>>4);	// about 1mS is the pulth width of propo. no need LSB nibble. Do not use "/32" to get faster instruction code.
                        //  Serial.print ("CONTROL_DATA_RAW: ");  //69 is max with Futba Tx.
                        //  Serial.println (CONTROL_DATA_RAW,DEC);        
                }
                //
                if ( CONTROL_DATA_RAW >= PWM_Frame_16kHz )
                {
                        CONTROL_DATA_RAW = PWM_Frame_16kHz; // max power limiter
                }
                MV_POINTER ++;	// Moving Average pointer
                if ( MV_POINTER == 8 )
                {
                        MV_POINTER = 0;
                }
                RX_CNT_DATA [MV_POINTER] = CONTROL_DATA_RAW;	// Moving Average Total for the control input pulse width noise filter
                //  Serial.print ("CONTROL_DATA_RAW: ");  //69 is max with Futba Tx.
                //  Serial.println (CONTROL_DATA_RAW,DEC);
                MV_TOTAL = 0;
                for ( LOOP=0; LOOP<=7; LOOP++)
                {
                        MV_TOTAL += RX_CNT_DATA [LOOP];
                }				
                //  Serial.print ("MV_TOTAL: ");
                //  Serial.println (MV_TOTAL,DEC);            
                CONTROL_DATA = MV_TOTAL>>3;	// 1/8 average for PPM RX jitter killer
                //  Serial.print ("CONTROL_DATA:");
                //  Serial.println (CONTROL_DATA,DEC);
                Rx_PULSE_IN_STATUS = 0;
        }
}
//
void Power_Up_Timer () // takes 2.5uS
{ 
        /*   Serial.print ("NEXT_POWER_UP_DOWN_TIME = ");
         Serial.println (NEXT_POWER_UP_DOWN_TIME,HEX);
         Serial.print ("TCNT1 = ");
         Serial.println (TCNT1,HEX);*/

        if ((TCNT1 - NEXT_POWER_UP_DOWN_TIME) >= Stick_Responce_Timing ) // no need to cinsider the over flow
        {
                //  PORTB &= 0xfe;        // scope trigger pin PB0.0
                NEXT_POWER_UP_DOWN_TIME = TCNT1;
                POWER_UP_TIME_COUNTER ++;
                if ( CONTROL_DATA >= LINER_ON_DUTY )	// DUTY > Rx input pulse?	
                {
                        if ( POWER_UP_TIME_COUNTER >= 3 )
                        {
                                LINER_ON_DUTY ++;	// increase ON_DUTY
                                POWER_UP_TIME_COUNTER = 0;
                        }
                }
                else
                {
                        LINER_ON_DUTY --;	// decrease ON_DUTY
                        //  LINER_ON_DUTY = CONTROL_DATA; // ( Need Rx Noise Filter )
                }
                if ( LINER_ON_DUTY >= PWM_Frame_16kHz )
                {
                        LINER_ON_DUTY = PWM_Frame_16kHz;	// Limiter for the stick resolution
                }
                ON_DUTY = LINER_ON_DUTY;
                //  *xread = INVERSE_EXP_DATA[LINER_ON_DUTY];
                //  ON_DUTY = *xread;
        } 
        //  LINER_ON_DUTY = 20;	// TEST ONLY!
        //  PORTB &= 0xfe;        // digitalWrite (D5_digital_monitor,0 
}
//-----------------------------------------------------------------------------
// Interrupt Service Routines to get Rx input pulse width
//-----------------------------------------------------------------------------
void Propo_Pulse_Width_Counting_L()        // takes 7.5uS
{
        //  PORTB |= 0x01;        // scope trigger pin PB0.0
        TIME1 = TCNT1;
        if (Rx_PULSE_IN_STATUS == 0)
        {
                Rx_PULSE_IN_STATUS = 1;
                attachInterrupt(0, Propo_Pulse_Width_Counting_H, RISING);  // poropo inversed signal by photo coupller 2uS
        }        
        //  PORTB &= 0xfe;        // scope trigger pin PB0.0      
}
//
void Propo_Pulse_Width_Counting_H ()        // takes 6uS
{
        //  PORTB |= 0x01;        // scope trigger pin PB0.0
        TIME2 = TCNT1;         // 
        PULSE_WIDTH = (TIME2-TIME1)>>1; // T1 is 500nS(1/8) clock 
        //  Serial.print ("Propo. pulth witdth(uS): ");  // 900-2300 with Futba Tx.
        //  Serial.println (PULSE_WIDTH,DEC);	
        //  PORTB &= 0xfe;        // scope trigger pin PB0.0    
        if (Rx_PULSE_IN_STATUS == 1)
        {
                Rx_PULSE_IN_STATUS = 2;
                attachInterrupt(0, Propo_Pulse_Width_Counting_L, FALLING);        // signal reversed by the photo coupler F+ 2.5uS
        }               
        //  PORTB &= 0xfe;        // This does not work on Arduino. Arduino BUG?        
}
//
void ZC_PWM_Timing_Control_Calc () // 09/18
{
        ON_TIME = ON_DUTY;        // reduce starting current
        OFF_TIME = PWM_FRAME - ON_TIME;
}
//-----------------------------------------------------------------------------
ISR(TIMER2_OVF_vect) 
{
        if (ZC_PWM_FLAG == 1)
        {
                TCNT2 = -((PWM_FRAME - ON_TIME)+10);	// some POPs time included
                PWM_OFF_EDGE_OVER_ZERO_CROSS_FLAG = 0; // 09/10
		switch ( START_drive_STATE )	// get zero cross point by comparator. +- input are reverse connected
		{
		case 1:	// Check the slope down
		case 3:
		case 5:
			{
			if (ACSR & 0x20)   // get the comparator output bit
				{
				PWM_OFF_EDGE_OVER_ZERO_CROSS_FLAG = 1;
				}
			break;				
			}
		case 2:	// Check the slope up
		case 4:
		case 6:
			{
			if (!(ACSR & 0x20))  // get the comparator output bit
				{
				PWM_OFF_EDGE_OVER_ZERO_CROSS_FLAG = 1;
				}
			}
		} 
                Spin_Free_N ();
                T2_STATE_CHANGE_CHECK_FLAG = 0;
                ZC_PWM_FLAG = 0;                
        }
        else
        {
                TCNT2 = ~ON_TIME; //  to avoid W interrout at very slow spin nearly = min. spin duty + PUSH delay
                Step_Start ();
                ZC_PWM_FLAG = 1;                
        }
}
//----------------------------------------------------------------------------
// End Of File




























































































































































