;*******************************************************************************
;                                                                              *
;    Filename: Project 4                                                       *
;    Date: 25 September 2016	                                               *
;    File Version: 1                                                           *
;    Author: Muhammad Vaid	                                               *
;    Company: University of Kwa-Zulu Natal                                     *
;    Description: Digital Thermometer with user set bounds		       *
;                                                                              *
;*******************************************************************************
; Processor Inclusion
#include "p16F690.inc"
;*******************************************************************************
; CONFIG
; __config 0xFFFF
__CONFIG _FOSC_EXTRCCLK & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_ON & _FCMEN_ON

;*******************************************************************************
; Variable Definitions
;*******************************************************************************
;---------------------------------
;Delay
d1 equ 0x31		;Used in Delay
d2 equ 0x32		;Used in Delay
d3 equ 0x33
;---------------------------------
;Timer2
flag equ 0x34		;Used to generate PWM
alarmOn equ 0x56
;---------------------------------
;Timer1
isFour equ 0x35
tempFlag equ 0x36
timer equ 0x37
dutyFlag equ 0x60	;toggle alarm on and off every second
;---------------------------------
;Display
Display equ 0x38	;Current count value
Check equ 0x39		;Used to switch between SSDs - Only one SSD is lit per cycle
Units equ 0x40		;Store Units value
Tens equ 0x41		;Store Tens value
Quot equ 0x42
temp1 equ 0x43		;Temporary register
flickSpeed equ 0x55

;---------------------------------
;Main
MODE0 equ 0x44
MODE1 equ 0x45
MODE2 equ 0x46 
temp equ 0x47
buttonClicked equ 0x48
threeSec equ 0x59
;---------------------------------
;Context Saving
W_TEMP equ 0x49		    
STATUS_TEMP equ 0x50    
 
;---------------------------------
;Temperature
temperature equ 0x51	;Temperature Reading from ADC
tempLow equ 0x52	;Lower Bound for temperature
tempHigh equ 0x53	;Upper bound for temperature
mode equ 0x54		;Run MODE(0), SETUP LOW MODE(1), SETUP HIGH MODE(2)  

;EEPROM Variables 
;----------------------  
HIGH_EE_ADDR equ 0x57
LOW_EE_ADDR equ 0x58 
DATA_EE_ADDR equ 0x75
DATA_EE_DATA equ 0x76 
;----------------------  
    ORG	0x2100		; data EEPROM location
    DE	13,23		; define first four EEPROM locations as 1, 2, 3, and 4 
	;23 --> 35 degrees
	;13 --> 19 degrees
	;5,46 --> 5 and 70
;0x60
;*******************************************************************************
    ;Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000	; processor reset vector
    GOTO    SETUP		; go to beginning of program

;*******************************************************************************
; Interrupt Service Routines

ISR CODE 0x0004			;interrupt vector location
 
 ;------------CONTEXT SAVING----------------- 
    MOVWF W_TEMP 
    SWAPF STATUS,W 
    CLRF STATUS 
    MOVWF STATUS_TEMP 
;-------------------------------------------- 
    Inter
    BTFSS INTCON,T0IF		;Check for Timer0 Interrupt
    goto CHECK_T1
    goto ISR_TIMER0
    
    CHECK_T1
    BTFSS PIR1,TMR1IF		;Check for Timer1 Interrupt
    goto CHECK_T2
    goto TIMER1_OVERFLOW
    
    CHECK_T2			
    BTFSS PIR1,TMR2IF		;Check for Timer2 Interrupt
    RETFIE
    goto ISR_TIMER2
    
;*******************************************************************************
    ; SETUP
;*******************************************************************************
   SETUP
    movlw 0x01			;Store HIGH EEPROM address
    movwf HIGH_EE_ADDR
    
    movlw 0x00			;Store LOW EEPROM address
    movwf LOW_EE_ADDR    
    
    movlw d'30'
    movwf tempHigh    

    movlw d'15'
    movwf tempLow  
    
    BANKSEL TRISA
    BCF TRISB,4	    ;Tens BJT - output
    BCF TRISB,5	    ;Units BJT - output
    BCF TRISA,2	    ;Units BJT - output
    BSF TRISB,7	    ;Up Switch - input
    BSF TRISB,6	    ;Down Switch - input
    MOVLW 0x00	    
    MOVWF TRISC	    ;7-Segment Display - output
	
    BANKSEL ANSEL
    clrf ANSEL
    clrf ANSELH
    BSF ANSEL,1	    ;Analog Select for Vref
    BSF ANSEL,0	    ;Analog select for pin RA0    
    
    BANKSEL PORTA
;--------------------------------------------------------------
    ;Timer 0 Interrupt
;--------------------------------------------------------------
    BSF STATUS,5    
    MOVLW   b'00000111'
	    ; -----111 prescalar 1:256
	    ; ----0--- prescaler assigned to timer0
	    ; ---0---- Edge Select
	    ; --0----- Used as a timer
	    ; -0------ Set external interrupt off (for now)
	    ; 0------- Disable pull-up resistors
	    
    MOVWF OPTION_REG
    BCF STATUS,5    
    MOVLW d'215'	    ;REQUIRED NUMBER OF STEPS:67 (~60Hz)
    MOVWF TMR0		    ;INITIAL STEP VALUE 	
;--------------------------------------------------------------
    ;Timer 1 Interrupt
;--------------------------------------------------------------
    BANKSEL T1CON
    MOVLW   b'00110001'
	    ; -------1 Enables Timer1
	    ; ------0- Internal Clock
	    ; -----0-- Bit Ignored
	    ; ----0--- Bit Ignored
	    ; --11---- Prescalar 8
	    ; -0------ Timer1 is on
	    ; 0------- Bit Ignored
    MOVWF T1CON
    
    movlw 0xDC ; Start timer at 0BDCh
    movwf TMR1L ; which allows 62,500
    movlw 0x0B ; clocks until the
    movwf TMR1H ; timer overflows. 
    ;--------------------------------------------------------------	
    
    BANKSEL ADCON1
    MOVLW b'01010000'
    MOVWF ADCON1    
    
    BANKSEL ADCON0
    MOVLW   b'01000001'
	    ; -------1 ADC Enabled
	    ; ------0- ADC not in progress
	    ; --0000-- AN0 - Channel Select
	    ; -1------ External Vref
	    ; 0------- Left Justified 
    MOVWF ADCON0 
    BANKSEL PORTA
;*******************************************************************************
    ;INITIALISE
;*******************************************************************************
    BANKSEL PIE1		    ;Enable Interrupt
    BSF PIE1,TMR1IE
    BANKSEL PIR1		    ;CLEAR FLAG
    BCF PIR1,TMR1IF
    
    BANKSEL INTCON 
    BSF INTCON,T0IE		    ;Timer 0 Interrupt set on        
    BCF INTCON,T0IF		    ;clear interrupt flag    
    BSF INTCON,GIE		    ;Set Global Interrupts on
    BANKSEL PORTA 
    
    ;clear ports
    bcf PORTA,2
    bcf	PORTB,5    
    bcf	PORTB,6
    bcf	PORTB,7
    bcf	PORTB,4
    ;clear registers
    CLRW
    CLRF Check
    CLRF Display
    CLRF Units
    CLRF Tens
    CLRF Quot
    CLRF temp1
    CLRF isFour
    CLRF tempFlag
    CLRF temperature
    CLRF flickSpeed
    CLRF dutyFlag
    
;--------------------------------------------------------------	
    ;Set registers
    movlw d'220'
    movwf flickSpeed
    
    movf HIGH_EE_ADDR,w			;Read HIGH bound 
    call READ
    movwf tempHigh

    movf LOW_EE_ADDR,w			;Read Low Bound
    call READ
    movwf tempLow
    
    movlw d'0'
    movwf mode
    movwf MODE0
    movlw d'1'
    movwf MODE1
    movlw d'2'
    movwf MODE2
    movlw d'3'
    movwf threeSec
    clrw
    
;*******************************************************************************
    ;MACROS
;*******************************************************************************    
BEQ MACRO REG1, REG2, DEST		;branch if REG1 == REG2
    movf REG2, W			;W &lt;- REG2
    subwf REG1, W			;W &lt;- REG1 - REG2
    btfsc STATUS, Z			;if result was nonzero: skip out
    goto DEST				;otherwise jump
    ENDM    

BNEQ MACRO REG1, REG2, DEST		;branch if REG1 == REG2
    movf REG2, W			;W &lt;- REG2
    subwf REG1, W			;W &lt;- REG1 - REG2
    btfss STATUS, Z			;if result was nonzero: skip out
    goto DEST				;otherwise jump
    ENDM      

BGTE MACRO REG1, REG2, DEST		;branch if REG1 >= REG2
    movf REG2, W			;W &lt;- REG2
    subwf REG1, W			;W &lt;- REG1 - REG2    
    btfsc STATUS, C
    GOTO DEST
    ENDM  
    
BLT MACRO REG1, REG2, DEST		;branch if REG1 < REG2
    movf REG2, W			;W &lt;- REG2
    subwf REG1, W			;W &lt;- REG1 - REG2    
    btfss STATUS, C
    GOTO DEST
    ENDM      

;*******************************************************************************
    ;MAIN PROGRAM
;*******************************************************************************

 MAIN_PROG CODE
  
START
    clrf PCLATH
    BEQ mode,MODE0,RUN_MODE
    BEQ mode,MODE1,Setup_LOW
    goto Setup_HIGH
;************************************************
    ;RUN_MODE
;************************************************
RUN_MODE
    movlw d'0'				;Main mode, display temp and poll buttons
    movwf mode
    movlw d'220'
    movwf flickSpeed
    call Delay05s
    call Delay05s
    MOVFW temperature
    MOVWF Display  

;----------------Alarm----------------
ALARM  
    movfw tempHigh
    subwf temperature,w
    
    BTFSS STATUS,C			;will skip if temp > high
    GOTO CheckLow
    call START_ALARM
    
    CheckLow
    movfw tempLow
    subwf temperature,w
    
    BTFSC STATUS,C			;will skip if temp < low
    GOTO AlarmOff
    call START_ALARM
    GOTO UP0
        
AlarmOff
    call STOP_ALARM

;----------------Poll Buttons----------------
UP0
    BTFSS PORTB,7			;check if Up button is pressed
    goto DOWN0				;if not check Down button
    call DELAY_10ms		
    BTFSS PORTB,7
    goto DOWN0
    ;UP clicked
    clrf timer
    clrf isFour
    bsf buttonClicked,0
        
    Check_ReleaseUP
    BTFSS buttonClicked,0
    GOTO JUMPUP0		    
    bcf buttonClicked,0			;if set - reset timer
    clrf timer
    clrf isFour
    JUMPUP0
	BGTE timer,MODE2,High_Mode	;if held up for 2s go to setup high
    BTFSC PORTB,7
    GOTO Check_ReleaseUP
    Goto Continue0
    
 DOWN0 
    BTFSS PORTB,6			;check if Down button is pressed
    goto Continue0			;if not, go back to start of loop
    call DELAY_10ms
    BTFSS PORTB,6
    goto Continue0 
    ;DOWN clicked
    clrf timer
    clrf isFour
    bsf buttonClicked,0
    
    Check_ReleaseDOWN
    BTFSS buttonClicked,0
    GOTO JUMPDOWN0 	    
    bcf buttonClicked,0			;if set - reset timer
    clrf timer
    clrf isFour
    JUMPDOWN0 
	BGTE timer,MODE2,Low_Mode	;if held down for 2s go to setup LOW
    BTFSC PORTB,6
    GOTO Check_ReleaseDOWN
    
 Continue0
    clrf timer
    clrf isFour
    bcf buttonClicked,0
    GOTO START

;************************************************
    ;Low_Mode
;************************************************
Low_Mode
    call STOP_ALARM
    clrf timer
    clrf isFour
    movlw d'1'				;change mode to 1
    movwf mode
    movlw d'160'
    movwf flickSpeed
    incf tempLow
    
Setup_LOW
    movfw tempLow			;move tempLow to Display
    movwf Display
	BGTE timer,threeSec,Write	;if timer reaches 3sec, write to EEPROM and go back to run

 UP1
    BTFSS PORTB,7			;check if Up button is pressed
    goto DOWN1				;if not check Down button
    call DELAY_10ms		
    BTFSS PORTB,7
    goto DOWN1
    ;UP clicked - reset timer and inc temp_low
    clrf timer
    clrf isFour
    incf tempLow,1
    
    ;--------------------------------------------------------------	
    ;If count overflows, reset W and keep temp @ 99
    ;--------------------------------------------------------------	
    movf tempLow,w
    sublw d'100'
    
    BTFSS STATUS,Z			;At 100, zero flag will be set - skip the next line and keep tempLow at 99
    GOTO Continue1
	
    movlw d'99'				;set tempLow to 99
    movwf tempLow
    Goto Continue1
    
 DOWN1 
    BTFSS PORTB,6			;check if Down button is pressed
    goto START				;if not, go back to start of loop
    call DELAY_10ms
    BTFSS PORTB,6
    goto START 
    ;DOWN clicked - reset timer and dec tempLow
    clrf timer
    clrf isFour
    decf tempLow,1
    
    ;--------------------------------------------------------------	
    ;Test MSB - Will be set if underflow occurred (negative value)
    ;--------------------------------------------------------------	
    BTFSS tempLow,7			;If tempLow is negative, MSB will be set - skip the next line and keep tempLow value at 0
    Goto Continue1
	
    clrf tempLow
    Goto Continue1
    
    Continue1
    MOVFW tempLow			;move updated tempLow to Display
    movwf Display
    
 Check_Release1
    clrf timer
    clrf isFour
    BTFSC PORTB,7
    GOTO Check_Release1
    BTFSC PORTB,6
    GOTO Check_Release1
    goto START   
    
;************************************************
    ;High_Mode
;************************************************   
High_Mode    
    call STOP_ALARM
    clrf timer
    clrf isFour
    movlw d'2'				;change mode to 2
    movwf mode
    movlw d'160'
    movwf flickSpeed
    decf tempHigh
    
Setup_HIGH
    movfw tempHigh			;move tempHigh to Display
    movwf Display
	BGTE timer,threeSec,Write

 UP2
    BTFSS PORTB,7			;check if Up button is pressed
    goto DOWN2				;if not check Down button
    call DELAY_10ms		
    BTFSS PORTB,7
    goto DOWN2
    ;UP clicked - reset timer and inc tempHigh
    clrf timer
    clrf isFour
    incf tempHigh,1
    ;--------------------------------------------------------------	
    ;If count overflows, reset W and keep temp @ 99
    ;--------------------------------------------------------------	
    movf tempHigh,w
    sublw d'100'
    
    BTFSS STATUS,Z			;At 100, zero flag will be set - skip the next line and keep tempHigh at 99
    GOTO Continue2
	
    movlw d'99'				;keep tempHigh at 99
    movwf tempHigh
    Goto Continue2
    
 DOWN2
    BTFSS PORTB,6			;check if Down button is pressed
    goto START				;if not, go back to start of loop
    call DELAY_10ms
    BTFSS PORTB,6
    goto START 
    clrf timer
    clrf isFour
    decf tempHigh,1
    ;--------------------------------------------------------------	
    ;Test MSB - Will be set if underflow occurred (negative value)
    ;--------------------------------------------------------------	
    BTFSS tempHigh,7			;If tempHigh is negative, MSB will be set - skip the next line and keep tempHigh value at 0
    Goto Continue2
	
    clrf tempHigh
    Goto Continue2
    ;DOWN clicked - reset timer and dec temp_high
        
    Continue2
    MOVFW tempHigh			;move updated tempHigh to Display
    movwf Display
    
 Check_Release2
    clrf timer
    clrf isFour
    BTFSC PORTB,7
    GOTO Check_Release2
    BTFSC PORTB,6
    GOTO Check_Release2
    goto START 

;************************************************
    ;Write to EEPROM
;************************************************   
Write					;Write to EEPROM, clear timer and return to RUN_MODE
    call WRITE
    clrf timer
    clrf isFour
    goto RUN_MODE
    
;*******************************************************************************
    ;Interrupt Service Routine - Timer1 - Seconds counter
;*******************************************************************************    
   TIMER1_OVERFLOW
    bcf T1CON,0				;Turn the timer off.
    BCF PIR1,TMR1IF 
    movlw 0xDC				;Start timer at 0BDCh
    movwf TMR1L				;which allows 62,500
    movlw 0x0B				;clocks until the
    movwf TMR1H				;timer overflows.
    
    BTFSC tempFlag,0			;Toggle alarm on and off
    goto SET_IT
    BSF tempFlag,0
    goto DONE1
    
SET_IT
    BTFSC dutyFlag,0
    goto SetDutyFlag
    BSF dutyFlag,0
    goto DONE1
    
SetDutyFlag
    BCF dutyFlag,0
    BCF tempFlag,0   
      
DONE1
    incf isFour
    movfw isFour
    sublw d'2'
    btfss STATUS,Z			;if isFOur = 2, Count++ - Every 2 interrupts one sec passes
    goto NotFour
    clrf isFour
    incf timer
    
NotFour
    bsf T1CON,0				;Turn the timer back on.
    
;-----------Context Saving-----------------                
    SWAPF STATUS_TEMP,W
    MOVWF STATUS 
    SWAPF W_TEMP,F 
    SWAPF W_TEMP,W   
;-------------------------------------------   
    
    goto CHECK_T2
;*******************************************************************************
    ;Interrupt Service Routine - TIMER2 - PWM
;*******************************************************************************         
ISR_TIMER2
    BTFSC flag,0			;Used to toggle pin using flag - bit-banging
    goto SET_IT2
    BCF PORTA,2
    BSF flag,0
    goto DONE2
    
SET_IT2
    BSF PORTA,2
    BCF flag,0   
    
    
DONE2    
    BANKSEL PIR1
    BCF PIR1,TMR2IF			;Clear Flag
    BANKSEL PORTA
    
;-----------Context Saving-----------------                
    SWAPF STATUS_TEMP,W
    MOVWF STATUS 
    SWAPF W_TEMP,F 
    SWAPF W_TEMP,W   
;-------------------------------------------     
    RETFIE 
    
;*******************************************************************************
    ;Interrupt Service Routine - Timer0 - Multiplexing & Display
;*******************************************************************************    
ISR_TIMER0
  
    BTFSC mode,0
    goto Cont
    BTFSC mode,1
    goto Cont
   
    BSF ADCON0,GO
    BTFSC ADCON0,GO			;Poll for end of conversion
    GOTO $-1
    MOVF ADRESH,W	
    MOVWF temperature			;Store value 
    
    Cont
    MOVFW Display			;move temp value to Display
    Call Binary2BCD			;call method to convert binary to BCD
    
   ;Tens and Units are switched on by sending a signal to the base of the corresponding BJT
   ;If Check = 0 - Turn Tens On
   ;If Check = 1 - Turn Units On
    BTFSC Check,0
    GOTO DispUnits
    
DispTens
    BCF PORTB,4				;Units OFF
    movlw HIGH SSD_Output		;Avoid PCL Rollover
    movwf PCLATH 
    MOVF Tens,w				;move tens value into w
    call SSD_Output			;Get SSD output value
    MOVWF PORTC				;output Tens to SSD
    
    BSF Check,0
    BSF PORTB,5				;Tens ON
    GOTO SET_TIMER			;skip DispUnits
   
DispUnits
    BCF PORTB,5				;Tens OFF
    movlw HIGH SSD_Output		;Avoid PCL rollover
    movwf PCLATH     
    MOVF Units,w			;move units value into w
    call SSD_Output			;Get SSD output value
    MOVWF PORTC
   
    BCF Check,0
    BSF PORTB,4				;Units ON
   
SET_TIMER    
;-------------RESET TIMER0------------------        
    MOVFW flickSpeed
    MOVWF TMR0	    ;Reset step value
    BCF INTCON,T0IF ;Clear Interrupt Flag
;-------------------------------------------       

;-----------Context Saving-----------------                
    SWAPF STATUS_TEMP,W
    MOVWF STATUS 
    SWAPF W_TEMP,F 
    SWAPF W_TEMP,W   
;-------------------------------------------   
    goto CHECK_T1
    
;*******************************************************************************	
    ;ALARM
;*******************************************************************************	
    
    START_ALARM
    
    ;----Duty Cycle----
    BTFSC dutyFlag,0
    goto STOP_ALARM
;------------CONTEXT SAVING----------------- 
    MOVWF W_TEMP 
    SWAPF STATUS,W 
    CLRF STATUS 
    MOVWF STATUS_TEMP 
;-------------------------------------------- 
   
    movlw d'1'			    
    movwf temp
    BEQ alarmOn, temp, DONT_START	;Flag is set? --> ALARM already on --> skip
    
    BANKSEL TRISB
    BCF TRISA,2
    
    BANKSEL PIE1
    BSF PIE1,TMR2IE

    BANKSEL T2CON
    MOVLW b'00000101'			;Configure TMR2 @ 440Hz
    MOVWF T2CON
 
    BANKSEL PIR1
    BCF PIR1,TMR2IF
    
    BANKSEL INTCON 
    BSF INTCON,PEIE
    BANKSEL PORTA
    
    BSF alarmOn,0
    
DONT_START      
;-----------Context Saving-----------------                
    SWAPF STATUS_TEMP,W
    MOVWF STATUS 
    SWAPF W_TEMP,F 
    SWAPF W_TEMP,W   
;-------------------------------------------   
    return
    
STOP_ALARM
    
    BANKSEL TRISB			;Disable pin
    BSF TRISA,2
    
    BANKSEL PIE1			;Disable Interrupt
    BCF PIE1,TMR2IE
 
    BANKSEL PIR1			;Clear Flag
    BCF PIR1,TMR2IF
    
    BANKSEL PORTA
    BCF alarmOn,0
    
;-----------Context Saving-----------------                
    SWAPF STATUS_TEMP,W
    MOVWF STATUS 
    SWAPF W_TEMP,F 
    SWAPF W_TEMP,W   
;-------------------------------------------   
    return 
    	 
;*******************************************************************************
    ;0.5S DELAY        
;*******************************************************************************     

DELAY_0.5s
    ;499994 cycles
    movlw	0x08
    movwf	d1
    movlw	0x2F
    movwf	d2
    movlw	0x03
    movwf	d3
Delay_01
    decfsz	d1, f
    goto	$+2
    decfsz	d2, f
    goto	$+2
    decfsz	d3, f
    goto	Delay_01
    ;6 cycles
    goto	$+1
    goto	$+1
    goto	$+1	

    return        
;*******************************************************************************
    ;10MS DELAY
;*******************************************************************************	 

DELAY_10ms
    ;9993 cycles
    movlw 0xE7
    movwf d1
    movlw 0x04
    movwf d2
Delay
    decfsz d1, f
    goto Delay
    decfsz d2, f
    goto Delay
    ;3 cycles
    goto $+1
    nop
    ;4 cycles (including call)	 
    return
    
;*******************************************************************************
    ;SSD Binary corresponding to PORTC configuration
;*******************************************************************************	   
;Common Cathode SSD
    SSD_Output
    addwf PCL
     
    retlw b'01011111' ;0
    retlw b'01000100' ;1
    retlw b'01101011' ;2
    retlw b'01101110' ;3
    retlw b'01110100' ;4
    retlw b'00111110' ;5
    retlw b'00111111' ;6
    retlw b'01001100' ;7
    retlw b'01111111' ;8
    retlw b'01111110' ;9    
    
;*******************************************************************************	
    ;BINARY TO BCD CONVERSION
;*******************************************************************************		
Binary2BCD

    movwf temp1			    ;store input
    bcf STATUS, C		    ;If Input>99, exit subroutine
    
    sublw d'99'			    ;if number is less than 99 then carry flag will be set
    
    btfss STATUS, C		    ;Only if carry flag set, continue with conversion
    goto EndConv
 
    Begin    
    movlw d'0'			    ;initialise: quotient = 0
    movwf Quot
    movlw d'10'			    ;store 10 in W register
    
    Divide			    ;keep subtracting until value reaches/passes zero
	
    incf Quot,1			    ;quotient += 1
	
    subwf temp1,1		    ;temp1 -= 10

    ;check if result is positive
    btfsc STATUS, C		    ;if carry flag not set, jump back to divide
    goto Divide

    btfsc STATUS, Z		    ;if zero flag not set, jump back to divide
    goto Divide

    EndDivide			    ;subtracted one time too many. Dec quotient and add 10 back to value
    decf Quot,1			    ;quotient -= 1
    addwf temp1,1		    ;value += 10
	
    MOVFW Quot			    ;Copy quotient value to Tens register
    MOVWF Tens
    MOVFW temp1			    ;Copy remainder value to Units register
    MOVWF Units
	    
EndConv
    return	
    
;*******************************************************************************
    ;0.5MS DELAY
;*******************************************************************************
Delay05s
    ;99993 cycles
    movlw	0x1E
    movwf	d1
    movlw	0x4F
    movwf	d2
Delay05s_0
    decfsz	d1, f
    goto	$+2
    decfsz	d2, f
    goto	Delay05s_0
    ;3 cycles
    goto	$+1
    nop
    ;4 cycles (including call)
    return
    
;*******************************************************************************
    ;Read from EEPROM
;*******************************************************************************        
READ    

    BANKSEL EEADR 
    MOVWF EEADR			    ;Data Memory
    
    BANKSEL EECON1		    ;Address to read BANKSEL EECON1 
    BCF EECON1, EEPGD		    ;Point to DATA memory
    BSF EECON1, RD		    ;EE Read

    BANKSEL EEDAT
    MOVF EEDAT, W		    ;W=EDAT
    BCF STATUS, RP1		    ;Bank 0    
    
    return
    
;*******************************************************************************
    ;Write to EEPROM
;*******************************************************************************    
WRITE
    MOVLW d'1'
    MOVWF temp
    BNEQ mode,temp,WRITE_HIGH	    ;Check which mode value need to be written

WRITE_LOW    
    MOVF LOW_EE_ADDR, W
    MOVWF DATA_EE_ADDR
    
    MOVF tempLow, W
    MOVWF DATA_EE_DATA
    
    GOTO CONTINUE_WRITE 
    
WRITE_HIGH
    MOVF HIGH_EE_ADDR, W
    MOVWF DATA_EE_ADDR
    
    MOVF tempHigh, W
    MOVWF DATA_EE_DATA
    
    GOTO CONTINUE_WRITE
    
CONTINUE_WRITE    
    BANKSEL EEADR 
    MOVF DATA_EE_ADDR, W
    MOVWF EEADR			    ;Data Memory Address to write 
    MOVF DATA_EE_DATA, W
    MOVWF EEDAT			    ;Data Memory Value to write

    BANKSEL EECON1
    BCF EECON1, EEPGD		    ;Point to DATA memory
    BSF EECON1, WREN		    ;Enable writes

    BCF INTCON, GIE		    ;Disable INTs.
    BTFSC INTCON, GIE 
    GOTO $-2

    MOVLW 0x55
    MOVWF EECON2		    ;Write 55h

    MOVLW 0xAA
    MOVWF EECON2		    ;Write AAh

    BSF EECON1, WR		    ;Set WR bit to begin write 
    BSF INTCON, GIE		    ;Enable INTs.
    
    BCF EECON1, WREN		    ;Disable writes
    BANKSEL 0x00		    ;Bank 0 
    
END_WRITE    
    return
    
;*******************************************************************************	
    END
;*******************************************************************************