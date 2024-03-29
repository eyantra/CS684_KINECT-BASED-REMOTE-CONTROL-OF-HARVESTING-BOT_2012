#include<avr/io.h>s
#include<avr/interrupt.h>
#include<util/delay.h>
#include "lcd.c"
/*to store received data from UDR0 */
int data; 
/*No. of slots moved by the Left motor of the firebird bot.
Inremented by 11 everytime the motor moves 1 slot which is 
tracked using an interrupt
*/

unsigned long int ShaftCountLeft = 0;

/*No. of slots moved by the Right motor of the firebird bot.
Inremented by 11 everytime the motor moves 1 slot which is 
tracked using an interrupt
*/
unsigned long int ShaftCountRight = 0;

/*
Half of the distance between the 2 motors of the firebird bot (in mm)
*/
int BotRadius = 80;
/*
Radius of either wheel
*/
float WheelRadius = 25.5;


unsigned int angledata=0,mode = 0;
int flag4=0, flag6=0, flag7=0, flag9=0,flag3 = 0;
int flag10=0,flagin=0,flagset=0;

int angle1=0,angle2=0;
int zleft=0;zright=0,zldiff=0,zrdiff=0,rad;

unsigned char sensor = 0;
unsigned char ADC_Value;
unsigned char Front_Sharp_Sensor=0;
unsigned char Front_IR_Sensor=0;

/*config
	TCCR1B = 0x00; 				stop
 	TCNT1H = 0xFC; 				Counter high value to which OCR1xH value is to be compared with
 	TCNT1L = 0x01;				Counter low value to which OCR1xH value is to be compared with
 	OCR1AH = 0x03;				Output compare eegister high value for servo 1
 	OCR1AL = 0xFF;				Output Compare Register low Value For servo 1
 	OCR1BH = 0x03;				Output compare eegister high value for servo 2
 	OCR1BL = 0xFF;				Output Compare Register low Value For servo 2
 	OCR1CH = 0x03;				Output compare eegister high value for servo 3
 	OCR1CL = 0xFF;				Output Compare Register low Value For servo 3
 	ICR1H  = 0x03;	
 	ICR1L  = 0xFF;
 	TCCR1A = 0xAB; 				{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 									For Overriding normal port functionalit to OCRnA outputs.
				  				{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode
 	TCCR1C = 0x00;
 	TCCR1B = 0x0C; 				WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
*/
void timer1_init(void)
{
 	TCCR1B = 0x00; 				//stop
 	TCNT1H = 0xFC; 				//Counter high value to which OCR1xH value is to be compared with
 	TCNT1L = 0x01;				//Counter low value to which OCR1xH value is to be compared with
 	OCR1AH = 0x03;				//Output compare eegister high value for servo 1
 	OCR1AL = 0xFF;				//Output Compare Register low Value For servo 1
 	OCR1BH = 0x03;				//Output compare eegister high value for servo 2
 	OCR1BL = 0xFF;				//Output Compare Register low Value For servo 2
 	OCR1CH = 0x03;				///Output compare eegister high value for servo 3
 	OCR1CL = 0xFF;				//Output Compare Register low Value For servo 3
 	ICR1H  = 0x03;	
 	ICR1L  = 0xFF;
 	TCCR1A = 0xAB; 				/*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 									For Overriding normal port functionalit to OCRnA outputs.
				  				{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 	TCCR1C = 0x00;
 	TCCR1B = 0x0C; 				//WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
function to configure pin 5 for servo motor 1 operation
*/
void servo1_pin_config (void)
{
 	DDRB  = DDRB | 0x20;  		//making PORTB 5 pin output
 	PORTB = PORTB | 0x20; 		//setting PORTB 5 pin to logic 1
}

/*
function to configure pin 6 for servo motor 2 operation
*/
void servo2_pin_config (void)
{
 	DDRB  = DDRB | 0x40;  		//making PORTB 6 pin output
 	PORTB = PORTB | 0x40; 		//setting PORTB 6 pin to logic 1
}


/*
function to configure pin 7 for servo motor 3 operation
*/
void servo3_pin_config (void)
{
 	DDRB  = DDRB | 0x80;  		//making PORTB 7 pin output
 	PORTB = PORTB | 0x80; 		//setting PORTB 7 pin to logic 1
}

/*servo_free functions unlocks the servo motors from the any angle 
and make them free by giving 100% duty cycle at the PWM. This function can be used to 
reduce the power consumption of the motor if it is holding load against the gravity.
*/
void servo_1_free (void) 	//makes servo 1 free rotating
{
 	OCR1AH = 0x03; 
 	OCR1AL = 0xFF; 			//Servo 1 off
}

void servo_2_free (void) 	//makes servo 2 free rotating
{
 	OCR1BH = 0x03;
 	OCR1BL = 0xFF; 			//Servo 2 off
}

void servo_3_free (void) 	//makes servo 3 free rotating
{
 	OCR1CH = 0x03;
 	OCR1CL = 0xFF; 			//Servo 3 off
}

/*
Function to rotate Servo 1 by a specified angle in the multiples of 2.25 degrees
*/
void servo_1(unsigned char degrees)  
{
 	float PositionPanServo = 0;
 	PositionPanServo = ((float)degrees / 1.86) + 35.0;
 	OCR1AH = 0x00;
 	OCR1AL = (unsigned char) PositionPanServo;
}


/*Function to rotate Servo 2 by a specified angle in the multiples of 2.25 degrees*/
void servo_2(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 	OCR1BH = 0x00;
 	OCR1BL = (unsigned char) PositionTiltServo;
}


/*Function to rotate Servo 3 by a specified angle in the multiples of 2.25 degrees*/
void servo_3(unsigned char degrees)
{
 	float PositionTiltServo = 0;
 	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 	OCR1CH = 0x00;
 	OCR1CL = (unsigned char) PositionTiltServo;
}

/*
Configure all servo pins
*/
void port_init_servo(void)
{ 
	servo1_pin_config(); 		//Configure PORTB 5 pin for servo motor 1 operation
 	servo2_pin_config(); 		//Configure PORTB 6 pin for servo motor 2 operation 
 	servo3_pin_config(); 		//Configure PORTB 7 pin for servo motor 3 operation  
}

/*
Pin configuration for firebird LCD operation
*/
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

/*
Function to configure INT4 (PORTE 4) pin as input for the left position encoder
*/
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pullup for PORTE 4 pin
}

/*
Function to configure INT5 (PORTE 5) pin as input for the right position encoder
*/
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 5 pin as input
 PORTE = PORTE | 0x20; //Enable internal pullup for PORTE 5 pin
}

/*
Configuration of motor pins
*/
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRE = 0x0F | 0x0F;
 PORTE = PORTE & 0x00; 
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

/*
Function to initialize ports
*/
void port_init()
{
	adc_pin_config();
	motion_pin_config();
	buzzer_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); 	
	lcd_port_config();
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

/*Function To Initialize UART0
 desired baud rate:9600
 actual baud rate:9600 (error 0.0%)
 char size: 8 bit
 parity: Disabled
*/
 void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}


/*
Left position encoder pin setup
*/
void left_position_encoder_interrupt_init(void)
{
	cli();
	EICRB = EICRB | 0x02;
	EIMSK = EIMSK | 0x10;
	sei();
}


/*
Right position encoder pin setup
*/
void right_position_encoder_interrupt_init(void)
{
	cli();
	EICRB = EICRB | 0x08;
	EIMSK = EIMSK | 0x20;
	sei();
}

/*
Function to return the no. of motor slots to move given an angle in radians
*/
int get_count(unsigned int angle)
{
	float arcDist = (angle*(3.14/180)) * BotRadius; //distance travelled by the wheels = radius * angle;
	float wheelCircum = 2 * 3.14 * WheelRadius;
	int count = (30 * arcDist) / wheelCircum;
	return count;

}

/*
Left hard rotation
*/
void rotate_left_hard(unsigned int count)
{
	//float ArcLength = BotRadius * angle;
	//float RequiredShaftCount = ArcLength/
	unsigned long int ReqdShaftCount = 0;
	ReqdShaftCount = (unsigned long int)count;
	ShaftCountLeft = ShaftCountRight = 0;
	PORTA = 0x05;
	while(1){
	lcd_print(1,1,ShaftCountLeft,4);
		if(ShaftCountLeft > ReqdShaftCount){
			break;
		}
	}
	PORTA=0x00;
	flag4=0;
}


/*
Left soft rotation
*/
void rotate_left_soft(unsigned int count)
{
	//float ArcLength = BotRadius * angle;
	//float RequiredShaftCount = ArcLength/
	unsigned long int ReqdShaftCount = 0;
	ReqdShaftCount = (unsigned long int)count;
	ShaftCountLeft = ShaftCountRight = 0;
	PORTA = 0x04;
	while(1){
		lcd_print(1,1,angledata,4);
		if(ShaftCountRight > ReqdShaftCount){
			break;
		}
	}
	PORTA=0x00;
	flag7=0;
}

/*
Right hard rotation
*/
void rotate_right_hard(unsigned int count)
{
	//float ArcLength = BotRadius * angle;
	//float RequiredShaftCount = ArcLength/
	unsigned long int ReqdShaftCount = 0;
	ReqdShaftCount = (unsigned long int) count;
	ShaftCountLeft = ShaftCountRight = 0;
	PORTA = 0x0A;
	while(1){
	lcd_print(1,1,angledata,4);
		if(ShaftCountLeft > ReqdShaftCount){
			break;
		}
	}
		PORTA=0x00;
		flag6=0;
}

/*
Right soft rotation
*/
void rotate_right_soft(unsigned int count)
{
	//float ArcLength = BotRadius * angle;
	//float RequiredShaftCount = ArcLength/
	unsigned long int ReqdShaftCount = 0;
	ReqdShaftCount = (unsigned long int)count;
	ShaftCountLeft = ShaftCountRight = 0;
	PORTA = 0x02;
	while(1){
		lcd_print(1,1,ShaftCountLeft,4);
		if(ShaftCountLeft > ReqdShaftCount){
			break;
		}
	}
	PORTA=0x00;
	flag9=0;
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/*
Interrupt service routine to detect shaftcountleft
*/
ISR(INT4_vect, ISR_NOBLOCK)
{
	ShaftCountLeft++;
	
}

/*
Interrupt service routine to detect shaftcountright
*/
ISR(INT5_vect, ISR_NOBLOCK)
{
	ShaftCountRight++;

}



/*
Interrupt  service routine to detect signal sent by the wieless module on UDR0
*/
ISR(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{	
	Front_Sharp_Sensor = ADC_Conversion(11);
	Front_IR_Sensor = ADC_Conversion(6);
	print_sensor(1,1,11);	//Prints Value of Front Sharp Sensor
	print_sensor(1,5,6);	//Prints Value of Front IR Sensor
	sensor=0;
	if(Front_IR_Sensor<220 || Front_Sharp_Sensor > 130)
	{
		sensor=1;
		PORTA=0x00;
	}
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 

	UDR0 = data; 				//echo data back to PC
	lcd_print(2,1,(char)UDR0,4);

	if(data == 1)
	{
		mode = 1;		
	}
	else if(data == 2)
	{
		mode = 2;		
	}
	else if(data == 3)
	{
		mode = 3;		
	}
	else if(data == 8 && mode==1) //ASCII value of 8
	{
		if(sensor==0)
		{
			PORTA=0x06;  //forward
			//lcd_print(1,1,data,4); 
		}
	}
	else if(data == 9 && mode==1) 
	{
		PORTA=0x09; //back
	}
	else if(data == 5 && mode==1) //ASCII value of 5
	{
		PORTA=0x00; //stop
	}
	else if(data > 11 && data < 55 && mode==1) //ASCII value of 4
	{	lcd_print(1,1,(char)data,4);
		flag6=1;  //hard right
		angledata = get_count(data - 10);
	}

	else if(data >= 55 && mode==1)
	{	lcd_print(1,1,(char)data,4);
		flag4=1; //hard left
		angledata = get_count(data-55);

	}

	else if(data > 11 && data < 55 && mode==2) 
	{	lcd_print(1,1,(char)data,4);
		flag9=1;  //soft right
		angledata = get_count(data - 10);
	}

	else if(data >= 55 && mode==2) 
	{	lcd_print(1,1,(char)data,4);
		flag7=1; //soft left
		angledata = get_count(data-55);

	}
	else if(mode==3 && data >= 90)
	{
		data=data-90;
		if(angle2-data>2 || angle2-data<-2)
		{
				servo_3(data);
				angle2=data;
		}
	
	}
	else if (mode==3 && data>=0 && data < 90)
	{
		if(angle1-data>7 || angle1-data<-7)
		{
			angle1=data;
			servo_1(data);
			//servo_2(190-data);
		}
	
	}

/*	else  //ASCII value of 5
	{
		PORTA=0x00; //stop
	}
*/
}


/*
Function To Initialize all The Devices
*/
void init_devices()
{
 cli(); //Clears the global interrupts
 
 port_init();  //Initializes all the ports
 uart0_init(); //Initailize UART1 for serial communiaction
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 port_init_servo();
 adc_init();
 timer1_init();
 sei();   //Enables the global interrupts
}



//Main Function
int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	servo_1(0);
	_delay_ms(30);
	servo_2(0);
	_delay_ms(30);
	servo_3(0);
	_delay_ms(30);
	while(1){	
			sensor=0;
			if(flag4==1) {rotate_left_hard(angledata); angledata = 0;} 
			if(flag6==1) {rotate_right_hard(angledata); angledata = 0;}
			if(flag7==1) {rotate_left_soft(angledata); angledata = 0;}
			if(flag9==1) {rotate_right_soft(angledata); angledata = 0;}
		
	};
	servo_1_free();
	servo_2_free();
	servo_3_free();
}

