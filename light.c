// USC EE 459
// Smart Light for Arduino
// Developed by Richard Chan


#include <avr/io.h>
#include <util/delay.h>

#define Light_Sensor_Threshold 150 // Strong light -> 0
#define movement_threshold 12 
//0 ~ 1024, 1 unit = 3mV
//Room light = ~0.33V

void adc_init(void);
void init();
void HeadLightOn(int *counter, char isNightTime);
void Detect_Rear_Object();
void Beep();
void BlinkSignal(int counter, int counter2, char port, char port2);
uint16_t read_adc(uint8_t channel);

// Declare variables
int rightTurnSignalCounter, leftTurnSignalCounter;
char rightTurnIsOn, leftTurnIsOn;
char buzzIsOn_sonar, buzzIsOn_button;
int light_sensor, y_sensor, x_sensor, range_sensor;
int x_sensor_prev, y_sensor_prev, movement_timer;
int sonar_beep_timer, half_time;
char isNightTime, sonar_timer_set;

int main(void)
{
    init();
	adc_init();


    while (1) {
    	//Read all sensors
    	x_sensor = read_adc(0);
    	y_sensor = read_adc(1);
    	light_sensor = read_adc(2);
    	range_sensor = read_adc(3);

    	// Set Light Sensor Switch
    	if(light_sensor > Light_Sensor_Threshold){
    		// Dark
    		isNightTime = 1;
    	} else if (light_sensor <= Light_Sensor_Threshold && movement_timer == 0){
    		isNightTime = 0;
    	}

    	// Accelerometer Setting
    	if ((x_sensor < x_sensor_prev - movement_threshold) || (x_sensor > x_sensor_prev + movement_threshold) ||
    		(y_sensor < y_sensor_prev - movement_threshold) || (y_sensor > y_sensor_prev + movement_threshold)) {
    		// movement in x-axis greater than threshold, turn the light on for 3 second
    		movement_timer = 300;
    	}
    	x_sensor_prev = x_sensor;
    	y_sensor_prev = y_sensor;
    	// Turn the headlight on if is night time
    	//HeadLightOn(&movement_timer, isNightTime);

    	//Brake Light, if y-axis goes negative
    	if (y_sensor >= 512){
    		PORTD |= 1 <<PD4;
        } else {
            PORTD &= ~(1 << PD4);
        }
        HeadLightOn(&movement_timer, isNightTime);
        Detect_Rear_Object();
        // Turn Signals
        // Right Turn Signal
        if ((PIND & (1 << PD2)) && (rightTurnIsOn == 0) && (leftTurnIsOn == 0)) {
            //If PD2, the red button is pressed, turn on Right Turn Signal
            rightTurnSignalCounter = 250;
            rightTurnIsOn = 1;
            //PORTD |= (1 << PD3);
        }
        if ((rightTurnSignalCounter > 0) && rightTurnIsOn) {
            rightTurnSignalCounter--;

        } else if (rightTurnSignalCounter <= 0) {
            rightTurnIsOn = 0;
        }
        // Left Turn Signal
        if ((PIND & (1 << PD7)) && (leftTurnIsOn == 0) && (rightTurnIsOn == 0)) {
            // If PD7 is pressed, turn on Left Turn Signal
            leftTurnSignalCounter = 250;
            leftTurnIsOn = 1;
        }
        if ((leftTurnSignalCounter > 0) && leftTurnIsOn) {
            leftTurnSignalCounter--;

        } else if (leftTurnSignalCounter <= 0) {
            leftTurnIsOn = 0;
        }

        // Blink signal
        BlinkSignal(rightTurnSignalCounter, leftTurnSignalCounter, (1 << PD3), (1<< PD5)); 
        //Trigger is Port B0
        Beep();


    	
    	_delay_ms(10);	// Do not change this 10 ms delay
    }

    return 0;   /* never reached */
}

//Initialize data
void init(){
    //Pin PD2 is the button for Right Turn Signal
    //Pin PD7 is the button for Left Turn Signal    
    DDRB |= (1 << DDB1);    // Front Light Control
    DDRB |= (1 << DDB7);    // Warninig Light Control
    DDRD |= (1 << DDD3);    // Right Turn Signal Light
    DDRD |= (1 << DDD4);    // Center Brake Light
    DDRD |= (1 << DDD5);    // Left Turn Signal Light
    DDRD |= (1 << DDD6);    // Buzzer 
    rightTurnIsOn = 0;
    leftTurnIsOn = 0;
    leftTurnSignalCounter = 0;
    rightTurnSignalCounter = 0;
    x_sensor_prev = 0;
    y_sensor_prev = 0;
    sonar_beep_timer = 0;
}

void adc_init(void){
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1);
	ADCSRA |= (1<<ADEN);   
	ADCSRA |= (1<<ADSC);  
}

uint16_t read_adc(uint8_t channel){
	ADMUX &= 0xF0;                    //Clear the older channel that was read
	ADMUX |= channel;                //Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);                //Starts a new conversion
	while(ADCSRA & (1<<ADSC));            //Wait until the conversion is done
	return ADCW;                    //Returns the ADC value of the chosen channel
}

void BlinkSignal(int counter, int counter2, char port, char port2){
	// Blink a led that is connected to PORTD = port
	if ((counter > 0 && counter <= 50) ||
    		 (counter > 100 && counter < 150) ||
    			(counter > 200 && counter <= 250)) {
    	PORTD |= port;
    	// Enable Buzzer
        buzzIsOn_button = 1;
    	//PORTD |= (1 << DDD6);
    } else if  ((counter2 > 0 && counter2 <= 50) ||
    		 (counter2 > 100 && counter2 < 150) ||
    			(counter2 > 200 && counter2 <= 250)){
    	PORTD |= port2;
    	// Enable Buzzer
        buzzIsOn_button = 1;
    	//PORTD |= (1 << DDD6);
    } else {
		PORTD &= ~port;
		PORTD &= ~port2;
		// Disable Buzzer
        buzzIsOn_button = 0;
		//PORTD &= ~(1 << DDD6);
    	}
}

void HeadLightOn(int *counter, char isNightTime){
	if (isNightTime == 1){
		if (*counter > 0){
			PORTB |= 1 << PB1;
			*counter = *counter - 1;
		} else if (*counter <= 0){
			PORTB &= ~(1 << PB1);
		}
	} else if (isNightTime == 0){
		// Dark Outside, turn off headlight
		PORTB &= ~(1 << PB1);
		if (*counter > 0){
			*counter = *counter - 1;
		}
	}
}

void Detect_Rear_Object(){
    if (sonar_beep_timer == 0){
        // Timer is zero, reset TimerIsSet and turn off buzzer
        sonar_timer_set = 0;
    } else if (sonar_beep_timer > half_time) {
        // Timer is set, enable buzzer
        //PORTD |= (1 << PD3); // Light up right signal instead of buzzer
        //PORTD |= (1 << DDD6);
        buzzIsOn_sonar = 1;
        PORTB |= (1 << PB7);
    } else if (sonar_beep_timer < half_time) {
        // Turn off buzzer
        //PORTD &= ~(1 << PD3);
        PORTB &= ~(1 << PB7);
        buzzIsOn_sonar = 0;
        //PORTD &= ~(1 << DDD6);
    }
    if (sonar_timer_set == 0){
        // Range Sensor , if range is further than 30 inch (31)
        // With 3.3V Supply Voltage, 6.4mV/in
        // Atmega ~ 3mV/point
        // if (range_sensor < 20){
        //     sonar_beep_timer = 70;
        // }   else if (range_sensor >= 20 && range_sensor < 50){
        //     sonar_beep_timer = 150;
        // }   else if (range_sensor >= 50 && range_sensor < 80){
        //     sonar_beep_timer = 150
        // }   else if (range_sensor >= 50 && range_sensor < 200){
        //     sonar_beep_timer = 300;
        // }
        if (range_sensor < 70) {
            sonar_beep_timer = range_sensor * 1.5;
            half_time = sonar_beep_timer/2;
            sonar_timer_set = 1;
        }
    }
    sonar_beep_timer =  sonar_beep_timer - 1;
}

void Beep(){
    if (buzzIsOn_sonar || buzzIsOn_button){
        PORTD |= (1 << DDD6);
    } else {
        PORTD &= ~(1 << DDD6);
    }
}
