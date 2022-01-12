#define IR_LEFT_PIN 4
#define IR_RIGHT_PIN 5
#define EMIT_PIN 11

#define NB_IR_PINS 2

#define TIMEOUT_uS 40000

int ir_pins[NB_IR_PINS] = {IR_LEFT_PIN, IR_RIGHT_PIN};


class irSensors_c {
	public:
	
	void initialise() {
		
		pinMode(EMIT_PIN, OUTPUT);
		digitalWrite(EMIT_PIN, LOW);
		
		pinMode(IR_LEFT_PIN, INPUT);
		pinMode(IR_RIGHT_PIN, INPUT);
		
		delay(100);
	}

	void readAllSensors(unsigned long* arr) {

		int i;

		for(i = 0; i < NB_IR_PINS; i++) {
			pinMode(ir_pins[i], OUTPUT);
			digitalWrite(ir_pins[i], HIGH);
		}

		delayMicroseconds(10);

		for(i = 0; i < NB_IR_PINS; i++) {
			pinMode(ir_pins[i], INPUT);
		}  

		unsigned long start_time;
		unsigned long end_times[NB_IR_PINS];
		bool pins_read[NB_IR_PINS] = {false, false};
		bool loop_break = false;

		start_time = micros();
		  
		while(!loop_break) {
			
			for(i = 0; i < NB_IR_PINS; i++) {
			  
				if(digitalRead(ir_pins[i]) == LOW && !pins_read[i]) {
					end_times[i] = micros();
					pins_read[i] = true;
				}
			
				if(pins_read[0] == true && pins_read[1] == true && pins_read[2] == true) {
					loop_break = true;
					break;
				}

				if(micros() - start_time > TIMEOUT_uS) {
					Serial.print("Error: Timeout value (");
					Serial.print(TIMEOUT_uS);
					Serial.println(" uS) exceeded.");
					loop_break = true;
					break;
				}
			}
		}

		unsigned long elapsed_times[NB_IR_PINS];

		for(i = 0; i < NB_IR_PINS; i++) {
			arr[i] = end_times[i] - start_time;
		}		

		//return elapsed_times; 
	}
	
};
