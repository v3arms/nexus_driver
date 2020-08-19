#include "PinChangeInt.h"
#include <math.h>


const long int BAUD_RATE = 38400;
const long int DELTA_T   = 50000;


const struct wheel_specs {
    int dir_pin;
    int pwm_pin;
    int encA_pin;
    int encB_pin;
} w1({9, 8, 4, 5}), w2({10, 11, 6, 7});


volatile struct encoder_ISRVars_p {
    volatile long int micros_old;
    volatile long int micros_new;
} w1_isr, w2_isr;


volatile int16_t pulses_count[2] = {}, dir_w1 = 1, dir_w2 = 1;
void encoder_ISR_w1() {
    pulses_count[0]++;
    // dir_w1 = !^dir_w1;
}
void encoder_ISR_w2() {
    pulses_count[1]++;
    // dir_w2 = !^dir_w2;
}


int16_t  spd[2] = {};


void setup() {
    Serial.begin(BAUD_RATE, SERIAL_8N1);
    TCCR1B = TCCR1B & 0xf8 | 0x01;
    TCCR2B = TCCR2B & 0xf8 | 0x01;
    pinMode(w1.dir_pin, OUTPUT);
    pinMode(w1.pwm_pin, OUTPUT);
    pinMode(w2.dir_pin, OUTPUT);
    pinMode(w2.pwm_pin, OUTPUT);
    PCattachInterrupt(w1.encA_pin, encoder_ISR_w1, CHANGE);
    PCattachInterrupt(w1.encB_pin, encoder_ISR_w1, CHANGE);
    PCattachInterrupt(w2.encA_pin, encoder_ISR_w2, CHANGE);
    PCattachInterrupt(w2.encB_pin, encoder_ISR_w2, CHANGE);
}


long int prev_micros = micros(),
         curr_micros = micros();


void loop() {
	curr_micros = micros();
	if (curr_micros - prev_micros > DELTA_T) {
	    if (Serial.availableForWrite() >= 2 * sizeof(int16_t)) {
            Serial.write((char*)pulses_count, 2 * sizeof(int16_t));
            
            /*
            Serial.println(curr_micros - prev_micros);
            Serial.print(pulses_count[0]);
            Serial.print(" ");
            Serial.println(pulses_count[1]);
            */
        }
		    
		prev_micros = curr_micros;
		pulses_count[0] = 0;
		pulses_count[1] = 0;
	}

	if (Serial.available() >= 4) {
        Serial.readBytes((char*)spd, 2 * sizeof(int16_t));

        digitalWrite(w1.dir_pin, spd[0] > 0);
        digitalWrite(w2.dir_pin, spd[1] > 0);
        analogWrite(w1.pwm_pin, abs(spd[0]));
        analogWrite(w2.pwm_pin, abs(spd[1]));
    }

}
