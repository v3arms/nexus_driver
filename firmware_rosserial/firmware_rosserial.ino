// future
#include <ros.h>
#include <std_msgs/Int16.h>
#include "PinChangeInt.h"


const long int DELTA_T   = 50000;


const char enc_state_map[][4] = {{0, -1, 1, 0}, 
                                 {1, 0, 0, -1},
                                 {-1, 0, 0, 1},
                                 {0, 1, -1, 0}};


struct wheel_specs {
    int dir_pin;
    int pwm_pin;
    int encA_pin;
    int encB_pin;

    char get_enc_state() {
        int A = digitalRead(encA_pin),
            B = digitalRead(encB_pin);
        return (A << 1) + B;
    }
} w1({9, 8, 4, 5}), w2({10, 11, 6, 7});


int16_t  spd[2] = {}, 
         w1_old_enc_state, w1_new_enc_state,
         w2_old_enc_state, w2_new_enc_state;


volatile int16_t pulses_count[2] = {};
void w1_isr() {
    pulses_count[0]++;
    w1_old_enc_state = w1_new_enc_state;
    w1_new_enc_state = w1.get_enc_state();
}
void w2_isr() {
    pulses_count[1]++;
    w2_old_enc_state = w2_new_enc_state;
    w2_new_enc_state = w2.get_enc_state();
}


void w1SetPwm(const std_msgs::Int16& pwm) {
    digitalWrite(w1.dir_pin, pwm.data > 0);
    analogWrite(w1.pwm_pin, abs(pwm.data));
}
void w2SetPwm(const std_msgs::Int16& pwm) {
    digitalWrite(w2.dir_pin, pwm.data < 0);
    analogWrite(w2.pwm_pin, abs(pwm.data));
}


ros::NodeHandle nh;
std_msgs::Int16 enc_state[2], enc_pwm[2];

ros::Publisher w1State("nexus/w1/state", &enc_state[0]);
ros::Publisher w2State("nexus/w2/state", &enc_state[1]);
ros::Subscriber<std_msgs::Int16> w1Pwm("nexus/w1/control_effort", &w1SetPwm);
ros::Subscriber<std_msgs::Int16> w2Pwm("nexus/w2/control_effort", &w2SetPwm);


void setup() {
    // Serial.begin(BAUD_RATE, SERIAL_8N1);
    TCCR1B = TCCR1B & 0xf8 | 0x01;
    TCCR2B = TCCR2B & 0xf8 | 0x01;
    pinMode(w1.dir_pin, OUTPUT);
    pinMode(w1.pwm_pin, OUTPUT);
    pinMode(w2.dir_pin, OUTPUT);
    pinMode(w2.pwm_pin, OUTPUT);

    pinMode(w1.encA_pin, INPUT);
    pinMode(w1.encB_pin, INPUT);
    pinMode(w2.encA_pin, INPUT);
    pinMode(w2.encB_pin, INPUT);
    
    PCattachInterrupt(w1.encA_pin, w1_isr, CHANGE);
    PCattachInterrupt(w1.encB_pin, w1_isr, CHANGE);
    PCattachInterrupt(w2.encA_pin, w2_isr, CHANGE);
    PCattachInterrupt(w2.encB_pin, w2_isr, CHANGE);

    nh.initNode();
    nh.advertise(w1State);
    nh.advertise(w2State);
    nh.subscribe(w1Pwm);
    nh.subscribe(w2Pwm);

    w1_new_enc_state = w1.get_enc_state();
    w2_new_enc_state = w2.get_enc_state();
}


long int prev_micros = micros(),
         curr_micros = micros();


void loop() {
    curr_micros = micros();
	if (curr_micros - prev_micros > DELTA_T) {
        nh.spinOnce();
        enc_state[0].data = pulses_count[0] * enc_state_map[w1_old_enc_state][w1_new_enc_state];
        enc_state[1].data = pulses_count[1] * -enc_state_map[w2_old_enc_state][w2_new_enc_state];
        w1State.publish(&enc_state[0]);
        w2State.publish(&enc_state[1]);

        prev_micros = curr_micros;
		pulses_count[0] = 0;
		pulses_count[1] = 0;
    }
}
