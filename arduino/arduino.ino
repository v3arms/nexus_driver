#include <MotorWheel.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>


irqISR(irq1, isr1);
MotorWheel w1(9, 8, 4, 5, &irq1);
irqISR(irq2, isr2);
MotorWheel w2(10, 11, 6, 7, &irq2);


int i = 0;
char spd[3] = {};


void setup() {
	Serial.begin(19200, SERIAL_8N1);
	TCCR1B = TCCR1B & 0xf8 | 0x01;
	TCCR2B = TCCR2B & 0xf8 | 0x01;
	w1.PIDEnable(KC, TAUI, TAUD, 30);
	w2.PIDEnable(KC, TAUI, TAUD, 30);
	Serial.println("Hello");
}


void loop() {
	w1.PIDRegulate();
    w2.PIDRegulate();

    if (Serial.available() >= 2) {
		Serial.println("Received!");
        Serial.readBytes(spd, 2);

		//w1.setSpeedMMPS(100), w2.setSpeedMMPS(-100);
		// w1.setSpeedMMPS(-100), w2.setSpeedMMPS(100), i = 0;
		
        if (spd[0] == 0 && spd[1] == 0) 
            w1.setSpeedMMPS(0), w2.setSpeedMMPS(0);
        if (spd[0] == 0 && spd[1] != 0) 
            w1.setSpeedMMPS(spd[1]), w2.setSpeedMMPS(spd[1]);
        if (spd[0] != 0 && spd[1] == 0)
            w1.setSpeedMMPS(spd[0]), w2.setSpeedMMPS(-spd[0]);
		
    }

	//if (Serial.availableForWrite() >= 8 && i % 1000000 == 0) {
		/*w1spd = w1.getSpeedMMPS();
		w2spd = w2.getSpeedMMPS();

		if (w1spd * w2spd >= 0)
			cur_spd[0] = 0, cur_spd[1] = w1spd;
		else
			cur_spd[0] = w1spd, cur_spd[1] = 0;
		
		Serial.write((char*)cur_spd, 8); */
	//	Serial.print(spd[0]);
	//	Serial.print(" ");
	//	Serial.println(spd[1]);
	//}
}

/*
void loop() {
	w1.PIDRegulate();
	w2.PIDRegulate(); 

	if (Serial.available()) {
		cmd = Serial.read();
		switch(cmd) {
		case 'w' :
			w1.setSpeedMMPS(65, DIR_ADVANCE);
			w2.setSpeedMMPS(65, DIR_BACKOFF);
			Serial.println("Forward"); 
			break;
		case 'a' :
			w1.setSpeedMMPS(65, DIR_BACKOFF);
			w2.setSpeedMMPS(65, DIR_BACKOFF);
			Serial.println("Left"); 
			break;
		case 'd' :
			w1.setSpeedMMPS(65, DIR_ADVANCE);
			w2.setSpeedMMPS(65, DIR_ADVANCE);
			Serial.println("Right"); 
			break;
		case ' ' :
			w1.setSpeedMMPS(0);
			w2.setSpeedMMPS(0);
			Serial.println("Stop"); 
			break;
		case 's' :
			w1.setSpeedMMPS(100, DIR_BACKOFF);
			w2.setSpeedMMPS(100, DIR_ADVANCE);
			Serial.println("Back"); 
			break;
		}
	}
	
}
*/
