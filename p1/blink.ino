/*

"Readings" vs. "Samples":
  one ultrasonic reading per second;
  save one sample every N_AVG readings (the average of the N_AVG readings);
  send after N_GROUP samples are collected.

  Note: When debugging using a serial monitor, hit RETURN to start receiving
  messages from the Particle Electron. Some "Arduino legacy".
*/

#define DEBUG 0
#define PUBLISH_ENABLED 1
#define N_AVG   (60)  // must match that on the server
#define N_GROUP (10)  // server can infer this

volatile uint8_t chrbuf[4];
volatile uint8_t chrbuf_i;
volatile uint16_t readcount = 0;
volatile uint16_t readings[N_AVG];    // a list of sensor readings
volatile uint16_t samplecount = 0;
volatile double samples[N_GROUP];   // a list of averages
long last_sampled = 0;

int led1 = D7;

void led_on() {
  digitalWrite(led1,HIGH);
}

void led_off() {
  digitalWrite(led1,LOW);
}

void sensor_on() {
  digitalWrite(D1,HIGH);
}

void sensor_off() {
  digitalWrite(D1,LOW);
}

double mean(volatile uint16_t* samples,const uint16_t length) {
	double sum = 0;
	for (uint16_t i = 0; i < length; i++) {
		sum += samples[i];
	}
	return sum/length;
}

void setup() {
  pinMode(led1,OUTPUT);
  pinMode(D1,OUTPUT);
  sensor_off();

  Serial.begin();
  Serial1.begin(9600,SERIAL_8N1);

  //Cellular.off();
  //RGB.control(true);
  //RGB.color(0,0,0);
  //RGB.brightness(0);
}

int fsm(uint8_t c) {
	if ('R' == c) {
		chrbuf_i = 0;
	} else if ('\r' == c) {
		if (readcount < N_AVG) {
			uint16_t tmp = (chrbuf[0] - '0')*1000 + (chrbuf[1] - '0')*100 + (chrbuf[2] - '0')*10 + (chrbuf[3] - '0');
			readings[readcount++] = tmp;
		}
		chrbuf[0] = 0;
		chrbuf[1] = 0;
		chrbuf[2] = 0;
		chrbuf[3] = 0;
	} else {
		if (chrbuf_i < sizeof(chrbuf)) {
			if ((c >= '0') && (c <= '9')) {
				chrbuf[chrbuf_i++] = c;
			} else {
				chrbuf_i = 0;	// invalid character
			}
		} else {
			chrbuf_i = 0;
		}
	}
}

void serialEvent1() {
  //char x = Serial1.read();
  //Serial.println(x);
  fsm(Serial1.read());
}

void loop() {
  //if (!Serial1.available()) {
    Particle.process(); // not needed except in MANUAL mode...
  //}

  /*if ((Time.hour() == 0) && (Time.minute() == 0) && (Time.second() == 0)) {
    Particle.syncTime();
  }*/
  /*if (Particle.timeSyncedLast() > 24*3600*1000) {
    Particle.syncTime();
    waitUntil(Particle.syncTimeDone);
  }*/

  if (Time.now() - last_sampled >= 1) {
    #if DEBUG
    led_on();
    Serial.println("ping");
    #endif
    sensor_on();  // TODO: put this in Timer
    //delay(1);   // ms
    delayMicroseconds(100);
    sensor_off();
    #if DEBUG
    led_off();
    #endif
    last_sampled = Time.now();
  }

  if (readcount >= N_AVG) {
    #if DEBUG
    led_on();
    Serial.println("\taverage");
    #endif
    samples[samplecount++] = mean(readings,N_AVG);
    //Serial.println(avg);
    //#ifndef DEBUG
    //Particle.publish("d",String(avg));
    //#endif
    readcount = 0;
    #if DEBUG
    led_off();
    #endif
  }

  if (samplecount >= N_GROUP) {
    #if DEBUG
    Serial.println("\t\tsend");
    #endif
    led_on();
    String msg = "";
    for (int i = 0; i < samplecount; i++) {
      msg += String(samples[i],1);
      if (i < samplecount - 1) {
        msg += ",";
      }
    }
    Serial.println(msg);
    #if PUBLISH_ENABLED
    Particle.publish("d2w",msg);
    #endif
    samplecount = 0;
    led_off();
  }
}

/*void loop() {
  if (Serial.available()) {
    digitalWrite(led1,HIGH);
    int inByte = Serial.read();
    Serial1.write(inByte);
    digitalWrite(led1,LOW);
  }
  if (Serial1.available()) {
    digitalWrite(led1,HIGH);
    int inByte = Serial1.read();
    Serial.write(inByte);
    digitalWrite(led1,LOW);
  }
}*/

/*void loop() {
  Particle.process();

  Serial.println("test");
  digitalWrite(led1, HIGH);
  delay(100);

  Serial.println("test");
  digitalWrite(led1, LOW);
  delay(100);
}*/
