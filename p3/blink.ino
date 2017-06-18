/*
Message format changed to include sample time.
Power report via FuelGauge (todo)
RTC is sycned once a day.

particle compile electron . --saveTo e && particle flash --serial e

https://community.particle.io/t/electron-sleep-mode-deep-tips-and-examples/27823

So there are
  measure interval
  average interval
  transmit interval

Send when 10-minute worth of samples are collected vs. send at 0, 15, 30, 45 of the hours:
the latter is easier to code, but bad for the server when all sensors try to send at the same time

"Readings" vs. "Samples":
  one ultrasonic reading per second;
  save one sample every N_AVG readings (the average of the N_AVG readings);
  send after N_GROUP samples are collected.

  Note: When debugging using a serial monitor, hit RETURN to start receiving
  messages from the Particle Electron. Some "Arduino legacy".
*/

#define PUBLISH_ENABLED 1
#define N_AVG   (60)  // server no longer makes assumption about this
#define N_GROUP (10)  // make sure the total message length stay under 255 chars!

volatile uint16_t readings[N_AVG];  // a list of sensor readings
volatile uint16_t readings_i = 0;

//volatile double samples[N_GROUP];   // a list of averages
typedef struct sample_t {
   uint32_t ts;
   double d2w;
} sample;
volatile sample_t samples[2*N_GROUP];
volatile uint16_t samples_i = 0;
FuelGauge fuel;

int led1 = D7;
int usen = D1;  // EN pin of ultrasonic sensor

void led_on() {  digitalWrite(led1,HIGH);}
void led_off() {  digitalWrite(led1,LOW);}
void sensor_on() {  digitalWrite(D1,HIGH);}
void sensor_off() {  digitalWrite(D1,LOW);}

SYSTEM_MODE(SEMI_AUTOMATIC);  // !!

void ustrigger() {
  sensor_on();
  delayMicroseconds(100);
  sensor_off();
}

void clocksync() {
  if (!Particle.syncTimePending()) {
    Serial.println("clocksync() begin");
    Particle.connect();
    Particle.syncTime();
    waitUntil(Particle.syncTimeDone);
    Serial.println("clocksync() end");
  } else {
    Serial.println("clocksync() in progress");
  }
}

Timer timer1(1000,ustrigger);
Timer timer2(24*60*60*1000,clocksync);

bool fsm(uint8_t c, uint16_t * r) {
  static uint8_t chrbuf_i;
  static uint8_t chrbuf[4];

	if ('R' == c) {
    chrbuf_i = 0;
    chrbuf[0] = 0;  // optional.
		chrbuf[1] = 0;
		chrbuf[2] = 0;
		chrbuf[3] = 0;
	} else if ('\r' == c) {
    *r = (chrbuf[0] - '0')*1000 + (chrbuf[1] - '0')*100 + (chrbuf[2] - '0')*10 + (chrbuf[3] - '0');
    return true;
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
  return false;
}

void serialEvent1() {
  //char x = Serial1.read();
  //Serial.println(x);
  uint16_t r = 0;
  if (fsm(Serial1.read(),&r)) {
//    Serial.println(r);
    if ((readings_i < N_AVG) && (r >= 300) && (r <= 5000)) {
      readings[readings_i++] = r;
    }
  }
}

double mean(volatile uint16_t* s,const uint16_t length) {
	double sum = 0;
	for (uint16_t i = 0; i < length; i++) {
		sum += s[i];
	}
	return sum/length;
}

void setup() {
  pinMode(led1,OUTPUT);
  pinMode(usen,OUTPUT);
  sensor_off();

  Serial.begin();
  Serial1.begin(9600,SERIAL_8N1);

  clocksync();

  //Cellular.off();
  RGB.control(true);
  RGB.color(0,0,0);
  RGB.brightness(0);

  timer1.start();
  timer2.start();
}

void loop() {
  if (readings_i >= N_AVG) {
    samples[samples_i].ts = Time.now();
    samples[samples_i].d2w = mean(readings,N_AVG);
    samples_i++;
    readings_i = 0;
    for (int i = 0; i < N_AVG; i++) {  // optional.
      readings[i] = 0;
    }
  }

// [[t1,d1],[t2,d2]]
  if (samples_i >= N_GROUP) {
    String msg = "[";
    for (int i = 0; i < samples_i; i++) {

      String tmp = "[";
      tmp += String(samples[i].ts);
      tmp += ",";
      tmp += String(samples[i].d2w,1);
      tmp += "]";

      msg += tmp;
      if (i < samples_i - 1) {
        msg += ",";
      }
    }
    msg += "]";

    Serial.println(msg);

    #if PUBLISH_ENABLED
    Serial.println("Connecting...");
    Particle.connect();
    if (Particle.connected()) {
      Serial.println("Connected.");
      Particle.publish("d2w",msg);
      Particle.publish("debug","{\"VbattV\":" + String(fuel.getVCell()) +"}");
      Serial.println("Sent.");
      Particle.disconnect();
      Cellular.off();
    }
    #endif
    samples_i = 0;
  }

  if (millis() - Particle.timeSyncedLast() > 24*60*60*1000) {
    clocksync();
  }

  //Particle.process();
  // https://docs.particle.io/reference/firmware/electron/
  //System.sleep(1);
  //Cellular.on();
  System.sleep(N_AVG);
}
