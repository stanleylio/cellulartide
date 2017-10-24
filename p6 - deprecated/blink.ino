/*
What's new:

Deep sleep means:
Long battery life
Low sampling rate
No more OTA update
... and GPIOs go into high-Z, so need hardware mod.

Stanley H.I. Lio
hlio@hawaii.edu
University of Hawaii, 2017
*/
#include "Adafruit_BME280.h"

#define PUBLISH_ENABLED (1)
#define SAMPLE_INTERVAL_SECOND  (5*60)  // sampling takes an additional 1 min...
#define N_AVG           (60)
#define N_GROUP         (1)  // make sure the overall message length stays under 255 chars!
#define HAS_BME280      (0)

//https://docs.particle.io/support/troubleshooting/mode-switching/electron/
//SYSTEM_MODE(SEMI_AUTOMATIC);  // !!

volatile uint16_t readings[N_AVG];  // a list of sensor readings
volatile uint16_t readings_i = 0;
typedef struct sample_t {
   uint32_t ts;
   double d2w;
} sample;
volatile sample_t samples[2*N_GROUP];
volatile uint16_t samples_i = 0;
FuelGauge fuel;
volatile bool sample_flag = false;

//int usen = D1;  // EN pin of ultrasonic sensor; v0.1 and prior (conflict with I2C)
int usen = D2;  // EN pin of ultrasonic sensor; v0.2 board
int dummy = D1;
void sensor_on() {  digitalWrite(usen,HIGH);}
void sensor_off() {  digitalWrite(usen,LOW);}

void ustrigger() {
  sensor_on();
  delayMicroseconds(500);
  sensor_off();
}

void clocksync() {
  if (!Particle.syncTimePending()) {
    Serial.println("clocksync() begin");
    //Particle.connect();
    Particle.syncTime();
    waitUntil(Particle.syncTimeDone);
    Serial.println("clocksync() end");
  } else {
    Serial.println("clocksync() in progress");
  }
}

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

/*void serialEvent1() {
  //char x = Serial1.read();
  //Serial.println(x);
  uint16_t r = 0;
  if (fsm(Serial1.read(),&r)) {
//    Serial.println(r);
    latest_reading = r;
  }
}*/

double mean(volatile uint16_t* s,const uint16_t length) {
	double sum = 0;
	for (uint16_t i = 0; i < length; i++) {
		sum += s[i];
	}
	return sum/length;
}

void setup() {
  pinMode(dummy,INPUT_PULLUP);
  pinMode(usen,OUTPUT);
  sensor_off();

  Serial.begin();
  Serial1.begin(9600,SERIAL_8N1);

  clocksync();

  //Cellular.off();
//  RGB.control(true);
//  RGB.color(0,0,0);
//  RGB.brightness(0);
}


void set_sample_flag() {
  sample_flag = true;
}

/*void loop() {
  System.sleep(SLEEP_MODE_DEEP,120);
}*/

void loop() {
  readings_i = 0;
  Timer t60(1000,set_sample_flag,false);
  t60.start();

  while (true) {
    if (sample_flag) {
      sample_flag = false;

      ustrigger();
      uint16_t r = 0;
      for (uint8_t i = 0; i < 250; i++) {
        if (Serial1.available()) {
          if (fsm(Serial1.read(),&r)) {
            if ((r >= 300) && (r < 5000)) {
              if (readings_i < N_AVG) {
                readings[readings_i++] = r;
                Serial.println(String(r));
              }
            } else {
              Serial.println("nope");
            }
            break;
          }
        } else {
          delay(1);
    // so if sensor is disconnected, wait max 250ms and retry on next loop()
        }
      }
    }

    if (readings_i >= N_AVG) {
      t60.stop();
      break;
    }
  }

  // calculate mean and store as one sample
  if ((readings_i >= N_AVG) && (samples_i < N_GROUP)) {
    samples[samples_i].ts = Time.now();
    samples[samples_i].d2w = mean(readings,N_AVG);
    samples_i++;
    readings_i = 0;
    for (int i = 0; i < N_AVG; i++) {  // optional.
      readings[i] = 0;
    }
  }

  // format the list of samples into a string of the form
  //  [[t1,d1],[t2,d2]...]
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

    // print/publish
    Serial.println(msg);

    String bmemsg = "";
    #if HAS_BME280
    Adafruit_BME280 bme;
    if (bme.begin(0x76)) {
      bmemsg = ",\"t\":" + String(bme.readTemperature(),2) + ",\"p\":" + String(bme.readPressure()/100.0F,2) + ",\"rh\":" + String(bme.readHumidity(),1);
    }
    #endif

    String debugmsg = "{\"Timestamp\":" + String(Time.now()) + ",\"VbattV\":" + String(fuel.getVCell(),3) + bmemsg + "}";
    Serial.println(debugmsg);

    #if PUBLISH_ENABLED
    //Particle.connect();
    Particle.publish("d2w",msg);
    Particle.publish("debug",debugmsg);
    Serial.println("Sent.");
    #endif

    samples_i = 0;
  }

  // sync the internal real-time clock
  if (millis() - Particle.timeSyncedLast() > 24*60*60*1000) {
    clocksync();
  }

  //delay(SAMPLE_INTERVAL_SECOND);
  //System.sleep(dummy,RISING,SLEEP_NETWORK_STANDBY,SAMPLE_INTERVAL_SECOND);
  // A bug apparently. System.sleep() does not wait for Particle.publish() to finish.
  for (int i = 0; i < 100; i++) {
    Particle.process();
    delay(100);
  }
  System.sleep(SLEEP_MODE_DEEP,SAMPLE_INTERVAL_SECOND);

  // IO pins go high-Z during deep sleep, WTH?

  // premature-optimization: queue up data at interval N and transmit during
  // the cellular init in interval N+1. saves a few seconds of Tx + wait time
  // since it has to wait for the 60 ultrasonic samples to come in anyway.
}
