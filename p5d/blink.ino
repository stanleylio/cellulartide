/*
What's new:

Report the # of valid samples in the group as sample_size.
At every 1-second cycle, retry max 4 times. It is marked 0 and excluded in mean
calculation if all four reads failed.

uC does not sleep. Cellular radio is always on so OTA update is possible.
RTC is sycned every 24hr.

Message format changed to include sample time (event name = d2w). Example:
  [[1497818212,1333.8],[1497818272,1333.8],[1497818332,1333.7],[1497818392,1333.5],[1497818452,1333.7],[1497818512,1334.2],[1497818572,1378.3],[1497818632,1334.3],[1497818692,1334.4],[1497818752,1334.5]]

Stanley H.I. Lio
hlio@hawaii.edu
University of Hawaii, 2017
*/

#include "Adafruit_BME280.h"

#define PUBLISH_ENABLED 1
#define N_AVG           (60)
#define N_GROUP         (10)  // make sure the overall message length stays under 255 chars!
#define HAS_BME280      0

int latest_reading = 0;
volatile uint16_t readings[N_AVG];  // a list of sensor readings
volatile uint16_t readings_i = 0;
typedef struct sample_t {
   uint32_t ts;           // POSIX timestamp
   double d2w;            // [300,5000)
   uint8_t sample_size;   // max 60
} sample;
volatile sample_t samples[2*N_GROUP];
volatile uint16_t samples_i = 0;
FuelGauge fuel;
volatile int last_sampled = 0;


//int usen = D1;  // EN pin of ultrasonic sensor; v0.1 and prior (conflict with I2C)
int usen = D2;  // EN pin of ultrasonic sensor; v0.2 board
void sensor_on() {  digitalWrite(usen,HIGH);}
void sensor_off() {  digitalWrite(usen,LOW);}

//https://docs.particle.io/support/troubleshooting/mode-switching/electron/
SYSTEM_MODE(SEMI_AUTOMATIC);  // !!

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

void mean(volatile uint16_t* s,const uint16_t length, double *d2w, uint8_t *sample_size) {
	*sample_size = 0;
	double sum = 0;
	for (uint8_t i = 0; i < length; i++) {
		if ((s[i] >= 300) && (s[i] < 5000)) {
			sum += s[i];
			(*sample_size) += 1;
		}
	}
	if (*sample_size) {
		*d2w = sum/(*sample_size);
	} else {
		*d2w = 0;
	}
}

void setup() {
  pinMode(usen,OUTPUT);
  sensor_off();

  Serial.begin();
  Serial1.begin(9600,SERIAL_8N1);

  Particle.variable("d2w", latest_reading);

  clocksync();

  //Cellular.off();
  RGB.control(true);
  RGB.color(0,0,0);
  RGB.brightness(0);
}

void loop() {
  int ct = Time.now();
  if (ct - last_sampled >= 1) {

    bool goodread = false;
    uint16_t r = 0;

    for (uint8_t retry = 0; (retry < 4) && (!goodread); retry++) {
      sensor_on();
      delayMicroseconds(200);
      sensor_off();
      for (uint8_t tick = 0; tick < 200; tick++) {
        if (Serial1.available()) {
          if (fsm(Serial1.read(),&r)) {
            if ((r >= 300) && (r < 5000)) {
              goodread = true;
              break;
            } else {
              // reading out of range
            }
          }
        }
        delay(1);	// so max ~200ms wait
      }
    }

    if (readings_i < N_AVG) {
      if (goodread) {
Serial.println("good");
        readings[readings_i++] = r;
        latest_reading = r;
      } else {
Serial.println("bad");
        readings[readings_i++] = 0;		// 0 means invalid reading
        latest_reading = 0;
      }
    }

    last_sampled = ct;
  }

  // calculate mean and store as one sample
  if ((readings_i >= N_AVG) && (samples_i < N_GROUP)) {
    double d2w;
    uint8_t sample_size = 0;
    mean(readings,N_AVG,&d2w,&sample_size);
    Serial.println(String(samples_i) + "," + String(d2w) + "," + String(sample_size));
    samples[samples_i].ts = Time.now();
    samples[samples_i].d2w = d2w;
    samples[samples_i].sample_size = sample_size;
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
      tmp += ",";
      tmp += String(samples[i].sample_size);
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

    String debugmsg = "{\"Timestamp\":" + String(Time.now()) + ",\"VbattV\":" + String(fuel.getVCell(),3) + ",\"SoC\":" + String(fuel.getSoC(),2) + bmemsg + "}";
    Serial.println(debugmsg);

    #if PUBLISH_ENABLED
    Particle.connect();
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
}
