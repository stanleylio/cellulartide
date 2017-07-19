/*
What's new:
Ultrasonic sensor's output is no longer parsed and stored in serialEvent1. Main
loops will fetch and store the latest measurement every second, regardless of
whether the sensor is being switched or not. That means if US_EN is
disconnected / wired to the wrong pin and sensor is running at 6Hz, measurements
are still taken at 1Hz.

Readings of 5000 are now rejected and sensor is immediately triggered again for
another measurement.

uC does not sleep. Cellular radio is always on so OTA update is possible.

Message format changed to include sample time (event name = d2w). Example:
  [[1497818212,1333.8],[1497818272,1333.8],[1497818332,1333.7],[1497818392,1333.5],[1497818452,1333.7],[1497818512,1334.2],[1497818572,1378.3],[1497818632,1334.3],[1497818692,1334.4],[1497818752,1334.5]]

Power report via FuelGauge (event name = debug):
  {"VbattV":4.066250}

BME280, if enabled, would be read and sent along with voltage report.

RTC is sycned every 24hr.

particle compile electron . --saveTo e && particle flash --serial e

https://community.particle.io/t/electron-sleep-mode-deep-tips-and-examples/27823

Sensor is polled every second. A sample is calculated and stored from the sample
mean of N_AVG measurements.
Samples are transmitted in one batch When N_GROUP samples are collected.

- - -
Design decision:

"Send when N-minute worth of samples are collected" vs. "send at 0, N, 2N, 3N... minute of the hours":
The latter is easier to code, but bad for the server when all sensors try to send at the same time.

Note: When debugging using a serial monitor, hit RETURN to start receiving
messages from the Particle Electron. Some "Arduino legacy".

TODO: watchdog

Stanley H.I. Lio
hlio@hawaii.edu
University of Hawaii, 2017
*/

#include "Adafruit_BME280.h"

#define PUBLISH_ENABLED 1
#define N_AVG           (60)
#define N_GROUP         (10)  // make sure the overall message length stays under 255 chars!
#define HAS_BME280      0

volatile uint16_t readings[N_AVG];  // a list of sensor readings
volatile uint16_t readings_i = 0;
typedef struct sample_t {
   uint32_t ts;
   double d2w;
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
//SYSTEM_MODE(SEMI_AUTOMATIC);  // !!

void ustrigger() {
  sensor_on();
  delayMicroseconds(500);
  sensor_off();
}

void clocksync() {
  if (!Particle.syncTimePending()) {
    Serial.println("clocksync() begin");
//    Particle.connect();
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
  pinMode(usen,OUTPUT);
  sensor_off();

  Serial.begin();
  Serial1.begin(9600,SERIAL_8N1);

  clocksync();

  //Cellular.off();
  RGB.control(true);
  RGB.color(0,0,0);
  RGB.brightness(0);
}

void loop() {
  int ct = Time.now();
  if (ct - last_sampled >= 1) {
    ustrigger();
    uint16_t r = 0;
    for (uint8_t i = 0; i < 250; i++) {
      if (Serial1.available()) {
        if (fsm(Serial1.read(),&r)) {
          if ((r >= 300) && (r < 5000)) {
            if (readings_i < N_AVG) {
              readings[readings_i++] = r;
              last_sampled = ct;
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
