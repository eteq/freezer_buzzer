#include <CapacitiveSensor.h>

#define FREEZER_PIN 8 // HIGH is open
#define FRIDGE_PIN 9 // HIGH is open
#define PIEZO_PIN 10
#define PIEZO_GND_PIN 12

#define FREEZER_WAIT_MS 20000
#define FRIDGE_WAIT_MS 20000

#define FREEZER_FREQ_1 300
#define FREEZER_FREQ_2 240
#define FRIDGE_FREQ_1 200
#define FRIDGE_FREQ_2 160

#define CYCLEMS 1000
#define ALARM_TIME_MS 3000
#define ALARM_WAIT 2000

#define PAUSE_MS 120000 
#define BUTTON_PIN -1 // if negative, use capacitive sensing instead
#define BUTTON_GND_PIN 1
#define CAP_SENS_SEND_PIN 2
#define CAP_SENS_RECV_PIN 7
#define CS_THRESH 1000

CapacitiveSensor cs = CapacitiveSensor(CAP_SENS_SEND_PIN, CAP_SENS_RECV_PIN);


long last_button_press;
unsigned long freezer_last_closed_millis = 0 , fridge_last_closed_millis = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  pinMode(FREEZER_PIN, INPUT_PULLUP);
  pinMode(FRIDGE_PIN, INPUT_PULLUP);
  
  pinMode(PIEZO_PIN, OUTPUT);
  digitalWrite(PIEZO_PIN, LOW);
  
  pinMode(PIEZO_GND_PIN, OUTPUT);
  digitalWrite(PIEZO_GND_PIN, LOW);
  
  if (BUTTON_PIN > 0) {
    pinMode(BUTTON_GND_PIN, OUTPUT);
    digitalWrite(BUTTON_GND_PIN, LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, LOW);
  } else { 
    cs.set_CS_Timeout_Millis(500);
    cs.reset_CS_AutoCal();
  }
  last_button_press = millis() - PAUSE_MS - 1;

  Serial.println("Setup complete.");
}

void loop() {
  unsigned long currentMillis = millis();

  if (BUTTON_PIN < 0) {
    // capacitive sensing mode
    long csVal = cs.capacitiveSensor(10);
    if ((csVal > CS_THRESH) || (csVal == -2)) {
      Serial.print("capacitive sensor triggered.  Level: ");
      Serial.println(csVal);
      //buttonPressed();
      last_button_press = currentMillis;
    } 
  }
  
  if ((currentMillis - last_button_press) < PAUSE_MS) {
    Serial.print("Pausing, ");
    Serial.print(currentMillis - last_button_press);
    Serial.println("msec since.");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }
  
  if (digitalRead(FREEZER_PIN) == HIGH) {
    // freezer is open
    Serial.println("freezer open");
    blink(LED_BUILTIN, 20, 3);
    if ((currentMillis - freezer_last_closed_millis) > FREEZER_WAIT_MS) {
    Serial.println("freezer alarm!");
      // freezer alarm!
      twoTone(PIEZO_PIN, FREEZER_FREQ_1, FREEZER_FREQ_2, CYCLEMS, ALARM_TIME_MS, FREEZER_PIN);
      if (ALARM_WAIT) {
        delay(ALARM_WAIT);
      }
    }
  } else {
    Serial.println("freezer closed");
    freezer_last_closed_millis = currentMillis;
  }
  
  if (digitalRead(FRIDGE_PIN) == HIGH) {
    // fridge is open
    Serial.println("fridge open");
    blink(LED_BUILTIN, 60, 1);
    if ((currentMillis - fridge_last_closed_millis) > FRIDGE_WAIT_MS) {
      // fridge alarm!
    Serial.println("fridge alarm!");
      twoTone(PIEZO_PIN, FRIDGE_FREQ_1, FRIDGE_FREQ_2, CYCLEMS, ALARM_TIME_MS, FRIDGE_PIN);
      if (ALARM_WAIT) {
        delay(ALARM_WAIT);
      }
    }
  } else {
    Serial.println("fridge closed");
    fridge_last_closed_millis = currentMillis;
  }
}

void buttonPressed() {
  last_button_press = millis();
}

void twoTone(uint8_t pin, unsigned int freq1, unsigned int freq2, unsigned long cyclems, unsigned long totalms, int pin_to_check_for_low) {
  unsigned long cycleStart, start, halfcyclems;
  int ontone = 0;
  
  halfcyclems = cyclems/2;
  
  start = millis();
  cycleStart = start-cyclems-1;  // make the first one always trigger tone 1
  while ((millis() - start) < totalms) {
    if (pin_to_check_for_low>0) {
      if (digitalRead(pin_to_check_for_low) == LOW) {
        break;
      }
    }
    unsigned long dcycle = millis() - cycleStart;
    if (dcycle > cyclems) {
      ontone = 1;
      tone(pin, freq1);
      cycleStart = millis();
    } else if ((dcycle > halfcyclems) && (ontone < 2)) {
      tone(pin, freq2);
      ontone = 2;
    }
  }
  noTone(pin);
}

void blink(uint8_t ledpin, unsigned long cyclems, unsigned int ntimes) {
  for (int i=0;i < ntimes;i++) {
    digitalWrite(ledpin, HIGH);
    delay(cyclems/2);
    digitalWrite(ledpin, LOW);
    delay(cyclems/2);
  }
}
