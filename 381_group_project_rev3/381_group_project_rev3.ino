#include <Keypad.h>

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <LiquidCrystal.h>

/**
 * pin
 * acc:
 *  scl -> scl 
 *  sda -> sda
 * ultrosnoic sensor:
 *  ping -> 7
 *  echo -> 6
 * buzzer: 
 *  4
 * Led: 
 *  5
 * lcd:
 *  3, 13, 12, 9, 8, 2
 * Keypad:
 *  A0 A1 A2 A3
 *  A4 A5 10 11
 */

// accelerometer setting
#define mpu_addr 0x68
int16_t acc_x, acc_y, acc_z; // raw data
int16_t cal_x, cal_y, cal_z;

// ultrasonic settings
const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor
long duration, inches, cm;

// counter settings
bool low_flag, high_flag;
float th_upper,th_lower;
int offset = 0;  // 10 cm
int counter = 0;
float altitude;
bool rep_finish = false;
int state = 0;
int up_counter = 0;
int hold_counter = 0;
int down_counter = 0;
int max_up_counter = 0;
int max_hold_counter = 0;
int max_down_counter = 0;
float max_up_timer = 0;
float rep_up_timer = 0;
float tdf = 0;
float sum_tdf = 0;
bool max_up_flag = 0;
bool max_rep_flag = 0;
int rep_count_max = 12;
int uppp_counter = 0;

// set & rest
int set_current_mode = 0;
int max_set = 0;
int set_counter = 0;
float rest_time = 0;
float rest_prepare_timer = 0;
float rest_start_timer = 0;
int rest_flag = 0;

// buzzer settings
int buz_pin1 = 4;
int buz_pin2 = 5;
int up_freq = 500;
int down_freq = 300;
int hold_freq = 200;
int freq_2 = 300;
float bpm = 60;
float general_timer = 0;
float start_timer=0;
float buz1_timer = 0;

float print_delay = 0.5 * 1000000;
float last_timer = 0;

float A_grade = 0;
float B_grade = 0;
float C_grade = 0;

LiquidCrystal lcd(3, 13, 12, 9, 8, 2);

const byte ROWS = 4;
const byte COLS = 4;

bool A_pressed, B_pressed, C_pressed, D_pressed;

char keys[ROWS][COLS] = {
  {'1', '4', '7', '0'},
  {'2', '5', '8', 'F'},
  {'3', '6', '9', 'E'},
  {'A', 'B', 'C', 'D'}
};

byte rowPins[ROWS] = {A0, A1, A2, A3};

byte colPins[COLS] = {A4, A5, 10, 11};

Keypad newKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


boolean menu2 = false;

int A_set, B_set, C_set;
int startTime;
int elapsedTime;

void setup() {
  Serial.begin(115200);
  // ultrasonic sensor
  pinMode(pingPin, OUTPUT);
  // counter
  low_flag = 0;
  high_flag = 0;

  //buzzer settings
  pinMode(buz_pin1, OUTPUT);
  pinMode(buz_pin2, OUTPUT);
  Serial.println("calibate distance");
  lcd.begin(16, 2);
  calibrate();
  Serial.println("ACC setup");
  //Serial.print("Current Raw acc: ");Serial.println(acc_x);
  Serial.println("lcd setup");
  //LCD display, user interface*/
  lcd_setup();
  
  Wire.begin();
  TWBR=12;
  // accelerometer
  acc_setup(); // setup accelerometer
  //acc_read();
  Serial.println("start pushing");
  last_timer = micros();
  buz1_timer = micros();
  A_grade = 0;
  B_grade = 0;
  C_grade = 0;
  state = 1;
}

void lift_buz() {
  // exercise rap count indicator
  if (micros() - buz1_timer >= (1000000*(60/bpm))) {
    if (state == 1) { // up
      tone(buz_pin1, up_freq, 100);
      up_counter++;
    } else if (state == 2) {  // down
      tone(buz_pin1, down_freq, 100);
      down_counter++;
    } else if (state == 3) {// hold
      tone(buz_pin1, hold_freq, 100);
      hold_counter++;
    } else {
      up_counter = 0;
      down_counter = 0;
      hold_counter = 0;
      max_up_flag = 0;
    }
    if (up_counter == max_up_counter && state == 1) {
      state = 2;
      up_counter = 0;
      down_counter = 0;
      max_up_timer = micros();
      max_up_flag = 1;
      uppp_counter++;
    } else if (down_counter == max_down_counter && state == 2) {
      state = 3;
      down_counter = 0;
      hold_counter = 0;
    } else if (hold_counter == max_hold_counter && state == 3) {
      state = 1;
      hold_counter = 0;
      up_counter = 0;
    }
    buz1_timer = micros();
  }
}

void rep_switch() {
  //-----------------------rep-counter---------------------
  if (rest_flag == 0) {
    if (altitude >= th_upper && low_flag == 1) {
      high_flag = 1;
      low_flag = 0;
      counter++;
      digitalWrite(buz_pin2,HIGH);
      rep_up_timer = micros();
      delay(100);
      digitalWrite(buz_pin2,LOW);
      max_rep_flag = 1;
    }
    if (altitude <= th_lower && high_flag == 1) {
      high_flag = 0;
      low_flag = 1;
    }
  }
  //---------------------------end------------------------

  // find time difference between standard and users
  if (max_rep_flag == 1 && max_up_flag == 1) {
    max_rep_flag = 0;
    max_up_flag = 0;
    tdf = abs(rep_up_timer - max_up_timer) / 1000000;
    sum_tdf = sum_tdf + tdf;
    if (set_current_mode == 0) {
      A_grade = A_grade + tdf;
    } else if (set_current_mode == 1) {
      B_grade = B_grade + tdf;
    } else if (set_current_mode == 2) {
      C_grade = C_grade + tdf; 
    }
  }
  lcd.setCursor(8,1);
  lcd.print(tdf);
  // set & rest
  if (rep_count_max == uppp_counter) {      // user finishing one set
    tone(buz_pin1, 392);
    delay(300);
    tone(buz_pin1, 329); 
    delay(300);
    tone(buz_pin1, 261); 
    delay(300);
    noTone(buz_pin1);
    counter = 0;
    uppp_counter = 0;
    rest_start_timer = micros();
    rest_flag = 1;
    state = 0;  // reset rep counter
    set_counter++;
  }
/**
  // if user ddin't select exercise A
  if (A_set == 0 && set_current_mode == 0) {
    set_current_mode = set_current_mode + 1;
  }
  if (B_set == 0 && set_current_mode == 1) {
    set_current_mode = set_current_mode + 1;
  }
  if (C_set == 0 && set_current_mode == 2) {
    set_current_mode = set_current_mode + 1;
  }
*/
  if (set_counter == max_set && set_current_mode == 0) {
    set_current_mode = set_current_mode + 1;
    Serial.println("A finished");
    Serial.println("Start B");
    delay(500);
    set_counter = 0;
    max_set = B_set;
    rep_count_max = 14;
    rest_time = 30; // 30s
    
    max_up_counter = 6;
    max_down_counter = 2;
    max_hold_counter = 1;
  } else if (set_counter == max_set && set_current_mode == 1) {
    set_current_mode = set_current_mode + 1;
    Serial.println("B finished");
    Serial.println("Start C");
    delay(500);
    set_counter = 0;
    max_set = C_set;
    rep_count_max = 8;
    rest_time = 30; // 30s
    
    max_up_counter = 4;
    max_down_counter = 2;
    max_hold_counter = 1;
  } else if (set_counter == max_set && set_current_mode == 2) {
    Serial.println("C finished");
    set_current_mode = set_current_mode + 1;
  }
  
  if (set_current_mode == 3) {
    Serial.println("EXercise finish!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Finished.");
    lcd.setCursor(0, 1);
    lcd.print("TDF: ");
    lcd.print(sum_tdf);

    int n = 3;
    Serial.println(A_grade);
    Serial.println(B_grade);
    Serial.println(C_grade);
    float sum_A_grade = 100 - A_grade;
    float sum_B_grade = 100 - B_grade;
    float sum_C_grade = 100 - C_grade;
    Serial.println();
    if (sum_A_grade == 100) {
      n = n - 1;
      sum_A_grade = 0;
    }
    if (sum_B_grade == 100) {
      n = n - 1;
      sum_B_grade = 0;
    }
    if (sum_C_grade == 100) {
      n = n - 1;
      sum_C_grade = 0;
    }
    float grade = 100;
    if (n != 0) {
      grade = (sum_A_grade + sum_B_grade + sum_C_grade) / n;
    }
    Serial.println();
    Serial.println(n);
    Serial.println(grade);
    lcd.setCursor(12, 1);
    if (grade > 93) {
      lcd.print("A");
    } else if (grade > 90) {
      lcd.print("A-");
    } else if (grade > 87) {
      lcd.print("B+");
    } else if (grade > 85) {
      lcd.print("B-");
    } else if (grade > 80) {
      lcd.print("B");
    } else if (grade > 77) {
      lcd.print("C+");
    } else if (grade > 75) {
      lcd.print("C");
    } else if (grade > 70) {
      lcd.print("C-");
    } else if (grade > 60) {
      lcd.print("D");
    } else {
      lcd.print("F");
    }
    delay(20000);
  }

  if (rest_flag == 1 && micros() - rest_start_timer >= (rest_time - 5) * 1000000) {
    rest_flag = 2;
    rest_prepare_timer = micros();
    tone(buz_pin1, 500, 200);
  }

  if (rest_flag == 2 && micros() - rest_prepare_timer > 0.5*1000000) {
    tone(buz_pin1, 500, 200);
    rest_prepare_timer = micros();
  }

  if (rest_flag == 2 && micros() - rest_start_timer >= rest_time * 1000000) {
    rest_flag = 0;
    delay(1000);
    tone(buz_pin1, 261); 
    delay(300);
    tone(buz_pin1, 329); 
    delay(300);
    tone(buz_pin1, 392);
    delay(300);
    noTone(buz_pin1);
    delay(1000);
    state = 1;
    rest_start_timer = 0; 
  }
}

void loop() {
  lift_buz();
  // calibrate altitude based on angle
   acc_read();
   altitude = get_altitude();
   float sum_acc = sqrt(pow(acc_x,2)+pow(acc_y,2)+pow(acc_z,2));
   float angle = acos((float)acc_z / sum_acc) / 3.1415 * 180;
   float up_angle = angle - 90;
   float up_altitude = altitude;   // not use for now
   if (up_angle > 0) {
     up_altitude = cos(up_angle/180*3.1415) * altitude;
   } else {
     up_altitude = cos(-up_angle/180*3.1415) * altitude;
   }

   up_altitude = altitude;

  if (set_current_mode == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Strength:");
    
  } else if (set_current_mode == 1) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Endurance:");
  } else if (set_current_mode == 2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hypertrophy:");
  }
  lcd.setCursor(0, 1);
  lcd.print(counter);
  lcd.setCursor(5, 1);
  lcd.print(set_counter);
  
  // print out the data
  if (micros() - last_timer > print_delay) {
    Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" cm");
    Serial.print("Up Altitude: "); Serial.print(up_altitude); Serial.println(" cm");
    Serial.print("Up angle: "); Serial.print(up_angle); Serial.println(" degrees");
    //Serial.print("current: ");Serial.println(counter);
    //Serial.print("Time difference(each rep): "); Serial.println(tdf);
    //Serial.print("Current Rep: "); Serial.println(uppp_counter);
    //Serial.print("Current Set: "); Serial.println(set_counter);
    last_timer = micros();
  }
  rep_switch();
}

void lcd_print() {
  char keyPress = newKeypad.getKey();
  if (keyPress) {
    Serial.println((int)keyPress);
  }
if(keyPress) {
switch(keyPress) {
case 'A':
  A_pressed = 1;
  B_pressed = 0;
  C_pressed = 0;
  D_pressed = 0;
 lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("# of sets (0-9)?");
  break;
case 'B':
  B_pressed = 1;
  A_pressed = 0;
  C_pressed = 0;
  D_pressed = 0;
 lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("# of sets (0-9)?");
  break;
case 'C':
  C_pressed = 1;
  B_pressed = 0;
  A_pressed = 0;
  D_pressed = 0;
 lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("# of sets (0-9)?");
  break;
case 'D':
  D_pressed = 1;
  B_pressed = 0;
  C_pressed = 0;
  A_pressed = 0;
 lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Confirmed. ");
  lcd.setCursor(0, 1);
  lcd.print("ST:");lcd.print(A_set);
  lcd.setCursor(6, 1);
  lcd.print("EN:");lcd.print(B_set);
  lcd.setCursor(12, 1);
  lcd.print("HY:");lcd.print(C_set);
  break;
}

if (keyPress >= 48 && keyPress <= 58) {
if (C_pressed || A_pressed || B_pressed) {
    lcd.setCursor(2, 1);
    lcd.print(keyPress);
    if (C_pressed) {
      C_set = keyPress - 48;
    } else if(A_pressed) {
      A_set = keyPress - 48;
    } else if (B_pressed) {
      B_set = keyPress - 48;
    }
  }
}
}
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void calibrate() {
  int N = 200;
  float sum_low = 0, sum_high = 0;
  float average_low, average_high;
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Calibrate: Low");
  lcd.setCursor(0, 1);
  
  Serial.println("\nPlease move to the lowest position");
  for (int i = 0; i < 10; i++) {
    lcd.print(".");
    Serial.print(".");
    delay(200);
  }
  Serial.println("");
  // wait 2 s
  delay(2000);
  //calibrate the lower bound
  for (int i = 0; i < N; i++) {
    altitude = get_altitude();
    sum_low = sum_low + altitude;
    delay(10);
  }
  average_low = sum_low / N;
  Serial.println("Finished.");
  Serial.println("Please move to the upper position");
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Calibrate: HIGH");
  lcd.setCursor(0, 1);
  for (int i = 0; i < 10; i++) {
    lcd.print(".");
    Serial.print(".");
    delay(200);
  }
  Serial.println("");
  //calibrate the upper bound
  for (int i = 0; i < N; i++) {
    altitude = get_altitude();
    sum_high = sum_high + altitude;
    delay(10);
  }
  average_high = sum_high / N;
  Serial.println("Finished");
  delay(1000);
  

  th_upper = int((average_high - offset));
  th_lower = int((average_low + offset));
  Serial.print("Upper Bound: "); Serial.println(th_upper);
  Serial.print("Lower Bound: "); Serial.println(th_lower);
  low_flag = 1;

  // start beep
  tone(buz_pin1, 261); 
  delay(300);
  tone(buz_pin1, 329); 
  delay(300);
  tone(buz_pin1, 392);
  delay(300);
  noTone(buz_pin1);

  delay(1000);
}

float get_altitude() {
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   cm = microsecondsToCentimeters(duration);
  return cm;
}

void lcd_setup() {
  lcd.clear();
  lcd.setCursor(0, 0);

  lcd.print("Select Type");
  lcd.setCursor(0, 1);

  lcd.print("ST:A EN:B HY:C");

  while(!D_pressed) {
    lcd_print();
  }
  Serial.println("USER SETTINGS CONFIRMED.");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Move to low position and ");
  lcd.setCursor(0, 1);
  lcd.print("begin exercise");
  delay(2000);
  Serial.println("Move to low position and begin");
  Serial.print("A_set: ");Serial.println(A_set);
  Serial.print("B_set: ");Serial.println(B_set);
  Serial.print("C_set: ");Serial.println(C_set);
  // user entered rep settings
  rep_count_max = 4;
  max_set = A_set;
  rest_time = 30; // 30s
  
  max_up_counter = 2;
  max_down_counter = 2;
  max_hold_counter = 1;
}


/**
 * Accelerometer reading data
 */
void acc_setup() {
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B); // acc address
  Wire.write(0);
  Wire.endTransmission();
  
  // set at +- 2g
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x1C);
  Wire.write(0); // set +- 2g
  Wire.endTransmission();

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x1D);
  Wire.write(0x0D);  // bandwidth 5Hz
  Wire.endTransmission();

  acc_calibrate();
}

void acc_calibrate() {
  
}

void acc_read() {
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3B); // acc address
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr, 6);
  while(Wire.available() < 6); // wait
  
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
}
