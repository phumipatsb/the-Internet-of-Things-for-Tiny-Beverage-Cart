#include <Keypad_I2C.h>
#include <Keypad.h>
#include <Wire.h>
#include <SPI.h>
#include <SharpIR.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define I2CADDR 0x21  // กำหนด Address ของ I2C

String pad;
char keypressed;
const byte ROWS = 4;  // กำหนดจำนวนของ Rows
const byte COLS = 4;  // กำหนดจำนวนของ Columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {0, 1, 2, 3}; // เชื่อมต่อกับ Pin แถวของปุ่มกด
byte colPins[COLS] = {4, 5, 6, 7}; // เชื่อมต่อกับ Pin คอลัมน์ของปุ่มกด
Keypad_I2C keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS, I2CADDR, PCF8574 );



float Kp = 5, Ki = 0.0, Kd = 0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[5] = {0, 0, 0, 0, 0};
int initial_motor_speed = 255;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

const int buzzer = 4;
int buttonState = 0;

#define sw 54 //BT
#define Infrared_Sensor 56
int myArray[6];

#define  ena 3
#define  enb 13
#define  in1 5
#define  in2 7
#define  in3 11
#define  in4 9
#include <FastLED.h>
#define LED_PIN     2
#define NUM_LEDS    10
CRGB leds[NUM_LEDS];
SharpIR SharpIR_A(Infrared_Sensor, 1080);   //กำหนดรุ่นให้ตรงกับที่ใช้งาน

//communication between board
#include <SoftwareSerial.h>
SoftwareSerial nodemcu(62, 64);

long int data;
int firstVal, secondVal, thirdVal;
int forthVal, fifthVal, sixVal, sevenVal, eightVal, nineVal, tenVal, elevenVal,
    twelthVal, thirteen;
String myString; // complete message from arduino, which consistors of snesors data
char rdata; // received charactors
String cdata; // complete data

void setup()
{
  Serial.begin(9600);
  nodemcu.begin(9600);
  Wire.begin();  // เรียกการเชื่อมต่อ Wire
  keypad.begin( makeKeymap(keys) );  // เรียกกาเชื่อมต่อ
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(3, OUTPUT); //PWM Pin 1
  pinMode(13, OUTPUT); //PWM Pin 2
  pinMode(5, OUTPUT); //Left Motor Pin 1
  pinMode(7, OUTPUT); //Left Motor Pin 2
  pinMode(11, OUTPUT); //Right Motor Pin 1
  pinMode(8, OUTPUT); //Right Motor Pin 2
  pinMode (Infrared_Sensor, INPUT);
  pinMode(sw, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);


  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  OLED.display();
  OLED.clearDisplay();               //Clear display
  OLED.setTextColor(WHITE);          //Set text color
  OLED.setCursor(15, 20);              //Set display start position
  OLED.setTextSize(3);               //Set text size x2
  OLED.println("MY CAR"); // Show result value
  OLED.display();

  ledloed();
  ledoff();

  tone(buzzer, 1047 ); // Send 1KHz sound signal...
  delay(250);        // ...for 1 sec
  tone(buzzer, 1175 ); // Send 1KHz sound signal...
  delay(250);        // ...for 1 sec
  tone(buzzer, 1319 ); // Send 1KHz sound signal...
  delay(250);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(1000);        // ...for 1sec

  OLED.clearDisplay();
  OLED.display();

}

void loop()
{
 
  if (nodemcu.available() == 0 )
  {
    choose_table();
    
  }
  if (nodemcu.available() > 0 ) 
  {
    rdata = nodemcu.read();
    myString = myString + rdata;
    if ( rdata == '\n') {
      String l = getValue(myString, ',', 0);
      String m = getValue(myString, ',', 1);
      String n = getValue(myString, ',', 2);

      String t = getValue(myString, ',', 3);
      String y = getValue(myString, ',', 4);
      String u = getValue(myString, ',', 5);
      String i = getValue(myString, ',', 6);
      String o = getValue(myString, ',', 7);
      String p = getValue(myString, ',', 8);
      String z = getValue(myString, ',', 9);
      String x = getValue(myString, ',', 10);
      String c = getValue(myString, ',', 11);
      String a = getValue(myString, ',', 12);

      firstVal = l.toInt(); // for left and right1
      secondVal = m.toInt(); // forward and reverse2
      thirdVal = n.toInt(); // speed
      forthVal = t.toInt();


      fifthVal  = y.toInt();
      sixVal    = u.toInt();
      sevenVal  = i.toInt();
      eightVal  = o.toInt();
      nineVal   = p.toInt();//3
      tenVal    = z.toInt();//4
      elevenVal = x.toInt();//5
      twelthVal = c.toInt();//6
      thirteen  = a.toInt();//7
      
      myString = "";
      smartcar2();
    }
  }

}
void carForward() {
  while (forthVal) {
    analogWrite(ena, thirdVal);
    analogWrite(enb, thirdVal);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
}
void carBackward() {
  while (fifthVal) {
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
}
void carLeft() {
  while (sixVal) {
    analogWrite(ena, thirdVal);
    analogWrite(enb, thirdVal);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
}
void carRight() {
  while (sevenVal) {
    analogWrite(ena, thirdVal);
    analogWrite(enb, thirdVal);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
}
void carStop() {
  while (eightVal) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
}
void smartcar2() {
  if (forthVal == 13) {
    carForward();
  }
  if (fifthVal == 14) {
    carBackward();
  }
  else if (sixVal == 15) {
    carLeft();
  }
  else if (sevenVal == 16) {
    carRight();
  }
  else if (eightVal == 17) {
    carStop();
  }

}
void tacksp()
{

  while (digitalRead(55) == 1  || digitalRead(63) == 1 )
  {
    int distance = SharpIR_A.distance() + 1;
    Serial.println(distance);
    if (distance <= 40 )
    {
      stop_bot(2000);
    }
    else if (distance >= 41 )
    {
      tack ();
    }
  }
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(1000);

}
void tack()
{
  read_sensor_values();
  calculate_pid();
  motor_control();
}
void read_sensor_values()
{
  sensor[0] = digitalRead(63); //IR63ขวา
  sensor[1] = digitalRead(61); //IR61
  sensor[2] = digitalRead(59); //IR59
  sensor[3] = digitalRead(57); //IR57
  sensor[4] = digitalRead(55); //IR55ซ้าย

  //Serial.println(sensor[0]);
  //Serial.println(sensor[1]);
  //Serial.println(sensor[2]);
  //Serial.println(sensor[3]);
  //Serial.println(sensor[4]);

  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[4] == 1) && (sensor[4] == 0)) {
    error = 4;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
    error = 3;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1)) {
    error = 2;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) {
    error = 1;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) {
    error = 0;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) {
    error = -1;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
    error = -2;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
    error = -3;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
    error = -4;
  }


  //Serial.println(error);

}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  previous_I = I;
  previous_error = error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);


}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;


  constrain(left_motor_speed, 0, 255);
  constrain(right_motor_speed, 0, 255);

  analogWrite(ena, initial_motor_speed - PID_value); //Left Motor Speed
  analogWrite(enb, initial_motor_speed + PID_value); //Right Motor Speed

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stop_bot(int time1 )
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(time1);
}



uint16_t get_gp2d12 (uint16_t value) {
  if (value < 10) value = 10;
  return ((67870.0 / (value - 3.0)) - 40.0);
}

void turn_right() //เลี้ยวขวา
{
  while (digitalRead(55) == 0  || digitalRead(63) == 0 ) {
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

  }
  while ( digitalRead(63) == 1 )
  {
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
}

void turn_left()
{
  while (digitalRead(55) == 0  || digitalRead(63) == 0 ) {
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(50);
  }
  while ( digitalRead(55) == 1 )
  {
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}


void raw() {
  while (digitalRead(sw) == 1 ); {
    analogWrite(ena, 0);
    analogWrite(enb, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  ledloed();
  ledoff();
}

void table1() {
  tacksp();
  turn_left();
  raw();
  tacksp();
  delay(10000);
  turn_right();
  delay(10000);
  turn_left();
  tacksp();
}

void ledflash()
{
  ledred();
  delay(50);
  ledoff();
  delay(50);
  ledred();
  delay(50);
  ledoff();
  delay(50);
  ledred();
  delay(50);
  ledoff();
  delay(50);
  ledred();
  delay(50);
  ledoff();
  delay(50);
  ledred();
  delay(50);
  ledoff();
  delay(50);
  ledred();
  delay(50);
  ledoff();
  delay(50);
}

void ledred()
{
  leds[0] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[1] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[2] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[3] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[4] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[5] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[6] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[7] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[8] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[9] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(0);

}
void ledoff()
{
  leds[0] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[1] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[2] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[3] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[4] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[5] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[6] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[7] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[8] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);
  leds[9] = CRGB(0, 0, 0);//RGB
  FastLED.show();
  delay(0);

}

void ledloed()
{
  int dl = 500;
  leds[0] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[1] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[2] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[3] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[4] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[5] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[6] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[7] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[8] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
  leds[9] = CRGB(255, 0, 0);//RGB
  FastLED.show();
  delay(dl);
}


void dow() {
  OLED.clearDisplay();
  OLED.setCursor(0, 1);
  OLED.display();
  OLED.setTextColor(WHITE);
  OLED.setCursor(55, 20);
  OLED.setTextSize(5);
  OLED.print("3");
  OLED.display();
  delay(1000);
  beep();
  OLED.clearDisplay();
  OLED.setCursor(0, 0);
  OLED.display();
  OLED.setTextColor(WHITE);
  OLED.setCursor(55, 20);
  OLED.setTextSize(5);
  OLED.print("2");
  OLED.display();
  delay(1000);
  beep();
  OLED.clearDisplay();
  OLED.setCursor(0, 0);
  OLED.display();
  OLED.setTextColor(WHITE);
  OLED.setCursor(55, 20);
  OLED.setTextSize(5);
  OLED.print("1");
  OLED.display();
  delay(1000);
  beep();
  OLED.clearDisplay();
  OLED.setCursor(0, 0);
  OLED.display();
  OLED.setTextColor(WHITE);
  OLED.setCursor(25 , 20);
  OLED.setTextSize(5);
  OLED.print("RUN");
  OLED.display();
  tone(buzzer, 1047 ); // Send 1KHz sound signal...
  delay(500);
  noTone(buzzer);     // Stop sound..
}

void clearstate() {
  pad = "";
  OLED.clearDisplay();
  OLED.setCursor(0, 1);
  OLED.display();
}

void beep() {
  tone(buzzer, 1047 ); // Send 1KHz sound signal...
  delay(300);
  noTone(buzzer);
  delay(300);// Stop sound..
}

void readKeypad() {
  keypressed = keypad.getKey(); //deteksi penekanan keypad
  if (keypressed != '#') {
    String konv = String(keypressed);
    pad += konv;
  }
}


void choose_table() {
  readKeypad();
  if (keypressed == '#') {
    if (pad == "10" || firstVal == 10) {
      dow();
      tacksp();

      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("1");
      OLED.display();

      before_turning_left1();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("2");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("3");
      OLED.display();
      raw();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("4");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("5");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("6");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("7");
      OLED.display();

      tacksp(); //5
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("8");
      OLED.display();

      tacksp();//6
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("9");
      OLED.display();

      tacksp();//7
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("10");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("11");
      OLED.display();

      before_entering_left_shop();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("12");    //12
      OLED.display();


      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("13");
      OLED.display();
      uturn();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("14");
      OLED.display();


      delay(500);
      clearstate();
    }
    else if (pad == "20" || secondVal == 11) {
      dow();
      tacksp();

      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("1");
      OLED.display();

      before_turning_left1();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("2");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("3");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("4");
      OLED.display();

      raw();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("5");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("6");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("7");
      OLED.display();

      tacksp(); //5
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("8");
      OLED.display();

      tacksp();//6
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("9");
      OLED.display();

      tacksp();//7
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("10");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("11");
      OLED.display();

      before_entering_left_shop();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("12");    //12
      OLED.display();


      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("13");
      OLED.display();
      uturn();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("14");
      OLED.display();


      delay(500);
      clearstate();
    }
    else if (pad == "30" || nineVal == 18) {
      dow();
      tacksp();

      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("1");
      OLED.display();

      before_turning_left1();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("2");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("3");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("4");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("5");
      OLED.display();
      raw();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("6");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("7");
      OLED.display();

      tacksp(); //5
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("8");
      OLED.display();

      tacksp();//6
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("9");
      OLED.display();

      tacksp();//7
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("10");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("11");
      OLED.display();

      before_entering_left_shop();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("12");    //12
      OLED.display();


      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("13");
      OLED.display();
      uturn();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("14");
      OLED.display();


      delay(500);
      clearstate();
    }
    else if (pad == "40" || tenVal == 19) {
      dow();
      tacksp();

      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("1");
      OLED.display();

      before_turning_left1();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("2");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("3");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("4");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("5");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("6");
      OLED.display();
      raw();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("7");
      OLED.display();
      tacksp(); //5
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("8");
      OLED.display();

      tacksp();//6
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("9");
      OLED.display();

      tacksp();//7
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("10");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("11");
      OLED.display();

      before_entering_left_shop();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("12");    //12
      OLED.display();


      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("13");
      OLED.display();
      uturn();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("14");
      OLED.display();


      delay(500);
      clearstate();
    }
    else if (pad == "50" || elevenVal == 20) {
      dow();
      tacksp();

      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("1");
      OLED.display();

      before_turning_left1();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("2");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("3");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("4");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("5");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("6");
      OLED.display();

      tacksp(); //5
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("7");
      OLED.display();
      raw();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("8");
      OLED.display();
      tacksp();//6
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("9");
      OLED.display();

      tacksp();//7
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("10");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("11");
      OLED.display();

      before_entering_left_shop();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("12");    //12
      OLED.display();


      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("13");
      OLED.display();
      uturn();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("14");
      OLED.display();


      delay(500);
      clearstate();
    }
    else if (pad == "60" || twelthVal == 21) {
      dow();
      tacksp();

      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("1");
      OLED.display();

      before_turning_left1();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("2");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("3");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("4");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("5");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("6");
      OLED.display();

      tacksp(); //5
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("7");
      OLED.display();

      tacksp();//6
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("8");
      OLED.display();

      raw();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("9");
      OLED.display();

      tacksp();//7
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("10");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("11");
      OLED.display();

      before_entering_left_shop();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("12");    //12
      OLED.display();


      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("13");
      OLED.display();
      uturn();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("14");
      OLED.display();


      delay(500);
      clearstate();
    }
    else if (pad == "70" || thirteen == 22) {
      dow();
      tacksp();

      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("1");
      OLED.display();

      before_turning_left1();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("2");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("3");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("4");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("5");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("6");
      OLED.display();

      tacksp(); //5
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("7");
      OLED.display();

      tacksp();//6
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("8");
      OLED.display();

      tacksp();//7
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("10");
      OLED.display();

      raw();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(5);
      OLED.print("9");
      OLED.display();

      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("11");
      OLED.display();

      before_entering_left_shop();

      turn_left();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("12");    //12
      OLED.display();


      tacksp();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("13");
      OLED.display();
      uturn();
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(55, 20);
      OLED.setTextSize(3);
      OLED.print("14");
      OLED.display();


      delay(500);
      clearstate();
    }
    else {
      OLED.clearDisplay();
      OLED.setCursor(0, 1);
      OLED.display();
      delay (10);
      OLED.setTextColor(WHITE);
      OLED.setCursor(10, 20);
      OLED.setTextSize(2);
      OLED.print("Try again");
      OLED.display();
      OLED.setTextColor(WHITE);
      OLED.setCursor(10, 40);
      OLED.setTextSize(2);
      OLED.print("Please *");
      OLED.display();
      beep();
      beep();

    }
  } if (keypressed == '*') {
    pad = "";
    OLED.clearDisplay();
    OLED.setCursor(0, 1);
    OLED.display();
  }
  OLED.setTextColor(WHITE);
  OLED.setCursor(0, 5);
  OLED.setTextSize(1);
  OLED.print("choose table :");
  OLED.display();
  OLED.setTextColor(WHITE);
  OLED.setCursor(85, 5);
  OLED.setTextSize(1);
  OLED.print(pad);
  OLED.display();
  delay(100);

}


void uturn() {
  while (digitalRead(55) == 0  || digitalRead(57) == 0 || digitalRead(59) == 0 || digitalRead(61) == 0 || digitalRead(63) == 0) {
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay (400);
  }
  while ( digitalRead(55) == 1 )
  {
    analogWrite(ena, 180);//240
    analogWrite(enb, 255); //225
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

  }
  analogWrite(ena, 100);//240
  analogWrite(enb, 100); //225
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(200);
  analogWrite(ena, 0);//240
  analogWrite(enb, 0); //225
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);


}

void before_turning_left1() {
  while (digitalRead(55) == 0  && digitalRead(57) == 0 && digitalRead(59) == 0 && digitalRead(61) == 0 && digitalRead(63) == 0)
  {
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(200);
  }
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(100);
}

void before_entering_left_shop() {
  while (digitalRead(63) == 1)
  {
    analogWrite(ena, 255);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(25);
  }
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
