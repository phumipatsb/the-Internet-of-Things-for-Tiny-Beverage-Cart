#define BLYNK_TEMPLATE_ID "TMPLGVEGCi9q"
#define BLYNK_DEVICE_NAME "Car"
//nodemcu
// Include the library files
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

int pinValue1;
int pinValue2;
int pinValue3;
int pinValue4, pinValue5, pinValue6, pinValue7, pinValue8, pinValue9;
int pinValue10, pinValue11, pinValue12, pinValue13, pinValue14;
int raw, batteryper;
float volt;
String v2arduino; // values to arduino
String v3arduino;

int val11;
float val2, temp;

char auth[] = "f7Bzob5wxdq7oD3B0iGUqFTLrpjkWaeK";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "2410";
char pass[] = "0969174870";
//char ssid[] = "iotwifi";
//char pass[] = "1234567890";

SimpleTimer timer;

String myString; // complete message from arduino, which consistors of snesors data
char rdata; // received charactors
char rdata2;
int firstVal, secondVal, thirdVal, forthVal, fifthVal, sixVal, sevenVal, eightVal, nineVal; // sensors
int tenVal, elevenVal, twelthVal, thirteen,thirteen1;
// This function sends Arduino's up time every second to Virtual Pin (1).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void myTimerEvent()
{
  Blynk.virtualWrite(V1, millis() / 2000);
}



void setup()
{
  // Debug console
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);

  timer.setInterval(1000L, sensorvalue1);
  timer.setInterval(1000L, sensorvalue2);
  timer.setInterval(1000L, sensorvalue3);
  timer.setInterval(1000L, sensorvalue4);
  timer.setInterval(5000L, myTimerEvent2);
}

void loop()
{
  if (Serial.available() == 0 )
  {
    Blynk.run();
    timer.run(); // Initiates BlynkTimer
   
    toarduino();




  }

  if (Serial.available() > 0 )
  {
    rdata = Serial.read();
    myString = myString + rdata;
    // Serial.print(rdata);
    if ( rdata == '\n')
    {

      // new code
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
      String ab = getValue(myString, ',',13);

      firstVal  = l.toInt(); //table1 0
      secondVal = m.toInt(); //table2 1

      thirdVal  = n.toInt(); //speed  2
      forthVal  = t.toInt();  //up    3
      fifthVal  = y.toInt();  //down  4
      sixVal    = u.toInt();  //left  5
      sevenVal  = i.toInt();  //right 6
      eightVal  = o.toInt(); //stop   7

      nineVal   = p.toInt(); //table3 8
      tenVal    = z.toInt(); //table4 9
      elevenVal = x.toInt(); //table5 10
      twelthVal = c.toInt(); //table6 11
      thirteen  = a.toInt(); //table7 12
      thirteen1  = ab.toInt(); //table7 12

      myString = "";


    }
  }

}
void sensorvalue1()
{
  int sdata = firstVal;
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, sdata);

}
void sensorvalue2()
{
  int sdata = secondVal;
  Blynk.virtualWrite(V3, sdata);

}

void sensorvalue3()
{

  // You can send any value at any time.
  // Please don't send more that 10 values per second.

  Blynk.virtualWrite(V18, nineVal);
  Blynk.virtualWrite(V19, tenVal);
}
void sensorvalue4()
{

  Blynk.virtualWrite(V20,  elevenVal);
  Blynk.virtualWrite(V21,  twelthVal);
  Blynk.virtualWrite(V22,  thirteen);
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

// in Blynk app writes values to the Virtual Pin V3
BLYNK_WRITE(V10)
{
  pinValue1 = param.asInt(); //Table1


}


// in Blynk app writes values to the Virtual Pin V4
BLYNK_WRITE(V11)
{
  pinValue2 = param.asInt(); //Table2

}

// in Blynk app writes values to the Virtual Pin V5, this is for the slider
BLYNK_WRITE(V12)
{
  pinValue3 = param.asInt(); // Speed

}

BLYNK_WRITE(V13)
{
  pinValue4 = param.asInt(); // Top

}
BLYNK_WRITE(V14)
{
  pinValue5 = param.asInt(); //Left

}
BLYNK_WRITE(V15)
{
  pinValue6 = param.asInt(); //Right

}
BLYNK_WRITE(V16)
{
  pinValue7 = param.asInt(); //down

}
BLYNK_WRITE(V17)
{
  pinValue8 = param.asInt(); //stop


}
BLYNK_WRITE(V18)
{
  pinValue9 = param.asInt(); // assigning incoming value from pin V10 to a variable

}
BLYNK_WRITE(V19) {
  pinValue10 = param.asInt();
}
BLYNK_WRITE(V20) {
  pinValue11 = param.asInt();
}
BLYNK_WRITE(V21) {
  pinValue12 = param.asInt();
}
BLYNK_WRITE(V22) {
  pinValue13 = param.asInt();
}
void myTimerEvent2() {
  raw = analogRead(A0);
  volt = 5.0 * raw / 1023.0;
  Serial.println(volt);
  if ( volt >= 12.6) {
    batteryper = 100;
  }
  else if (volt >= 11.5 || volt <= 12.5 ) {
    batteryper = 75;
  }
  else if (volt >= 11.1 || volt <= 11.4) {
    batteryper = 50;
  }
  else if (volt >= 10.5 || volt <= 11.0) {
    batteryper = 25;
  }
  else if (volt <= 10.4) {
    batteryper = 0;
  }
   Blynk.virtualWrite(V23, batteryper);
   //Serial.println(batteryper);
   pinValue14 = batteryper;
}

void toarduino()
{
  v2arduino = v2arduino + pinValue1 + "," + pinValue2 + "," + pinValue3 + "," + pinValue4 + "," + pinValue5 + ","
              + pinValue6 + "," + pinValue7 + "," + pinValue8 + "," + pinValue9 + "," + pinValue10 + "," + pinValue11 + "," + pinValue12 + "," + pinValue13+ "," + pinValue14;

  Serial.println(v2arduino);
  delay(1000);
  v2arduino = "";
}
