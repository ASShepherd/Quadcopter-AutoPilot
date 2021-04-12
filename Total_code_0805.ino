#include <SoftwareSerial.h>
#include<Servo.h>
#define MSP_ATTITUDE 108
#include <SPI.h>
#include <SD.h>
#include<Servo.h>
Servo roll,pitch,throttle,aux1,yaw,dropper;
File myFile;
double IRdistance1;
double IRdistance2;
double IRdistance3;
double avgdistance1;
double avgdistance2;
double avgdistance3;
String filename = "Flight.txt";
int sensorpin1 = A0;                 // analog pin used to connect the sharp sensor
int val1 = 0;                 // variable to store the values from sensor(initially zero)
int sensorpin2 = A1;                 // analog pin used to connect the sharp sensor
int val2 = 0;                 // variable to store the values from sensor(initially zero)
int sensorpin3 = A2;                 // analog pin used to connect the sharp sensor
int val3 = 0;                 // variable to store the values from sensor(initially zero)
int i = 1;
int j = 0;
int pos=0;
unsigned long currentduration;
unsigned long prevduration;
boolean fexst;
int throttleval;
SoftwareSerial mspSerial(9, 8); // RX TX

void setup() {
    dropper.write(179);
    mspSerial.begin(9600);
    Serial.begin(9600);
    throttle.attach(4);
    roll.attach(5);
    pitch.attach(6);
    yaw.attach(7);
    dropper.attach(A3);
      while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  if (SD.exists(filename)) {
    Serial.print(filename);
    Serial.println(" exists");
    do {
      i = i+1;
      filename = ("Flight" + String(i) + ".txt");
      Serial.println(filename);
      fexst = (SD.exists(filename));
      Serial.println(fexst);
    }while (fexst == 1);
  } else {
    Serial.print(filename);
    Serial.println("doesn't exist.");
  }

  // open a new file and immediately close it:
  Serial.print(filename);
  Serial.println(" is being created");
  myFile = SD.open(filename, FILE_WRITE);
  myFile.close();

  // Check to see if the file exists:
  if (SD.exists(filename)) {
    Serial.print(filename);
    Serial.println(" exists.");
  } else {
    Serial.print(filename);
    Serial.println("doesn't exist.");
  }

}

void loop() {
  // put your main code here, to run repeatedly:
 rcswitch();
 if ((currentduration>1600) && (prevduration<1500)){
   intauto();
 }
 else{
   //return;
 }
 //arduinoservos();
 writesensors();
 arduinoservoiter();
 drop();
 uint8_t datad = 0;
 uint8_t *data = &datad;
 //pitch.writeMicroseconds(1800);  // set servo to mid-point
 
 sendMSP(MSP_ATTITUDE, data, 0);
 readData();
 prevduration = currentduration;
}

void IRs() {
  j = 0;
  avgdistance1 = 0;
  avgdistance2 = 0;
  avgdistance3 = 0;
  do{
  val1 = analogRead(sensorpin1);       // reads the value of the sharp sensor
  val2 = analogRead(sensorpin2);       // reads the value of the sharp sensor
  val3 = analogRead(sensorpin3);       // reads the value of the sharp sensor
  /*Serial.print("Sensor 1: ");            // prints the number of the sensor to the serial monitor
  Serial.println(val1);            // prints the value of the sensor to the serial monitor
  Serial.print("Sensor 2: ");            // prints the number of the sensor to the serial monitor
  Serial.println(val2 );            // prints the value of the sensor to the serial monitor
  Serial.print("Sensor 3: ");            // prints the number of the sensor to the serial monitor
  Serial.println(val3 );            // prints the value of the sensor to the serial monitor*/
 
  IRdistance1 = 187754 * pow(val1, -1.51);
  IRdistance2 = 187754 * pow(val2, -1.51);
  IRdistance3 = 187754 * pow(val3, -1.51);
  /*Serial.print("Sensor 1(cm): ");            // prints the number of the sensor to the serial monitor
  Serial.println(IRdistance1);            // prints the value of the sensor to the serial monitor in cm 
  Serial.print("Sensor 2(cm): ");            // prints the number of the sensor to the serial monitor
  Serial.println(IRdistance2 );            // prints the value of the sensor to the serial monitor in cm 
  Serial.print("Sensor 3(cm): ");            // prints the number of the sensor to the serial monitor
  Serial.println(IRdistance3 );            // prints the value of the sensor to the serial monitor in cm */
  delay(100);                    // wait for this much time before printing next value
  avgdistance1 = avgdistance1 + IRdistance1;
  avgdistance2 = avgdistance2 + IRdistance2;
  avgdistance3 = avgdistance3 + IRdistance3;
  j = j + 1;
  Serial.print("J is ="); 
  Serial.println(j); 
  }
  while (j <5);
  avgdistance1 = avgdistance1 / 5;
  avgdistance2 = avgdistance2 / 5;
  avgdistance3 = avgdistance3 / 5;
  Serial.println("Averaging....");
  Serial.print("Avg Sensor 1(cm): ");            // prints the number of the sensor to the serial monitor
  Serial.println(avgdistance1);            // prints the value of the sensor to the serial monitor in cm 
  Serial.print("Avg Sensor 2(cm): ");            // prints the number of the sensor to the serial monitor
  Serial.println(avgdistance2 );            // prints the value of the sensor to the serial monitor in cm 
  Serial.print("Avg Sensor 3(cm): ");            // prints the number of the sensor to the serial monitor
  Serial.println(avgdistance3 );            // prints the value of the sensor to the serial monitor in cm   
}

void writesensors(){
  IRs();
  //Serial.println(filename);
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    
  }
    myFile.print("Sensor 1(cm): ");            // prints the number of the sensor to the serial monitor);
    myFile.print(avgdistance1);
    myFile.print("     Sensor 2(cm): ");            // prints the number of the sensor to the serial monitor);
    myFile.print(avgdistance2);
    myFile.print("     Sensor 3(cm): ");            // prints the number of the sensor to the serial monitor);
    myFile.print(avgdistance3);
    myFile.println();
    myFile.close();

}

void arduinoservos() {
  // put your main code here, to run repeatedly:
throttle.writeMicroseconds(1800);  // set servo to mid-point
roll.writeMicroseconds(1800);  // set servo to mid-point
pitch.writeMicroseconds(1800);  // set servo to mid-point
yaw.writeMicroseconds(1800);  // set servo to mid-point
Serial.println("Hello");
delay(1000);
throttle.writeMicroseconds(900);  // set servo to mid-point
roll.writeMicroseconds(1500);  // set servo to mid-point
pitch.writeMicroseconds(1500);  // set servo to mid-point
yaw.writeMicroseconds(1500);  // set servo to mid-point
delay(500);

}
void arduinoservoiter() {
  // put your main code here, to run repeatedly:
throttle.writeMicroseconds(throttleval);  // set servo to mid-point
roll.writeMicroseconds(1500);  // set servo to mid-point
pitch.writeMicroseconds(1500);  // set servo to mid-point
yaw.writeMicroseconds(1500);  // set servo to mid-point
Serial.println("Hello");
if (avgdistance1 >= 100){
  throttleval = throttleval - 20;
}
else if(avgdistance1 <100){
  throttleval = throttleval + 20;
}

}


void sendMSP(uint8_t cmd, uint8_t *data, uint8_t n_bytes) {

    uint8_t checksum = 0;

    mspSerial.write((byte *) "$M<", 3);
    mspSerial.write(n_bytes);
    checksum ^= n_bytes;

    mspSerial.write(cmd);
    checksum ^= cmd;

    mspSerial.write(checksum);
}

void readData() {
    delay(100);

    byte count = 0;

    int16_t roll;
    int16_t pitch;
    int16_t yaw;

    while (mspSerial.available()) {
        count += 1;
        byte c = mspSerial.read();
        switch (count) {
            case 6:
                roll = c;
                break;
            case 7:
                roll <<= 8;
                roll += c;
                roll = (roll & 0xFF00) >> 8 | (roll & 0x00FF) << 8; // Reverse the order of bytes
                break;
            case 8:
                pitch += c;
                break;
            case 9:
                pitch <<= 8;
                pitch += c;
                pitch = (pitch & 0xFF00) >> 8 | (pitch & 0x00FF) << 8; // Reverse the order of bytes
                break;
            case 10:
                yaw += c;
                break;
            case 11:
                yaw <<= 8;
                yaw += c;
                yaw = (yaw & 0xFF00) >> 8 | (yaw & 0x00FF) << 8; // Reverse the order of bytes
                break;
        }
    }

    Serial.print("Roll: " + String(roll/10.0));
    Serial.print(" Pitch: " + String(pitch/10.0));
    Serial.println(" Yaw: " + String(yaw));
}

void drop(){
  if ((avgdistance1 >=95.00) && (avgdistance1<= 105.00))
  {
    Serial.println(avgdistance1);
    Serial.println("Dropping....");
      for (pos = 180; pos >= 40; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    dropper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);   
/*      for (pos = 40; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    dropper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15); 
  }*/
  }
  }
  else{
    return;
  }
}

void rcswitch(){
 currentduration = pulseIn(A5, HIGH);
 Serial.println(currentduration);
}

void intauto(){
  Serial.println("Autopilot values initialised");
  throttleval = 1050;
}

