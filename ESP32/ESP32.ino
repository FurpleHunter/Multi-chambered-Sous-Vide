//Temperature Sensor Initialization
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>
#include <LiquidCrystal.h>
#include <BluetoothSerial.h> 

// Temperature sensor setup code from https://randomnerdtutorials.com/esp32-ds18b20-temperature-arduino-ide/
const int oneWireBus = 5;     
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
int numberOfDevices;
DeviceAddress tempDeviceAddress;

//Button Initialization
#define MIDDLE_BUTTON 32
#define LEFT_BUTTON 33
#define RIGHT_BUTTON 25
#define UP_BUTTON 26
#define DOWN_BUTTON 27

int lastStateMid = HIGH;  // the previous state from the input pin
int currentStateMid = HIGH;     // the current reading from the input pin
int lastStateLeft = HIGH;  // the previous state from the input pin
int currentStateLeft = HIGH;     // the current reading from the input pin
int lastStateUp = HIGH;  // the previous state from the input pin
int currentStateUp = HIGH;     // the current reading from the input pin
int lastStateRight = HIGH;  // the previous state from the input pin
int currentStateRight = HIGH;     // the current reading from the input pin
int lastStateDown = HIGH;  // the previous state from the input pin
int currentStateDown = HIGH;     // the current reading from the input pin

unsigned long lastMidPressed = 0;
unsigned long lastLeftPressed = 0;
unsigned long lastRightPressed = 0;
unsigned long lastUpPressed = 0;
unsigned long lastDownPressed = 0;

int midFlag = 0;
int leftFlag = 0;
int rightFlag = 0;
int upFlag = 0;
int downFlag = 0;

void IRAM_ATTR MiddlePressed(){
  currentStateMid = digitalRead(MIDDLE_BUTTON);
    if ((currentStateMid == LOW) && (lastStateMid == HIGH)){
      if(millis() - lastMidPressed > 200){
        midFlag = 1;
        lastMidPressed = millis();
      }
    }
  lastStateMid = currentStateMid;
}

void IRAM_ATTR LeftPressed(){
  currentStateLeft = digitalRead(LEFT_BUTTON);
    if ((currentStateLeft == LOW) && (lastStateLeft == HIGH)){
      if(millis() - lastLeftPressed > 200){
        leftFlag = 1;
        lastLeftPressed = millis();
      }
    }
  lastStateLeft = currentStateLeft;
}

void IRAM_ATTR RightPressed(){
  currentStateRight = digitalRead(RIGHT_BUTTON);
    if ((currentStateRight == LOW) && (lastStateRight == HIGH)){
      if(millis() - lastRightPressed > 200){
        rightFlag = 1;
        lastRightPressed = millis();
      }
    }
  lastStateRight = currentStateRight;
}

void IRAM_ATTR UpPressed(){
  currentStateUp = digitalRead(UP_BUTTON);
    if ((currentStateUp == LOW) && (lastStateUp == HIGH)){
      if(millis() - lastUpPressed > 200){
        upFlag = 1;
        lastUpPressed = millis();
      }
    }
  lastStateUp = currentStateUp;
}

void IRAM_ATTR DownPressed(){
  currentStateDown = digitalRead(DOWN_BUTTON);
    if ((currentStateDown == LOW) && (lastStateDown == HIGH)){
      if(millis() - lastDownPressed > 200){
        downFlag = 1;
        lastDownPressed = millis();
      }
    }
  lastStateDown = currentStateDown;
}

// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal My_LCD(16, 17, 18, 19, 21, 22);

byte InvertUpArrow[8] = {
	0b11111,
	0b11011,
	0b10001,
	0b01010,
	0b11011,
	0b11011,
	0b11011,
	0b11111
};

byte UpArrow[8] = {
	0b00000,
	0b00100,
	0b01110,
	0b10101,
	0b00100,
	0b00100,
	0b00100,
	0b00000
};

byte InvertDownArrow[8] = {
	0b11111,
	0b11011,
	0b11011,
	0b11011,
	0b01010,
	0b10001,
	0b11011,
	0b11111
};

byte DownArrow[8] = {
	0b00000,
	0b00100,
	0b00100,
	0b00100,
	0b10101,
	0b01110,
	0b00100,
	0b00000
};

unsigned long upArrowFlash = 0;
unsigned long downArrowFlash = 0;
int upFlashFlag = 0;
int downFlashFlag = 0;

int mode = 0;
int editMode = 0;
int editPos = 0;

int numChambers = 0;
float chamberSet[3] = {20.0,20.0,20.0};
float currentTemp[3];
float PID_e[3] = {0};
float PID_ePrev[3] {0};
float PID_dt[3] = {0};
float PID_currentTime[3] = {0};
float PID_prevTime[3] = {0};
float PID_proportional[3] = {0};
float PID_integral[3] = {0};
float PID_integralPrev[3] = {0};
float PID_derivative[3] = {0};
float PID_outputFloat[3] = {0};
int PID_output[3] = {0};
char PWMmessage[21] = "#:PWM:0000:00.00:0:$";

int kp = 95;
int ki = 35;
int kd = 70;

BluetoothSerial Bluetooth; 
char currentTemp1Send[7] = "0:00.0";
char currentTemp2Send[7] = "1:00.0";
char currentTemp3Send[7] = "2:00.0";
char setTemp1Send[7] = "3:00.0";
char setTemp2Send[7] = "4:00.0";
char setTemp3Send[7] = "5:00.0";

char bluetoothReceive[7] = {0};

unsigned long bluetoothLastSend = 0;
int set1ChangedFlag = 0;
int set2ChangedFlag = 0;
int set3ChangedFlag = 0;

void setup() {
  Bluetooth.begin("Sous Vide-z V1.0");
  // LCD setup

  My_LCD.createChar(0, UpArrow);
  My_LCD.createChar(1, InvertUpArrow);
  My_LCD.createChar(2, DownArrow);
  My_LCD.createChar(3, InvertDownArrow);

  // Initialize The LCD.
  My_LCD.begin(20, 4);
  My_LCD.clear();
  My_LCD.setCursor(0, 0);
  My_LCD.print("Welcome");
  My_LCD.setCursor(0, 1);
  My_LCD.print("Sous Vide-Z");
  My_LCD.setCursor(0, 2);
  My_LCD.print("V1.0");

  //Serial setup (Baud rate for UART)

  Serial.begin(115200); 
  Serial1.begin (115200, SERIAL_8N1, 3, 1);
  
  sensors.begin();

  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }

  // initialize the pushbutton pin as an pull-up input
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(MIDDLE_BUTTON, INPUT_PULLUP);
  attachInterrupt(MIDDLE_BUTTON, MiddlePressed, CHANGE);
  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  attachInterrupt(LEFT_BUTTON, LeftPressed, CHANGE);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  attachInterrupt(RIGHT_BUTTON, RightPressed, CHANGE);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  attachInterrupt(UP_BUTTON, UpPressed, CHANGE);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);
  attachInterrupt(DOWN_BUTTON, DownPressed, CHANGE);

  delay(2000);
  My_LCD.clear();
}


void loop() {
  Modes();
  PID();
  AppComms();
}
void AppComms(){
  if (millis() - bluetoothLastSend > 1000){ 
    if (set1ChangedFlag == 1){
      setTemp1Send[2] = (chamberSet[0]/10) + 48;
      setTemp1Send[3] = ((int(chamberSet[0] * 10) % 100) / 10) + 48;
      setTemp1Send[5] = ((int(chamberSet[0] * 10) % 100) % 10) + 48;
      uint8_t sTemp1Buf[7];
      memcpy(sTemp1Buf, setTemp1Send, 7);
      Bluetooth.write(sTemp1Buf, 7);
      set1ChangedFlag = 0;
    }
    if (set2ChangedFlag == 1){
      setTemp2Send[2] = (chamberSet[1]/10) + 48;
      setTemp2Send[3] = ((int(chamberSet[1] * 10) % 100) / 10) + 48;
      setTemp2Send[5] = ((int(chamberSet[1] * 10) % 100) % 10) + 48;
      uint8_t sTemp2Buf[7];
      memcpy(sTemp2Buf, setTemp2Send, 7);
      Bluetooth.write(sTemp2Buf, 7);
      set2ChangedFlag = 0;
    }
    if (set3ChangedFlag == 1){
      setTemp3Send[2] = (chamberSet[2]/10) + 48;
      setTemp3Send[3] = ((int(chamberSet[2] * 10) % 100) / 10) + 48;
      setTemp3Send[5] = ((int(chamberSet[2] * 10) % 100) % 10) + 48;
      uint8_t sTemp3Buf[7];
      memcpy(sTemp3Buf, setTemp3Send, 7);
      Bluetooth.write(sTemp3Buf, 7);
      set3ChangedFlag = 0;
    }

    currentTemp1Send[2] = (currentTemp[0]/10) + 48;
    currentTemp1Send[3] = ((int(currentTemp[0] * 10) % 100) / 10) + 48;
    currentTemp1Send[5] = ((int(currentTemp[0] * 10) % 100) % 10) + 48;

    currentTemp2Send[2] = (currentTemp[1]/10) + 48;
    currentTemp2Send[3] = ((int(currentTemp[1] * 10) % 100) / 10) + 48;
    currentTemp2Send[5] = ((int(currentTemp[1] * 10) % 100) % 10) + 48;
    
    currentTemp3Send[2] = (currentTemp[2]/10) + 48;
    currentTemp3Send[3] = ((int(currentTemp[2] * 10) % 100) / 10) + 48;
    currentTemp3Send[5] = ((int(currentTemp[2] * 10) % 100) % 10) + 48;

    uint8_t cTemp1Buf[7];
    uint8_t cTemp2Buf[7];
    uint8_t cTemp3Buf[7];

    memcpy(cTemp1Buf, currentTemp1Send, 7);
    memcpy(cTemp2Buf, currentTemp2Send, 7);
    memcpy(cTemp3Buf, currentTemp3Send, 7);
 
    Bluetooth.write(cTemp1Buf, 7);
    Bluetooth.write(cTemp2Buf, 7);
    Bluetooth.write(cTemp3Buf, 7);

    bluetoothLastSend = millis();
  }
  
  
  if (Bluetooth.available()){
    String tempReceive = Bluetooth.readString();
    memcpy(bluetoothReceive, tempReceive.c_str(), 7);
    if (bluetoothReceive[0] == 54){
      chamberSet[0] = ((bluetoothReceive[2] - 48) * 10) + ((bluetoothReceive[3] - 48) * 1) + ((bluetoothReceive[5] - 48) * 0.1);
    }
    if (bluetoothReceive[0] == 55){
      chamberSet[1] = ((bluetoothReceive[2] - 48) * 10) + ((bluetoothReceive[3] - 48) * 1) + ((bluetoothReceive[5] - 48) * 0.1);
    }
    if (bluetoothReceive[0] == 56){
      chamberSet[2] = ((bluetoothReceive[2] - 48) * 10) + ((bluetoothReceive[3] - 48) * 1) + ((bluetoothReceive[5] - 48) * 0.1);
    }
  } 
}

void PID(){
  // PID equations from Electronoobs: https://electronoobs.com/eng_arduino_tut24_code3.php
  sensors.requestTemperatures();
  for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if (millis() - PID_prevTime[i] > 2000){
      if(sensors.getAddress(tempDeviceAddress, i)){
        PID_prevTime[i] = PID_currentTime[i];
        PID_currentTime[i] = millis();
        PID_dt[i] = (PID_currentTime[i] - PID_prevTime[i])/1000;

        currentTemp[i] = sensors.getTempC(tempDeviceAddress);
        PID_e[i] = chamberSet[i] - currentTemp[i];

        PID_proportional[i] = 0.01*kp*PID_e[i];

        PID_integral[i] = 0.01*PID_integralPrev[i] + (ki*PID_e[i]);

        PID_derivative[i] = 0.01*kd*((PID_e[i] - PID_ePrev[i])/PID_dt[i]);

        PID_outputFloat[i] = PID_proportional[i] + PID_integral[i] + PID_derivative[i];
        PID_output[i] = round(PID_outputFloat[i]);

        if (PID_output[i] > 100){
          PID_output[i] = 100;
        }
        if (PID_output[i] < 0){
          PID_output[i] = 0;
        }
        PWMmessage[6] = i + 48;
        PWMmessage[7] = (PID_output[i] / 100) + 48;
        PWMmessage[8] = ((PID_output[i] % 100) / 10) + 48;
        PWMmessage[9] = ((PID_output[i] % 100) % 10) + 48;

        PWMmessage[11] = (currentTemp[i] / 10) + 48;
        PWMmessage[12] = (int(currentTemp[i]) % 10) + 48;
        PWMmessage[14] = (((int(currentTemp[i]*100) % 1000) % 100) / 10) + 48;
        PWMmessage[15] = (((int(currentTemp[i]*100) % 1000) % 100) % 10) + 48;
        if(currentTemp[i] < 30){
          PWMmessage[17] = 48;
        }
        if((currentTemp[i] > 30) && (currentTemp[i] < (chamberSet[i] - 2))){
          PWMmessage[17] = 49;
        }
        if((currentTemp[i] > 30) && (currentTemp[i] > (chamberSet[i] - 2))){
          PWMmessage[17] = 50;
        }
        if (currentTemp[i] > 0.00){
          Serial1.print(PWMmessage);  
        }
       

        PID_integralPrev[i] = PID_integral[i];
        PID_ePrev[i] = PID_e[i];
        delay(250);

      }
    }
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}

void Modes(){
  if (leftFlag == 1){
    My_LCD.clear();
    if (editMode == 0){
      mode = mode - 1;
      if (mode == -1){
        mode = numChambers + 1;
      }
    }
    if (editMode == 1){
      if ((mode == 2) || (mode == 3) || (mode == 4)){
        editPos = editPos - 1;
        if (editPos == -1){
          editPos = 0;
        }
      }
    }
    leftFlag = 0;
  }

  if (rightFlag == 1){
    My_LCD.clear();
    if (editMode == 0){
      mode = mode + 1;
      if (mode == (numChambers + 2)){
        mode = 0;
      }
    }
    if (editMode == 1){
      if ((mode == 2) || (mode == 3) || (mode == 4)){
        editPos = editPos + 1;
        if (editPos == 3){
          editPos = 2;
        }
      }
    }
    rightFlag = 0;
  }

  if (midFlag == 1){
    My_LCD.clear();
    editPos = 0;
    if ((mode == 1) || (mode == 2) || (mode == 3) || (mode == 4)){
      editMode = editMode + 1;
      if (editMode > 1){
        editMode = 0; 
      }
    }
    midFlag = 0;
  }

  if (mode == 0){
    My_LCD.setCursor(4, 0);
    My_LCD.print("Chambers(");
    My_LCD.setCursor(13, 0);
    My_LCD.print((char)223);
    My_LCD.setCursor(14, 0);
    My_LCD.print("C)");
    My_LCD.setCursor(6, 1);
    My_LCD.print("1:");
    My_LCD.setCursor(11, 1);
    My_LCD.print("2:");
    My_LCD.setCursor(16, 1);
    My_LCD.print("3:");
    My_LCD.setCursor(0, 2);
    My_LCD.print("Set:");
    My_LCD.setCursor(0, 2);
    My_LCD.print("Set:");
    My_LCD.setCursor(0, 3);
    My_LCD.print("Temp:");
    My_LCD.setCursor(6, 3);
    My_LCD.print(currentTemp[0],1);
    My_LCD.setCursor(11, 3);
    My_LCD.print(currentTemp[1],1);
    My_LCD.setCursor(16, 3);
    My_LCD.print(currentTemp[2],1);

    if (numChambers == 0){
      My_LCD.setCursor(6, 2);
      My_LCD.print("N/A");
      My_LCD.setCursor(11, 2);
      My_LCD.print("N/A");
      My_LCD.setCursor(16, 2);
      My_LCD.print("N/A");
    }
    if (numChambers == 1){
      My_LCD.setCursor(6, 2);
      My_LCD.print(chamberSet[0],1);
      My_LCD.setCursor(11, 2);
      My_LCD.print("N/A");
      My_LCD.setCursor(16, 2);
      My_LCD.print("N/A");
    }
    if (numChambers == 2){
      My_LCD.setCursor(6, 2);
      My_LCD.print(chamberSet[0],1);
      My_LCD.setCursor(11, 2);
      My_LCD.print(chamberSet[1],1);
      My_LCD.setCursor(16, 2);
      My_LCD.print("N/A");
    }
    if (numChambers == 3){
      My_LCD.setCursor(6, 2);
      My_LCD.print(chamberSet[0],1);
      My_LCD.setCursor(11, 2);
      My_LCD.print(chamberSet[1],1);
      My_LCD.setCursor(16, 2);
      My_LCD.print(chamberSet[2],1);
    }
  }

  if (mode == 1){
    if (editMode == 0){
      My_LCD.setCursor(0, 0);
      My_LCD.print("Set number");
      My_LCD.setCursor(4, 1);
      My_LCD.print("of");
      My_LCD.setCursor(1, 2);
      My_LCD.print("chambers");
      My_LCD.setCursor(11, 1);
      My_LCD.print("=");
      My_LCD.setCursor(14, 1);
      My_LCD.print(numChambers);
      if (upFlag == 1){
        upFlag = 0;
      }
      if (downFlag == 1){
        downFlag = 0;
      }
    }

    if (editMode == 1){
      My_LCD.setCursor(0, 0);
      My_LCD.print("Set number");
      My_LCD.setCursor(4, 1);
      My_LCD.print("of");
      My_LCD.setCursor(1, 2);
      My_LCD.print("chambers");
      My_LCD.setCursor(11, 1);
      My_LCD.print("=");
      My_LCD.setCursor(14, 1);
      My_LCD.print(numChambers);

      if (upFlashFlag == 0){
        My_LCD.setCursor(14, 0);
        My_LCD.write((uint8_t)0);
      }
      if (downFlashFlag == 0){
        My_LCD.setCursor(14, 2);
        My_LCD.write((uint8_t)2);
      }

      if (upFlag == 1){
        numChambers = numChambers + 1;
        if (numChambers == 4){
          numChambers = 0;
        }
        My_LCD.setCursor(14, 0);
        My_LCD.write((uint8_t)1);
        upArrowFlash = millis();
        upFlashFlag = 1;
        upFlag = 0;
      }
      if (downFlag == 1){
        numChambers = numChambers - 1;
        if (numChambers == -1){
          numChambers = 0;
        }
        My_LCD.setCursor(14, 2);
        My_LCD.write((uint8_t)3);
        downArrowFlash = millis();
        downFlashFlag = 1;
        downFlag = 0;
      }

      if (((millis() - upArrowFlash) > 300) && upFlashFlag == 1){
        upFlashFlag = 0;
      }
      if (((millis() - downArrowFlash) > 300) && downFlashFlag == 1){
        downFlashFlag = 0;
      }
    }
    
  }
  
  if (mode == 2){
    if (editMode == 0){
      My_LCD.setCursor(0, 0);
      My_LCD.print("Set chamber 1 temp");  
      My_LCD.setCursor(7, 2);
      My_LCD.print(chamberSet[0],1); 
      My_LCD.setCursor(11, 2);
      My_LCD.print((char)223);
      My_LCD.setCursor(12, 2);
      My_LCD.print("C");
      if (upFlag == 1){
        upFlag = 0;
      }
      if (downFlag == 1){
        downFlag = 0;
      }
    }
    if (editMode == 1){
      My_LCD.setCursor(0, 0);
      My_LCD.print("Set chamber 1 temp");  
      My_LCD.setCursor(7, 2);
      My_LCD.print(chamberSet[0],1); 
      My_LCD.setCursor(11, 2);
      My_LCD.print((char)223);
      My_LCD.setCursor(12, 2);
      My_LCD.print("C");

      if (editPos == 0){
        if (upFlashFlag == 0){
          My_LCD.setCursor(7, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(7, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[0] = chamberSet[0] + 10;
          if (chamberSet[0] > 90.0){
            chamberSet[0] = 20.0;
          }
          My_LCD.setCursor(7, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          upFlag = 0;
          set1ChangedFlag = 1;
        }
        if (downFlag == 1){
          chamberSet[0] = chamberSet[0] - 10;
          if (chamberSet[0] < 20.0){
            chamberSet[0] = 20.0;
          }
          My_LCD.setCursor(7, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          downFlag = 0;
          set1ChangedFlag = 1;
        }
      }

      if (editPos == 1){
        if (upFlashFlag == 0){
          My_LCD.setCursor(8, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(8, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[0] = chamberSet[0] + 1;
          if (chamberSet[0] > 90.0){
            chamberSet[0] = 20.0;
          }
          My_LCD.setCursor(8, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          set1ChangedFlag = 1;
          upFlag = 0;
        }
        if (downFlag == 1){
          chamberSet[0] = chamberSet[0] - 1;
          if (chamberSet[0] < 20.0){
            chamberSet[0] = 20.0;
          }
          My_LCD.setCursor(8, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          set1ChangedFlag = 1;
          downFlag = 0;
        }
      }

      if (editPos == 2){
        if (upFlashFlag == 0){
          My_LCD.setCursor(10, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(10, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[0] = chamberSet[0] + 0.1;
          if (chamberSet[0] > 90.0){
            chamberSet[0] = 20.0;
          }
          My_LCD.setCursor(10, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          set1ChangedFlag = 1;
          upFlag = 0;
        }
        if (downFlag == 1){
          chamberSet[0] = chamberSet[0] - 0.1;
          if (chamberSet[0] < 20.0){
            chamberSet[0] = 20.0;
          }
          My_LCD.setCursor(10, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          set1ChangedFlag = 1;
          downFlag = 0;
        }
      }

      if (((millis() - upArrowFlash) > 300) && upFlashFlag == 1){
        upFlashFlag = 0;
      }
      if (((millis() - downArrowFlash) > 300) && downFlashFlag == 1){
        downFlashFlag = 0;
      }
    }
    
  }

  if (mode == 3){
    if (editMode == 0){
      My_LCD.setCursor(0, 0);
      My_LCD.print("Set chamber 2 temp");  
      My_LCD.setCursor(7, 2);
      My_LCD.print(chamberSet[1],1); 
      My_LCD.setCursor(11, 2);
      My_LCD.print((char)223);
      My_LCD.setCursor(12, 2);
      My_LCD.print("C");
      if (upFlag == 1){
        upFlag = 0;
      }
      if (downFlag == 1){
        downFlag = 0;
      }
    }
    
    if (editMode == 1){
      My_LCD.setCursor(0, 0);
      My_LCD.print("Set chamber 2 temp");  
      My_LCD.setCursor(7, 2);
      My_LCD.print(chamberSet[1],1); 
      My_LCD.setCursor(11, 2);
      My_LCD.print((char)223);
      My_LCD.setCursor(12, 2);
      My_LCD.print("C");

      if (editPos == 0){
        if (upFlashFlag == 0){
          My_LCD.setCursor(7, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(7, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[1] = chamberSet[1] + 10;
          if (chamberSet[1] > 90.0){
            chamberSet[1] = 20.0;
          }
          My_LCD.setCursor(7, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          set2ChangedFlag = 1;
          upFlag = 0;
        }
        if (downFlag == 1){
          chamberSet[1] = chamberSet[1] - 10;
          if (chamberSet[1] < 20.0){
            chamberSet[1] = 20.0;
          }
          My_LCD.setCursor(7, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          set2ChangedFlag = 1;
          downFlag = 0;
        }
      }

      if (editPos == 1){
        if (upFlashFlag == 0){
          My_LCD.setCursor(8, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(8, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[1] = chamberSet[1] + 1;
          if (chamberSet[1] > 90.0){
            chamberSet[1] = 20.0;
          }
          My_LCD.setCursor(8, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          set2ChangedFlag = 1;
          upFlag = 0;
        }
        if (downFlag == 1){
          chamberSet[1] = chamberSet[1] - 1;
          if (chamberSet[1] < 20.0){
            chamberSet[1] = 20.0;
          }
          My_LCD.setCursor(8, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          set2ChangedFlag = 1;
          downFlag = 0;
        }
      }

      if (editPos == 2){
        if (upFlashFlag == 0){
          My_LCD.setCursor(10, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(10, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[1] = chamberSet[1] + 0.1;
          if (chamberSet[1] > 90.0){
            chamberSet[1] = 20.0;
          }
          My_LCD.setCursor(10, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          set2ChangedFlag = 1;
          upFlag = 0;
        }
        if (downFlag == 1){
          chamberSet[1] = chamberSet[1] - 0.1;
          if (chamberSet[1] < 20.0){
            chamberSet[1] = 20.0;
          }
          My_LCD.setCursor(10, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          set2ChangedFlag = 1;
          downFlag = 0;
        }
      }
      if (((millis() - upArrowFlash) > 300) && upFlashFlag == 1){
        upFlashFlag = 0;
      }
      if (((millis() - downArrowFlash) > 300) && downFlashFlag == 1){
        downFlashFlag = 0;
      }
    }
    
  }

  if (mode == 4){
    if (editMode == 0){
      My_LCD.setCursor(0, 0);
      My_LCD.print("Set chamber 3 temp");  
      My_LCD.setCursor(7, 2);
      My_LCD.print(chamberSet[2],1); 
      My_LCD.setCursor(11, 2);
      My_LCD.print((char)223);
      My_LCD.setCursor(12, 2);
      My_LCD.print("C");
      if (upFlag == 1){
        upFlag = 0;
      }
      if (downFlag == 1){
        downFlag = 0;
      }
    }

    if (editMode == 1){
      My_LCD.setCursor(0, 0);
      My_LCD.print("Set chamber 3 temp");  
      My_LCD.setCursor(7, 2);
      My_LCD.print(chamberSet[2],1); 
      My_LCD.setCursor(11, 2);
      My_LCD.print((char)223);
      My_LCD.setCursor(12, 2);
      My_LCD.print("C");

      if (editPos == 0){
        if (upFlashFlag == 0){
          My_LCD.setCursor(7, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(7, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[2] = chamberSet[2] + 10;
          if (chamberSet[2] > 90.0){
            chamberSet[2] = 20.0;
          }
          My_LCD.setCursor(7, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          set3ChangedFlag = 1;
          upFlag = 0;
        }
        if (downFlag == 1){
          chamberSet[2] = chamberSet[2] - 10;
          if (chamberSet[2] < 20.0){
            chamberSet[2] = 20.0;
          }
          My_LCD.setCursor(7, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          set3ChangedFlag = 1;
          downFlag = 0;
        }
      }

      if (editPos == 1){
        if (upFlashFlag == 0){
          My_LCD.setCursor(8, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(8, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[2] = chamberSet[2] + 1;
          if (chamberSet[2] > 90.0){
            chamberSet[2] = 20.0;
          }
          My_LCD.setCursor(8, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          set3ChangedFlag = 1;
          upFlag = 0;
        }
        if (downFlag == 1){
          chamberSet[2] = chamberSet[2] - 1;
          if (chamberSet[2] < 20.0){
            chamberSet[2] = 20.0;
          }
          My_LCD.setCursor(8, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          set3ChangedFlag = 1;
          downFlag = 0;
        }
      }

      if (editPos == 2){
        if (upFlashFlag == 0){
          My_LCD.setCursor(10, 1);
          My_LCD.write((uint8_t)0);
        }
        if (downFlashFlag == 0){
          My_LCD.setCursor(10, 3);
          My_LCD.write((uint8_t)2);
        }

        if (upFlag == 1){
          chamberSet[2] = chamberSet[2] + 0.1;
          if (chamberSet[2] > 90.0){
            chamberSet[2] = 20.0;
          }
          My_LCD.setCursor(10, 1);
          My_LCD.write((uint8_t)1);
          upArrowFlash = millis();
          upFlashFlag = 1;
          set3ChangedFlag = 1;
          upFlag = 0;
        }
        if (downFlag == 1){
          chamberSet[2] = chamberSet[2] - 0.1;
          if (chamberSet[2] < 20.0){
            chamberSet[2] = 20.0;
          }
          My_LCD.setCursor(10, 3);
          My_LCD.write((uint8_t)3);
          downArrowFlash = millis();
          downFlashFlag = 1;
          set3ChangedFlag = 1;
          downFlag = 0;
        }
      }
      if (((millis() - upArrowFlash) > 300) && upFlashFlag == 1){
        upFlashFlag = 0;
      }
      if (((millis() - downArrowFlash) > 300) && downFlashFlag == 1){
        downFlashFlag = 0;
      }
    }
    
  }

 }