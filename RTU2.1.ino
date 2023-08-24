/*Zayan Tofeeq
  Arduino Project
  Digitize .inc
*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <EEPROM.h>
#include <SimpleDHT.h>
#include "Arduino.h"
#include <FastLED.h>
#include "uRTCLib.h"
#include "AnalogKeypad.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <LiquidCrystal_I2C.h>


// Defined tokens
#define t_OFF 0
#define t_ON 1
#define t_LED 2
#define t_D13 3
#define t_RED 5
#define t_GREEN 6
#define t_BLINK 7
#define t_SET 8
#define t_WORD 9
#define t_STATUS 10
#define t_LEDS 11
#define t_VERSION 12
#define t_HELP 13
#define t_READ 14
#define t_TIME 15
#define t_ADD 16
#define t_RGB 17
#define t_RGBBLINK 18
#define t_TEMP 19
#define t_EEPROM 20
#define t_RESET 21
#define t_POWER 22

#define LOOKUPTABLE_SIZE 22
#define t_EOL 255
#define BUFFER_SIZE 30
#define MAX_COMMAND_SIZE 10
#define NUM_LEDS 4
#define DATA_PIN 2

// initilizating variables
char commandBuffer[BUFFER_SIZE];
int commandIndex = 0;
char inputBuffer[BUFFER_SIZE];
int inputIndex = 0;
int wordIndex = 0;
char chr[BUFFER_SIZE];
long tokenbuffer[BUFFER_SIZE];
int token_buffer_index = 0;
long TIME_INTERVAL = 500;
byte ledPattern = 0b01010101;
unsigned long previous_d13_millis = 0;
unsigned long previous_led_millis = 0;
unsigned long previous_rgb_millis = 0;

byte _temperature;
byte _humidity;
byte highTemp;
byte lowTemp;

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress localip(192, 168, 16, 177);
unsigned int localPort = 8888;

IPAddress remoteip(192, 168, 16, 24);
unsigned int remotePort = 41946;            // keeps changing

int packets_rec = 0;
int packets_sent = 0;

int nextAddress = 0;
const int EEPROM_START_ADDRESS = 0;
const int EEPROM_SIZE = 976;
const int EEPROM_LAST_ADDRESS = EEPROM_SIZE - 1;
const int ENTRY_SIZE = 8;

const unsigned long eeprom_write_interval = 30 * 1000; // 15 min
const unsigned long write_interval = 5 * 1000; // 15 min
static unsigned long eeprompreviousMillis = 0;
static unsigned long writepreviousMillis = 0;

uRTCLib rtc(0x68);
CRGB leds[NUM_LEDS];

char packetBuffer[BUFFER_SIZE];
char ReplyBuffer[BUFFER_SIZE];
EthernetUDP Udp;

// flags
bool is_Blinking = false;
bool led_is_Blinking = false;
bool Red_is_Blinking = false;
bool Green_is_Blinking = false;
bool D13_is_On = false;
bool Red_is_On = false;
bool Green_is_On = false;
bool Last_on_Red = false;
bool Last_on_Green = false;
bool rgb_is_on = false;
bool rgb_is_blinking = false;
bool Power_status = false;
bool UDP_request = false;

byte temperatureBounds[] = {60, 70, 80, 90, 100};  // Upper bounds of each range


enum TemperatureRange {
    Major_Under,
    Minor_Under,
    Comfortable,
    Minor_Over,
    Major_Over
};
TemperatureRange temperatureRange = Comfortable;
TemperatureRange previousTemperatureRange;


// initilizing led pins
const int ledPin12 = 9;
const int ledPin11 = 8;
int pinDHT = 4;

// lookup table
const char lookupTable[][4] = {
  {'o', 'f', 3, t_OFF},
  {'o', 'n', 2, t_ON},
  {'d', '1', 3, t_D13},
  {'l', 'e', 3, t_LED},
  {'r', 'e', 3, t_RED},
  {'g', 'r', 5, t_GREEN},
  {'b', 'l', 5, t_BLINK},
  {'s', 'e', 3, t_SET},
  {'t', 'w', 4, t_WORD},
  {'s', 't', 6, t_STATUS},
  {'l', 'e', 4, t_LEDS},
  {'v', 'e', 7, t_VERSION},
  {'h', 'e', 4, t_HELP},
  {'r', 'e', 4,t_READ},
  {'r', 'e', 5,t_RESET},
  {'t', 'i', 4,t_TIME},
  {'a', 'd', 3, t_ADD},
  {'r', 'g', 3, t_RGB},
  {'r', 'g', 8, t_RGBBLINK},
  {'t', 'e', 4, t_TEMP},
  {'e','e', 6, t_EEPROM},
  {'p', 'o', 5, t_POWER}
};


const char menuOption1[] PROGMEM = "D13 ON, D13 OFF, D13 BLINK";
const char menuOption2[] PROGMEM = "LED GREEN, LED RED, LED OFF, LED BLINK, STATUS LEDS, RGB # # #, RGBBLINK";
const char menuOption3[] PROGMEM = "TIME, SET TIME yyyy mm dd hh mm ss";
const char menuOption4[] PROGMEM = "SET BLINK #, ADD # #, TEMP";
const char menuOption5[] PROGMEM = "EEPROM READ, EEPROM RESET";
const char menuOption6[] PROGMEM = "VERSION";


SimpleDHT22 dht22(pinDHT);
struct TempHumidity {
  byte temperature;
  byte humidity;
};


LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address to 0x27 for a 16 chars and 2 line display
AnalogKeypad keypad(A0);
int currentMenu = 0;       
int menuIndex[] = {0, 0, 0, 0};       
int numOptions[] = {4, 3, 3, 5};
const char* menus[][5] = {
  {"HOME", "HISTORY", "STATS", "SETTINGS"},
  {"NETWORK", "THRESHOLD", "ERASE"},
  {"IP ADDRESS", "SUBNET", "GATEWAY"},
  {"MAJ_U", "MIN_U", "COMFORTABLE", "MIN_O", "MAJ_O"}};


char ipAddress[16] = "192.168.1.21";
int cursorPosition = 0;
bool cursorVisible = true;
bool saveIP = false;
bool doneviewing = false;
int currentHistAddress = 0;
bool doneconfiguring = false;


/**
 * @brief Setup function to initialize hardware and serial communication.
 */
void setup() {
  Ethernet.begin(mac, localip);
  FastLED.addLeds < NEOPIXEL, DATA_PIN > (leds, NUM_LEDS);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledPin12, OUTPUT);
  pinMode(ledPin11, OUTPUT);
  URTCLIB_WIRE.begin();

  rtc.set(00, 00, 00, 00, 00, 00, 00);
  TempHumidity initialreading = readDHT();
  highTemp = initialreading.temperature;
  lowTemp = initialreading.temperature;

  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  displayMenu(currentMenu); 
  
    while (!Serial) {
    ;
  }
  Udp.begin(localPort);
  Serial.print(F("> "));
}

/**
 * @brief Loop function to continuously read input and manage LED states.
 */
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    // handle backspaces
    if (c == '\b' || c == 127) {
      if (inputIndex > 0) {
        inputIndex--;
        Serial.write('\b');
        Serial.write(' ');
        Serial.write('\b');
      }
    } else if (c == '\r' || c == '\n') {

      inputBuffer[inputIndex] = '\0';
      inputIndex = 0;
      Serial.println();
      parseInput(inputBuffer);
    } else {
      if (inputIndex < BUFFER_SIZE - 1) {
        inputBuffer[inputIndex++] = c;
        Serial.write(c);
      }
    }
  }

  if (Power_status){
  leds[1] = CRGB::Green;
  FastLED.show();
  
  UDP_request = false;
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    
    packets_rec = packets_rec+1;
    snprintf(ReplyBuffer, sizeof(ReplyBuffer), "ack");
    UDP_request = true;
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(F("."));
      }
    }
    Serial.print(F("::"));
    Serial.print(Udp.remotePort());
    Serial.print(F(" > "));
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println(packetBuffer);
    parseInput(packetBuffer);

  for (int i = 0; i < packetSize; i++) {
    packetBuffer[i] = 0;
  }

  }
  delay(10);
  }
  
  else{
    leds[1] = CRGB::Black;
    FastLED.show();
  }


  unsigned long curr_millis = millis();

  if (is_Blinking) {
    if (curr_millis - previous_d13_millis >= TIME_INTERVAL) {
      previous_d13_millis = curr_millis;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }

  if (led_is_Blinking) {
    if (curr_millis - previous_led_millis >= TIME_INTERVAL) {
      previous_led_millis = curr_millis;

      if (Green_is_On) {
        greenblink();
      }
      if (Red_is_On) {
        redblink();
      }
      if (!Green_is_On && !Red_is_On && !Last_on_Red && !Last_on_Green) {
        //redblink();
        ledPattern = (ledPattern << 2) | (ledPattern >> 6);

        digitalWrite(ledPin12, ledPattern & 0b01);
        digitalWrite(ledPin11, (ledPattern >> 1) & 0b01);

        /*if (ledPattern == 0b01010101) {
          ledPattern = 0b01100110;
        }*/
      }
      if (!Green_is_On && !Red_is_On && Last_on_Red) {
        redblink();
      }
      if (!Green_is_On && !Red_is_On && Last_on_Green) {
        greenblink();

      }
    }
  }
  if (rgb_is_blinking) {
    if (curr_millis - previous_rgb_millis >= TIME_INTERVAL) {
      previous_rgb_millis = curr_millis;

      leds[0] = leds[0] == CRGB::Black ? CRGB(0, 200, 255) : CRGB::Black;
      
//      for (int i = 0; i < NUM_LEDS; i++) {
//        leds[i] = leds[i] == CRGB::Black ? CRGB(0, 200, 255) : CRGB::Black;
//      }

      FastLED.show();
    }
  }

  if (curr_millis - writepreviousMillis >= write_interval) {
    writepreviousMillis = curr_millis;
    TempHumidity reading = readDHT();
    _temperature = reading.temperature;
    _humidity = reading.humidity;
    if (_temperature > highTemp) {
      highTemp = _temperature;
    }
    if (_temperature < lowTemp) {
      lowTemp = _temperature;
    }
    
    if (_temperature <= temperatureBounds[0]) {
            temperatureRange = Major_Under;
        }
    else if (_temperature <= temperatureBounds[1]) {
            temperatureRange = Minor_Under;
        }
    else if (_temperature <= temperatureBounds[2]) {
            temperatureRange = Comfortable;
        }
    else if (_temperature <= temperatureBounds[3]) {
            temperatureRange = Minor_Over;
        }
    else if(_temperature <= temperatureBounds[4]) {
            temperatureRange = Major_Over;
        }    
    
  }

  if (temperatureRange != previousTemperatureRange) {
        Serial.print(F("State changed to: "));
        switch (temperatureRange) {
            case Major_Under:
                Serial.println(F("Major_Under"));
                leds[2] = CRGB::Purple;
                UDPalarmBuffer("Alarm::Maj_U");
                FastLED.show();
                break;
            case Minor_Under:
                Serial.println(F("Minor_Under"));
                UDPalarmBuffer("Alarm::Min_U");
                leds[2] = CRGB::Blue;
                FastLED.show();
                break;
            case Comfortable:
                Serial.println(F("Comfortable"));
                UDPalarmBuffer("Comfortable");
                leds[2] = CRGB::Green;
                FastLED.show();
                break;
            case Minor_Over:
                Serial.println(F("Minor_Over"));
                UDPalarmBuffer("Alarm::Min_O");
                leds[2] = CRGB::Orange;
                FastLED.show();
                break;
            case Major_Over:
                Serial.println(F("Major_Over"));
                UDPalarmBuffer("Alarm::Maj_O");
                leds[2] = CRGB::Red;
                FastLED.show();
                break;
        }
      previousTemperatureRange = temperatureRange;  
  }

  
    if (Ethernet.linkStatus() == LinkON) {
      leds[3] = CRGB::Green;
      FastLED.show();
  }
  else{
      leds[3] = CRGB::Black;
      FastLED.show();
    }


  if (curr_millis - eeprompreviousMillis >= eeprom_write_interval) {
    eeprompreviousMillis = curr_millis;

    rtc.refresh();
    byte dateTimeArray[8] = { _temperature, _humidity, rtc.year(), rtc.month(), rtc.day(), rtc.hour(), rtc.minute(), rtc.second() };
    storeInputInEEPROM(dateTimeArray);
  
  }

   int pressedKey = keypad.getPressedKey();
  switch (pressedKey) {
    case 2:  // Scroll up
      if (menuIndex[currentMenu] > 0) {
        menuIndex[currentMenu]--;
        displayMenu(currentMenu);
      }
      break;
    case 3:  // Scroll down
      if (menuIndex[currentMenu] < numOptions[currentMenu]) {
        menuIndex[currentMenu]++;
        displayMenu(currentMenu);
      }
      break;
    case 1:  // Back button
      if (currentMenu > 0) {
        currentMenu--;
        displayMenu(currentMenu);
      }
      else
      {
        displayMenu(currentMenu);
        }
      break;
    case 5:  // OK/Confirm
      if (currentMenu == 0 && menuIndex[currentMenu] == 0) {
        performAction(0);  // Execute action for HOME
      } else if (currentMenu > 0 || menuIndex[currentMenu] > 0) {
        performAction(menuIndex[currentMenu]);  // Execute action based on the menu index
      }
      break;
  }


}

/**
 * @brief Parses the input string into individual words and calls executeInput().
 * 
 * @param input The input string to be parsed.
 */

void parseInput(char * input) {
  char words[MAX_COMMAND_SIZE][MAX_COMMAND_SIZE];
  int wordCount = 0;
  int wordLength = 0;

  for (int i = 0; input[i] != '\0'; i++) {
    input[i] = tolower(input[i]);
    if (input[i] == ' ' || input[i] == '\t' || input[i] == '\n') {
      if (wordLength > 0) {
        words[wordCount][wordLength] = '\0';
        wordCount++;
        wordLength = 0;
      }
    } else {
      words[wordCount][wordLength] = input[i];
      wordLength++;
    }
  }

  if (wordLength > 0) {
    words[wordCount][wordLength] = '\0';
    wordCount++;
  }

  executeInput(words, wordCount);
}

/**
 * @brief Executes the commands based on parsed input words.
 * 
 * @param words An array of parsed words.
 * @param size The size of the words array.
 */

void executeInput(char words[][MAX_COMMAND_SIZE], int size) {

  int value = 0;

  for (int i = 0; i < size; i++) {
    int match = 0;
    for (int j = 0; j < LOOKUPTABLE_SIZE; j++) {
      if ((words[i][0] == lookupTable[j][0] && words[i][1] == lookupTable[j][1] && getCharSize(words[i]) == lookupTable[j][2])) {
        int code = lookupTable[j][3];
        tokenbuffer[token_buffer_index] = code;
        token_buffer_index++;
        match = 1;
      }
    }

    if (match == 0) {
      long value = ltoa(words[i]);
      tokenbuffer[token_buffer_index] = value;
      token_buffer_index++;
    }
  }
  switch_cases(tokenbuffer);
  for (int i = 0; i < token_buffer_index; i++) {
    tokenbuffer[i] = 0;
  }
  token_buffer_index = 0;
}

/**
 * @brief Handles different cases based on the parsed command tokens.
 * 
 * @param tokenbuffer An array of command tokens.
 */

void switch_cases(long * tokenbuffer) {

  switch (tokenbuffer[0]) {
  case t_D13:
    switch (tokenbuffer[1]) {
    case t_ON:
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      builtin_led_on(); //Serial.println("D13 is on");  
      break;
    case t_OFF:
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      builtin_led_off(); //Serial.println("D13 is off");  
      break;
    case t_BLINK:
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      is_Blinking = true; //Serial.println("D13 is blinking");
      break;
    default: // Serial.println("D13 Invalid Command");
      break;
    }
    break;

  case t_LED:
    switch (tokenbuffer[1]) {
    case t_GREEN:
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      led_green(); // Serial.println("LED is Green"); 
      break;
    case t_RED:
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      led_red(); //Serial.println("LED is Red"); 
      break;
    case t_OFF:
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      off(); //Serial.println("LED is Off");
      break;
    case t_BLINK:
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      led_is_Blinking = true; //Serial.println("LED is Blinking"); 
      break;
    default: //Serial.println("LED Invalid Command"); 
      break;
    }
    break;

  case t_SET:
    switch (tokenbuffer[1]) {
    case t_BLINK:
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      if (tokenbuffer[2] && tokenbuffer[2] >= 0 && tokenbuffer[2] <= 65535) {
        TIME_INTERVAL = tokenbuffer[2];
        //Serial.println("Sucessfully updated TIME_INTERVAL for blink");
      }
      //else {Serial.println("Error: valid range (0-65535)/ms");}
      break;

    case t_TIME:
      for (int i = 1; i == 6; i++) {
        Serial.println(tokenbuffer[i]);
      }
      rtc.set(tokenbuffer[7], tokenbuffer[6], tokenbuffer[5], rtc.dayOfWeek(), tokenbuffer[4], tokenbuffer[3], tokenbuffer[2] - 2000);
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      Serial.println(F("time is set!"));
      break;
    default:
      /*Serial.println("Set Invalid Command");*/ break;
    }
    break;

  case t_STATUS:
    switch (tokenbuffer[1]) {
    case t_LEDS:
      /*state()*/
      int d13State = digitalRead(LED_BUILTIN);
      int greenState = digitalRead(ledPin12);
      int redState = digitalRead(ledPin11);
      snprintf(ReplyBuffer, sizeof(ReplyBuffer), "D13:%d G:%d R:%d Blink:%d", d13State,greenState,redState,TIME_INTERVAL);
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      Serial.println(ReplyBuffer);
      clearReplyBuffer();
      break;

    default:
    //Serial.println("Status Invalid command");
      break;
    }
    break;

  case t_TIME:
    rtc.refresh();
    snprintf(ReplyBuffer, sizeof(ReplyBuffer), "%d/%d/%d %d:%d:%d", rtc.year(),rtc.month(),rtc.day(),rtc.hour(),rtc.minute(),rtc.second());
    if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
    /*printDateTime();*/
    Serial.println(ReplyBuffer);
    clearReplyBuffer();
    break;

  case t_ADD:
    if (tokenbuffer[1] && tokenbuffer[2]) {
      snprintf(ReplyBuffer, sizeof(ReplyBuffer), "%d",tokenbuffer[1] + tokenbuffer[2] );
      if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
      Serial.println(ReplyBuffer);
      clearReplyBuffer();
    }
    break;
    
  case t_RGB:
    rgb_on(tokenbuffer[1], tokenbuffer[2], tokenbuffer[3]);
    if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
    break;

  case t_RGBBLINK:
    rgb_is_blinking = true;
    if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
    break;

  case t_TEMP:
    rtc.refresh();
    snprintf(ReplyBuffer, sizeof(ReplyBuffer), "Temp: %dF Hum:%d%% H:%d L:%d",_temperature,_humidity,highTemp,lowTemp);
    if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
    Serial.println(ReplyBuffer);
    clearReplyBuffer();
    break;
  /*
    Serial.print("Temp: ");
    Serial.print(_temperature);
    Serial.print("C  Hum: ");
    Serial.print(_humidity);
    Serial.println("%");
    Serial.print("      H:");
    Serial.print(highTemp);
    Serial.print("  L:");
    Serial.println(lowTemp);
    break;
  */
  case t_EEPROM:
    switch (tokenbuffer[1]) {
    case t_READ:
      readEEPROMData();
      break;
    case t_RESET:
      clearEEPROM(); if (UDP_request){UDPreplyBuffer(ReplyBuffer);} break;
    default:
    //Serial.println("Status Invalid command");
      break;
    }break;

    
  case t_POWER:
    switch (tokenbuffer[1]) {
      case t_ON:
        Power_status = true; break;
      case t_OFF:
        Power_status = false;   break;
      default: break;
    }
    break;

  case t_VERSION:
    snprintf(ReplyBuffer, sizeof(ReplyBuffer), "v2.0");
    if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
    Serial.println(ReplyBuffer);
    clearReplyBuffer();
    break;

  case t_HELP:
    snprintf(ReplyBuffer, sizeof(ReplyBuffer), "ack");
    if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
    printHelpMenu();
    clearReplyBuffer();
    break;

  default:
    snprintf(ReplyBuffer, sizeof(ReplyBuffer), "Invalid cmd");
    if (UDP_request){UDPreplyBuffer(ReplyBuffer);}
    Serial.println(ReplyBuffer);
    clearReplyBuffer();
    break;


  }
}

/*********** led functions ***********/

void builtin_led_on() {
  is_Blinking = false;
  digitalWrite(LED_BUILTIN, HIGH);
}

void builtin_led_off() {
  is_Blinking = false;
  digitalWrite(LED_BUILTIN, LOW);
}

void rgb_on(int R, int G, int B) {
  rgb_is_on = true;
  rgb_is_blinking = false;
  
  leds[0] = CRGB(R, G, B);
  FastLED.show();
  
//    for (int i = 0; i < NUM_LEDS; i++) {
//    leds[i] = CRGB(R, G, B);
//    FastLED.show();
//  }
}

void led_green() {
  Green_is_On = true;
  Red_is_On = false;
  led_is_Blinking = false;
  Last_on_Red = false;
  Last_on_Green = true;
  digitalWrite(ledPin12, HIGH);
  digitalWrite(ledPin11, LOW);

}

void led_red() {
  Red_is_On = true;
  Green_is_On = false;
  led_is_Blinking = false;
  Last_on_Red = true;
  Last_on_Green = false;
  digitalWrite(ledPin12, LOW);
  digitalWrite(ledPin11, HIGH);

}

void off() {
  led_is_Blinking = false;
  Red_is_On = false;
  Green_is_On = false;
  digitalWrite(ledPin12, LOW);
  digitalWrite(ledPin11, LOW);
}

/*
void state() {
  int d13State = digitalRead(LED_BUILTIN);
  int greenState = digitalRead(ledPin12);
  int redState = digitalRead(ledPin11);

  Serial.print("D13: ");
  Serial.println(d13State);
  Serial.print("Green: ");
  Serial.println(greenState);
  Serial.print("Red: ");
  Serial.println(redState);
  Serial.print("Blink: ");
  Serial.println(TIME_INTERVAL);
}
*/

/**
 * @brief Prints the help menu, listing available commands.
*/

void printHelpMenu() {
Serial.println(reinterpret_cast<const __FlashStringHelper *>(menuOption1));
Serial.println(reinterpret_cast<const __FlashStringHelper *>(menuOption2));
Serial.println(reinterpret_cast<const __FlashStringHelper *>(menuOption3));
Serial.println(reinterpret_cast<const __FlashStringHelper *>(menuOption4));
Serial.println(reinterpret_cast<const __FlashStringHelper *>(menuOption5));
Serial.println(reinterpret_cast<const __FlashStringHelper *>(menuOption6));
}

/**
 * @brief Prints the help menu, listing available commands.
 */
void redblink() {
  digitalWrite(ledPin11, !digitalRead(ledPin11));
  digitalWrite(ledPin12, LOW);
}

/**
 * @brief Blinks the green LED.
 */
void greenblink() {
  digitalWrite(ledPin11, LOW);
  digitalWrite(ledPin12, !digitalRead(ledPin12));
}

/**
 * @brief Returns the size of a character string.
 * 
 * @param str The input character string.
 * @return The size of the input string.
 */
int getCharSize(const char * str) {
  int size = 0;
  while (str[size] != '\0') {
    size++;
  }
  return size;
}

/**
 * @brief Converts a character string to a long integer.
 * 
 * @param str The input character string.
 * @return The converted long integer value.
 */
long ltoa(const char * str) {
  long result = 0;
  int sign = 1;
  int i = 0;

  if (str[i] == '-') {
    sign = -1;
    i++;
  }
  while (str[i] != '\0') {
    if (isdigit(str[i])) {
      result = result * 10 + (str[i] - '0');
    } else {
      break;
    }
    i++;
  }
  return result * sign;
}


/**
 * @brief Print Current Date and Time
 */
/*void printDateTime() {
  Serial.print(rtc.year());
  Serial.print('/');
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.day());
  Serial.print(' ');
  Serial.print(rtc.hour());
  Serial.print(':');
  Serial.print(rtc.minute());
  Serial.print(':');
  Serial.print(rtc.second());
  Serial.println();
} */



/**
 * @brief Read Temp and Humidity
 */
TempHumidity readDHT() {
  TempHumidity data;
  int err = SimpleDHTErrSuccess;

  if ((err = dht22.read( & data.temperature, & data.humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print(F("Read DHT22 failed, err="));
    Serial.print(SimpleDHTErrCode(err));
    Serial.print(F(","));
    Serial.println(SimpleDHTErrDuration(err));
  }
  data.temperature = (data.temperature * 9/5) + 32;
  return data;
}

/**
 * @brief Write into EEPROM
 */
void writeEntry(int address, const byte * entry, int entry_size) {
  for (int i = 0; i < entry_size ; i++) {
    EEPROM.write(address + i, entry[i]);
  }
}


/**
 * @brief Read from EEPROM
 */
void readEntry(int address, byte * entry, int entry_size) {
  for (int i = 0; i < entry_size; i++) {
    entry[i] = EEPROM.read(address + i);
  }
}


/**
 * @brief Store the MAX_ENTRY bytes of data into EEPROM
 * @param data temperature humidity and timestamp in bytes
 */
void storeInputInEEPROM(const byte * data) {
  int lastStoredAddress = EEPROM.read(EEPROM_LAST_ADDRESS);
  writeEntry(lastStoredAddress, data,ENTRY_SIZE );
  nextAddress = (lastStoredAddress + ENTRY_SIZE) % EEPROM_SIZE;
  EEPROM.write(EEPROM_LAST_ADDRESS, nextAddress);
  Serial.println(F("EEPROM write..."));
}

/**
 * @brief Read all the stored form EEPROM
 */
void readEEPROMData() {
  int lastStoredAddress = EEPROM.read(EEPROM_LAST_ADDRESS);
  int currentAddress = EEPROM_START_ADDRESS;

  Serial.print(F("C  %  time")); 
  char entryStr[ENTRY_SIZE * 4 + ENTRY_SIZE / 8 + 1];

  if (lastStoredAddress == 0 && currentAddress == 0){
    snprintf(ReplyBuffer, sizeof(ReplyBuffer), "empty");
    if (UDP_request){UDPreplyBuffer(ReplyBuffer); clearReplyBuffer();}
  }
  while (currentAddress != lastStoredAddress) {
    byte entry[ENTRY_SIZE];
    readEntry(currentAddress, entry, ENTRY_SIZE);
    char lineBuffer[ENTRY_SIZE * 4 + ENTRY_SIZE / 8 + 1];
    int offset = 0;

    for (int i = 0; i < ENTRY_SIZE; i++) {
      offset += snprintf(lineBuffer + offset, sizeof(lineBuffer) - offset, "%d ", entry[i]);
      if ((i + 1) % 8 == 0) {
        Serial.println();
      }
    }
    if (UDP_request){UDPreplyBuffer(lineBuffer);}
    Serial.print(lineBuffer);

    currentAddress = (currentAddress + ENTRY_SIZE) % EEPROM_SIZE;
  }
  Serial.println();
  
}


/**
 * @brief CLear EEPROM Memory
 */
void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.update(i, 0);
  }
  Serial.println(F("mem cleared"));
}


void UDPreplyBuffer(char* str){
    packets_sent = packets_sent + 1;
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(str);
    Udp.endPacket();
}

void UDPalarmBuffer(char* str){
    packets_sent = packets_sent + 1;
    Udp.beginPacket(remoteip,remotePort);
    Udp.write(str);
    Udp.endPacket();
}

void clearReplyBuffer(){
   for (int i = 0; i < BUFFER_SIZE; i++) {
    ReplyBuffer[i] = 0;
  }
}


void displayMenu(int currentMenu) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(menus[   currentMenu  ][   menuIndex[currentMenu]  ]);
  
  lcd.setCursor(0, 1);
  if (currentMenu > 0) {
    lcd.print("< BACK   ");
  }
  lcd.print("> ENTER");
}


void performAction(int action) {
  // Handle actions based on the selected menu and submenu
  switch (currentMenu) {
    case 0:
      // Handle actions for top-level menu options
      switch (action) {
        case 0:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print( "Temp:");
          lcd.print( _temperature);
          lcd.print( "C ");
          lcd.print( "Hum:");
          lcd.print( _humidity);
          lcd.print( "%");
          break;
        case 1:
         DisplayScrollHist(currentHistAddress); 
          historyloop();
          currentMenu = 0;       
          displayMenu(currentMenu);
               
          break;
        case 2:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Packets Sent ");
          lcd.print(packets_sent);
          lcd.setCursor(0, 1);
          lcd.print("Packets Rec ");
           lcd.print(packets_rec);
          break;
        case 3:
          // Navigate to the "SETTINGS" submenu
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Settings");  
          currentMenu = 1;       // Index of the current menu level
          displayMenu(currentMenu);
          break;
      }
      break;
    case 1:
      // Handle actions for settings sub options
      switch (action) {
        case 0:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Settings");  
          currentMenu = 2;       // Index of the current menu level
          displayMenu(currentMenu);
          break;
        case 1:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Settings");  
          currentMenu = 3;       // Index of the current menu level
          displayMenu(currentMenu);
          break;
        case 2:
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("history erased");
          clearEEPROM();
          
          break;
      }
      break;

    
    case 2:
      // Handle actions for network settings options
      switch (action) {
        case 0:
          lcd.clear();
          readEntry(976,ipAddress,16 );
          //Serial.println(ipAddress);
          networkloop();
          writeEntry(976, ipAddress, sizeof(ipAddress));
          //Serial.println(ipAddress);
          currentMenu = 1;       
          displayMenu(currentMenu);
          break;
        case 1:
          lcd.clear();
          readEntry(991,ipAddress,16 );
          //Serial.println(ipAddress);
          networkloop();
          writeEntry(991, ipAddress, sizeof(ipAddress));
          //Serial.println(ipAddress);
          currentMenu = 1;       
          displayMenu(currentMenu);
          break;
        case 2:
          lcd.clear();
          readEntry(1007,ipAddress,16 );
          networkloop();
          writeEntry(1007, ipAddress, sizeof(ipAddress));
          currentMenu = 1;       
          displayMenu(currentMenu);
          break;
      }
      break;

    case 3:
    
      switch (action) {
        case 0:
        DisplayThreshold(0);
        Thresholdloop(0);
        currentMenu = 1;       
        displayMenu(currentMenu);
        break;

        case 1:
        DisplayThreshold(1);
        Thresholdloop(1);
        currentMenu = 1;       
        displayMenu(currentMenu);
        break;

        case 2:
        DisplayThreshold(2);
        Thresholdloop(2);
        currentMenu = 1;       
        displayMenu(currentMenu);
        break;

        case 3:
        DisplayThreshold(3);
        Thresholdloop(3);
        currentMenu = 1;       
        displayMenu(currentMenu);
        break;

        case 4:
        DisplayThreshold(4);
        Thresholdloop(4);
        currentMenu = 1;       
        displayMenu(currentMenu);
        break;

      }
      break;
  }
}


void networkloop(){
  
  lcd.print(F("Address:"));
  lcd.setCursor(0, 1);
  lcd.print(ipAddress);

while(!saveIP){
  int pressedKeysub = keypad.getPressedKey();

  if (pressedKeysub >= 1 && pressedKeysub <= 5) {
    updateAddress(pressedKeysub);
  }

  // Toggle cursor visibility every 500 milliseconds (adjust as needed)
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;
    cursorVisible = !cursorVisible;
    updateDisplay();
  }

}
saveIP = false;
}

void updateAddress(int key) {
  if (cursorPosition < 15) {
    if (key == 2) { // Up key
      if (ipAddress[cursorPosition] == '9') {
        ipAddress[cursorPosition] = '.';
      } else {
        ipAddress[cursorPosition]++;
      }
    } else if (key == 3) { // Down key
      if (ipAddress[cursorPosition] == '0') {
        ipAddress[cursorPosition] = '.';
      } else {
        ipAddress[cursorPosition]--;
      }
    } else if (key == 1) { // Left key
      cursorPosition--;
      if (cursorPosition < 0) {
        cursorPosition = 0;
      }
    } else if (key == 4) { // Right key
      cursorPosition++;
      if (cursorPosition > 15) {
        cursorPosition = 15;
      }
    } else if (key == 5) { // OK key
      if (cursorPosition == 15) {
        ipAddress[cursorPosition] = '.';
        cursorPosition = 0;
      } else {
        saveIP = true;
        Serial.println(saveIP);
      }
    }
  }
}

void updateDisplay() {
  lcd.setCursor(0, 1);
  lcd.print(F("                ")); // Clear the line
  lcd.setCursor(0, 1);
  lcd.print(ipAddress);

  if (cursorVisible) {
    lcd.setCursor(cursorPosition, 1);
    lcd.write('_'); // Display an underscore to indicate cursor position
  }
}


void historyloop(){

while(!doneviewing){
  int pressedKeysub = keypad.getPressedKey();

  if (pressedKeysub >= 1 && pressedKeysub <= 5) {
    updatehistory(pressedKeysub);
  }
}
doneviewing = false;
}


void updatehistory(int key) {
    
     if (key == 2) {
       if (currentHistAddress > ENTRY_SIZE && currentHistAddress < EEPROM_SIZE ){
          currentHistAddress = currentHistAddress - ENTRY_SIZE;
          DisplayScrollHist(currentHistAddress);
        }
      } 
      
    else if (key == 3) { 
       if (currentHistAddress >= 0 && currentHistAddress < EEPROM_SIZE ){
        currentHistAddress = currentHistAddress + ENTRY_SIZE;
          DisplayScrollHist(currentHistAddress);
        }
    } 
      else if (key == 1) { // OK key
        doneviewing = true;
      }
    }

  void DisplayScrollHist(int currentHistAddress) {
  lcd.clear();
  lcd.print(F("F  %  time"));
  lcd.setCursor(0, 1);
  
  for (int i = 0; i < ENTRY_SIZE; i++) {
    byte dataByte = EEPROM.read(currentHistAddress + i);
    lcd.print(dataByte);
    lcd.print(" ");
  }
}


void Thresholdloop(int index){
while(!doneconfiguring){
  int pressedKeysub = keypad.getPressedKey();

  if (pressedKeysub >= 1 && pressedKeysub <= 5) {
    updateThreshold(pressedKeysub, index);
  }
}
doneconfiguring = false;
}


void updateThreshold(int key, int index) {
    
     if (key == 2) {
       if (temperatureBounds[index] >=0 && temperatureBounds[index]<256){
         temperatureBounds[index] = temperatureBounds[index] + 1;
         DisplayThreshold(index);
        }
      } 
      
    else if (key == 3) { 
       if (temperatureBounds[index] >=0 && temperatureBounds[index]<256){
         temperatureBounds[index] = temperatureBounds[index] - 1;
         DisplayThreshold(index);
        }
    } 
      else if (key == 5) { // OK key
        doneconfiguring = true;

      }
    }

  void DisplayThreshold(int index) {
  lcd.clear();
  lcd.print(F("Upper Bound: "));
  lcd.setCursor(0, 1);
  Serial.print(temperatureBounds[index]);
  lcd.print(temperatureBounds[index]);
  }
