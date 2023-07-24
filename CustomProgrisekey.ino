//#include "mp3tf16p.h"
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"

#define SW_0 2
#define SW_1 3
#define SW_2 4
#define TX 10
#define RX 11

#define BTNS 3

#define ACTIVATION 1
#define ABILITY_1 2
#define EXTRA_ATTACK 3
#define AUTHORIZE 4
#define DRIVER_HENSHIN 5
#define DRIVER_ATTACK 6
#define SHOTRISE_HENSHIN 7
#define SHOTRISE_LOOP 8
#define SHOTRISE_ATTACK 9

#define STATE_IDLE_A             1 /* !alternate_activation ? ON_OFF_OFF --> OFF_OFF_OFF */
#define STATE_IDLE_B             2 /*  alternate_activation ? ON_OFF_OFF --> OFF_OFF_OFF */

#define STATE_WEAPON_RDY         3
#define STATE_WEAPON_ACTIVE      4 /* OFF_ON_ON */
#define STATE_WEAPON_ATTACK      5 /* OFF_OFF_ON */

#define STATE_FORCE_READY        6 /* STATE_IDLE_A || STATE_IDLE_B ? OFF_OFF_ON */
#define STATE_FORCE_ACTIVE       7 /* OFF_ON_ON --> OFF_OFF_ON */
#define STATE_FORCE_FINISH_RDY   8 /* OFF_OFF_ON */
#define STATE_FORCE_ATTACK       9 /* OFF_ON_ON --> OFF_OFF_ON */

#define STATE_AUTH_A            10 /* ON_OFF_OFF --> !alternate_activation */
#define STATE_AUTH_B            11 /* ON_OFF_OFF -->  alternate_activation */

#define STATE_DRIVER_READY      12 /* OFF_ON_OFF --> authorized */
#define STATE_DRIVER_ACTIVE     13 /* OFF_ON_ON --> OFF_ON_OFF */
#define STATE_DRIVER_FINISH_RDY 14 /* OFF_ON_OFF */
#define STATE_DRIVER_ATTACK     15 /* OFF_ON_ON --> OFF_ON_OFF */

#define STATE_SHOT_READY        16 /* OFF_OFF_ON --> authorized */
#define STATE_SHOT_ACTIVE       17 /* OFF_ON_ON --> OFF_OFF_ON */
#define STATE_SHOT_FINISH_RDY   18 /* OFF_OFF_ON --> ON_OFF_ON */
#define STATE_SHOT_FINISH_WAT   19 /* OFF_OFF_ON
#define STATE_SHOT_ATTACK       20 /* OFF_ON_ON --> OFF_OFF_ON */

SoftwareSerial ss(TX,RX);
DFRobotDFPlayerMini mp3;
//MP3Player mp3(11,10);

const uint8_t OFF_OFF_OFF[] = {LOW, LOW, LOW};
const uint8_t ON_OFF_OFF[] = {HIGH, LOW, LOW};
const uint8_t OFF_ON_OFF[] = {LOW, HIGH, LOW};
const uint8_t OFF_OFF_ON[] = {LOW, LOW, HIGH};
const uint8_t OFF_ON_ON[] = {LOW, HIGH, HIGH};
const uint8_t ON_ON_OFF[] = {HIGH, HIGH, LOW};
const uint8_t ON_OFF_ON[] = {HIGH, LOW, HIGH};
const uint8_t ON_ON_ON[] = {HIGH, HIGH, HIGH};

uint8_t prev_sw[] = {LOW, LOW, LOW};
uint8_t sw[] = {LOW, LOW, LOW};
uint8_t state = STATE_IDLE_A;
uint8_t prev_state = STATE_IDLE_A;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(SW_0, INPUT);
  pinMode(SW_1, INPUT);
  pinMode(SW_2, INPUT);
  /*
    Serial.println(ACTIVATION);
    Serial.println(ABILITY_1);
    Serial.println(EXTRA_ATTACK);
    Serial.println(AUTHORIZE);
    Serial.println(DRIVER_HENSHIN);
    Serial.println(DRIVER_ATTACK);
    Serial.println(SHOTRISE_HENSHIN);
    Serial.println(SHOTRISE_LOOP);
    Serial.println(SHOTRISE_ATTACK);
  */
  ss.begin(9600);
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!mp3.begin(ss, true, false)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  
  mp3.setTimeOut(500); //Set serial communictaion time out 500ms
  mp3.volume(25);  //Set volume value (0~30).
  mp3.play(AUTHORIZE);
  //mp3.initialize();
  //mp3.playTrackNumber(ACTIVATION, 20);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long now = millis();

  sw[0] = digitalRead(SW_0);
  sw[1] = digitalRead(SW_1);
  sw[2] = digitalRead(SW_2);

  if(memcmp(prev_sw, OFF_OFF_OFF, BTNS) == 0) {
    if(memcmp(sw, OFF_OFF_OFF, BTNS) == 0) {
      // Reset
    }
    else if(memcmp(sw, ON_OFF_OFF, BTNS) == 0) {
      // Activate top button
      Serial.print(F("SW_0          "));
      if(state == STATE_IDLE_A)
        state = STATE_IDLE_B;
      else if(state == STATE_IDLE_B)
        state = STATE_IDLE_A;
      else {
        Serial.print(F("Unreachable STATE_IDLE -- "));
        Serial.print(prev_state); Serial.print(F(" | "));
        Serial.print(state);
        Serial.println();
      }
    } 
    else if(memcmp(sw, OFF_ON_OFF, BTNS) == 0) {
      // Driver Mode (Amazing Caucasus, Zero-Two, Ark-One, Triceratops)
      if(state == STATE_IDLE_B)
        state = STATE_DRIVER_READY;

    } 
    else if(memcmp(sw, OFF_OFF_ON, BTNS) == 0) {
      // Forceriser Standby Mode
      Serial.println(F("          SW_2"));
      if(state == STATE_IDLE_A)
        state = STATE_FORCE_READY;
    } 
    else if(memcmp(sw, ON_ON_OFF, BTNS) == 0) {
      // Blank
    } 
    else if(memcmp(sw, ON_OFF_ON, BTNS) == 0) {
      // Blank
    } 
    else if(memcmp(sw, OFF_ON_ON, BTNS) == 0) {
      // Weapon Mode
      if(state == STATE_IDLE_A || state == STATE_IDLE_B) {
        state = STATE_WEAPON_RDY;
      } else 
        Serial.println(F("Unreachable OFF_OFF_OFF -> OFF_ON_ON state")); 
    } 
    else if(memcmp(sw, ON_ON_ON, BTNS) == 0) {
      // Blank
    }
    else {
      Serial.println(F("Unreachable OFF_OFF_OFF condition"));
    }
  } 
  else if(memcmp(prev_sw, ON_OFF_OFF, BTNS) == 0) {
    if(memcmp(sw, OFF_OFF_OFF, BTNS) == 0) {
      // Reset State
    } 
    else if(memcmp(sw, ON_OFF_OFF, BTNS) == 0) {
      // Holding down SW_0
    } 
    else if(memcmp(sw, OFF_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_ON_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_ON_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else {
      Serial.println(F("Unreachable ON_OFF_OFF condition"));
    }
  } 
  else if(memcmp(prev_sw, OFF_ON_OFF, BTNS) == 0) {
    if(memcmp(sw, OFF_OFF_OFF, BTNS) == 0) {
      // Reset Item
      Reset();
      mp3.stop();
    } 
    else if(memcmp(sw, ON_OFF_OFF, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, OFF_ON_OFF, BTNS) == 0) {
      // Holding down SW_1
      
    } 
    else if(memcmp(sw, OFF_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, ON_ON_OFF, BTNS) == 0) {
      // Special functionality -- Activate Finisher Charge (ASSAULT CHARGE, FINISH MODE)
      Serial.println(F("SW_0 SW_1     "));
    } 
    else if(memcmp(sw, ON_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, OFF_ON_ON, BTNS) == 0) {
      // Driver Activation -- Henshin or Finisher
      if(state == STATE_DRIVER_READY)
        state = STATE_DRIVER_ACTIVE;
      else if(state = STATE_DRIVER_FINISH_RDY)
        state = STATE_DRIVER_ATTACK;
    } 
    else if(memcmp(sw, ON_ON_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else {
      Serial.println(F("Unreachable OFF_ON_OFF condition"));
    }
  } 
  else if(memcmp(prev_sw, OFF_OFF_ON, BTNS) == 0) {
    if(memcmp(sw, OFF_OFF_OFF, BTNS) == 0) {
      // Reset Item
      Reset();
      mp3.stop();
    } 
    else if(memcmp(sw, ON_OFF_OFF, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, OFF_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, OFF_OFF_ON, BTNS) == 0) {
      // Holding down SW_2
    } 
    else if(memcmp(sw, ON_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, ON_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, OFF_ON_ON, BTNS) == 0) {
      // Driver Activation -- Henshin or Finisher
      if(state == STATE_WEAPON_ATTACK) { 
        state = STATE_WEAPON_ACTIVE;
      } else if(state == STATE_FORCE_READY)
        state = STATE_FORCE_ACTIVE;
      else if(state == STATE_FORCE_FINISH_RDY) {
        state = STATE_FORCE_ATTACK;
      } else 
        Serial.println(F("Unreachable OFF_OFF_ON -> OFF_ON_ON state"));
    } 
    else if(memcmp(sw, ON_ON_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else {
      Serial.println(F("Unreachable OFF_OFF_ON condition"));
    }
  } 
  else if(memcmp(prev_sw, ON_ON_OFF, BTNS) == 0) {
    if(memcmp(sw, OFF_OFF_OFF, BTNS) == 0) {
      // Reset Item
      Reset();
      mp3.stop();
    } 
    else if(memcmp(sw, ON_OFF_OFF, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, OFF_ON_OFF, BTNS) == 0) {
      // Finisher Standby State
      ;
    } 
    else if(memcmp(sw, OFF_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, ON_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, ON_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, OFF_ON_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else if(memcmp(sw, ON_ON_ON, BTNS) == 0) {
      // Blank
      ;
    } 
    else {
      Serial.println(F("Unreachable ON_ON_OFF condition"));
    }
  } 
  else if(memcmp(prev_sw, ON_OFF_ON, BTNS) == 0) {
    if(memcmp(sw, OFF_OFF_OFF, BTNS) == 0) {
      // Reset State
      Reset();
      mp3.stop();
    }
    else if(memcmp(sw, ON_OFF_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_ON_ON, BTNS) == 0) {
      // Driver Activation -- Henshin or Finisher
    }
    else if(memcmp(sw, ON_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_ON_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else {
      Serial.println(F("Unreachable ON_OFF_ON condition"));
    }
  } 
  else if(memcmp(prev_sw, OFF_ON_ON, BTNS) == 0) {
    if(memcmp(sw, OFF_OFF_OFF, BTNS) == 0) {
      // Reset
      Reset();
      mp3.stop();
    }
    else if(memcmp(sw, ON_OFF_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_ON_OFF, BTNS) == 0) {
      // Driver Standby State
      if(state == STATE_DRIVER_ACTIVE || state == STATE_DRIVER_ATTACK)
        state = STATE_DRIVER_FINISH_RDY;
    }
    else if(memcmp(sw, OFF_OFF_ON, BTNS) == 0) {
      // Other Driver Standby State
      if(state == STATE_WEAPON_RDY || state == STATE_WEAPON_ACTIVE)
        state = STATE_WEAPON_ATTACK;
      else if(state == STATE_FORCE_ACTIVE || state == STATE_FORCE_ATTACK)
        state = STATE_FORCE_FINISH_RDY;
      else 
        Serial.println(F("Unreachable OFF_ON_ON -> OFF_OFF_ON state"));
    }
    else if(memcmp(sw, OFF_ON_ON, BTNS) == 0) {
      // SW_1 and SW_2 held down
    }
    else if(memcmp(sw, ON_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_ON_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else {
      Serial.println(F("Unreachable OFF_ON_ON condition"));
    }
  } 
  else if(memcmp(prev_sw, ON_ON_ON, BTNS) == 0) {
    if(memcmp(sw, OFF_OFF_OFF, BTNS) == 0) {
      // Reset State
      Reset();
      mp3.stop();
    }
    else if(memcmp(sw, ON_OFF_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, OFF_ON_ON, BTNS) == 0) {
      // Special Functionality Standby State
      ;
    }
    else if(memcmp(sw, ON_ON_OFF, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_OFF_ON, BTNS) == 0) {
      // Blank
      ;
    }
    else if(memcmp(sw, ON_ON_ON, BTNS) == 0) {
      // All Held Down
      Serial.println(F("SW_0 SW_1 SW_2"));
    }
    else {
      Serial.println(F("Unreachable ON_ON_ON condition"));
    }
  } 
  else {
    Serial.println(F("Unreachable Previous Switch Condition"));
  }

  playSound();

  /*if(prev_sw[0] == LOW && prev_sw[1] == LOW) {
    if(sw[0] == HIGH && sw[1] == LOW) {
      Serial.println("SW0");
      if(alternate_activation) {
        mp3.play(ABILITY_1);
        alternate_activation = false;
      } else if (!alternate_activation) {
        mp3.play(ACTIVATION);
        alternate_activation = true;
      }
    } else if(sw[0] == LOW && sw[1] == HIGH) {
      Serial.println("SW1");
      mp3.play(SHOTRISE_LOOP);
    }
  }*/

  if (mp3.available()) {
    //printDetail(mp3.readType(), mp3.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  prev_sw[0] = sw[0];
  prev_sw[1] = sw[1];
  prev_sw[2] = sw[2];
  prev_state = state;

  delay(20);
}

void Reset() {
  state = STATE_IDLE_A;
  Serial.println(F("RESET"));
}

void playSound() {
  if(prev_state == state)
    return;
  switch(state) {
    case STATE_IDLE_A:
      if(prev_state == STATE_IDLE_B) {
        Serial.print(prev_state); Serial.print(state);
        Serial.println(F(" | CAUCASUS'S ABILITY"));
        mp3.play(ABILITY_1);
      }
      break;
    case STATE_IDLE_B:
      if(prev_state == STATE_IDLE_A) {
        Serial.print(prev_state); Serial.print(state);
        Serial.println(F(" | BREAKHORN"));
        mp3.play(ACTIVATION);
      }
      break;
    case STATE_WEAPON_RDY:
      Serial.print(prev_state); Serial.print(state);
      Serial.println(F(" | PROGRISEKEY CONFIRMED"));
      mp3.play(ABILITY_1);
      break;
    case STATE_WEAPON_ACTIVE:
      break;
    case STATE_WEAPON_ATTACK:
      if(prev_state == STATE_WEAPON_ACTIVE || prev_state == STATE_WEAPON_RDY) {
        Serial.print(prev_state); Serial.print(state);
        Serial.println(F(" | THOUSAND CONTAINED STRASH"));
        mp3.play(EXTRA_ATTACK);
      }
      break;
    case STATE_FORCE_ACTIVE:
      if(prev_state == STATE_FORCE_READY) {
        Serial.print(prev_state); Serial.print(state);
        Serial.println(F(" | FORCERISE "));
        mp3.play(SHOTRISE_HENSHIN);
      }
      break;
    case STATE_FORCE_ATTACK:
      if(prev_state == STATE_FORCE_FINISH_RDY) {
        Serial.print(prev_state); Serial.print(state);
        Serial.println(F(" | DYSTOPIA"));
        mp3.play(EXTRA_ATTACK);
      }
      break;
    case STATE_DRIVER_ACTIVE:
      if(prev_state == STATE_DRIVER_READY) {
        Serial.print(prev_state); Serial.print(state);
        Serial.println(F(" | PERFECTRISE"));
        mp3.play(DRIVER_HENSHIN);
      }
      break;
    case STATE_DRIVER_ATTACK:
      if(prev_state == STATE_DRIVER_FINISH_RDY) {
        Serial.print(prev_state); Serial.print(state);
        Serial.println(F(" | DESTRUCTION"));
        mp3.play(DRIVER_ATTACK);
      }
      break;
    default:
      break;
  }
}

void stopMusic() {
  
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
      default:
      break;
    }
  }