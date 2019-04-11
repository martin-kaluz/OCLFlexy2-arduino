#include <OCL_Flexy2.h>
#include "Adafruit_LEDBackpack.h"

OCL_Flexy2 flexy;   // Create an instance of OCL_Flexy2

float f = 10.0;            // sampling frequency [Hz]
float fd = 10.0;           // update frequency of display [Hz] 
float bend;
int fpfd = round(f/fd);
volatile boolean flag1=false;
boolean flag2=false;
volatile uint16_t counter2 = 0;
uint16_t hold_counter = 0;
boolean hold = false; 
boolean filt = false;
boolean printData = false;
const char startChar = '<';
const char endChar = '>';
const byte buffSize = 8;
char command;
uint8_t cmdVal;
char buff[buffSize];
byte numOfBytes = 0;
char actByte;
boolean reading = false;
boolean finishedReading = false;
char message[9];
int pot1_old;
int pot2_old;
int knob;
char display_buffer[3] = {' ',' ',' '};
boolean left_right = false;
float rand_noise_multiplier = 0.0;
float filter_weight = 0.0;

Adafruit_AlphaNum4 display = Adafruit_AlphaNum4();

void setup() {  
  flexy.begin();
  pot1_old = flexy.getUserInputLPerc();
  pot2_old = flexy.getUserInputRPerc();

  Serial.begin(115200);
  setup_realtime_execution(f);              // setup the timer for boolean variable 'flag1'
  display.begin(0x70);
  display.clear(); 
  if(Serial){
    strncpy(message, "FLEX", 4);
    print2Display(message,display);
    delay(750);
    strncpy(message, "LEXY", 4);
    print2Display(message,display);
    delay(750);
    strncpy(message, "v2.0.0", 6);
    print2Display(message,display);
    delay(1000);
  }
}

void loop() {
  updateDisplay();
  readCommand();
  processCommand();
  printDataFun();
}

void readCommand(){
  if(Serial.available()>0){
    actByte = Serial.read();
    if(actByte == endChar){
      reading = false;
      finishedReading = true;
      buff[numOfBytes] = 0;
      parseCommand();
      }
    
    if(reading){
      buff[numOfBytes] = actByte;
      numOfBytes++;
      }
    
    if(actByte == startChar){
      numOfBytes = 0;
      reading = true;
      }
    }
  }

void parseCommand(){
  if(finishedReading){
    char *token = strtok(buff,":");
    command = token[0];
    token = strtok(NULL,":");
    cmdVal = atoi(token);
    }
  }

void processCommand(){
  if(finishedReading){
    finishedReading = false;
    switch(command){
      case 'C':
        strncpy(message, "WAIT", 4);
        print2Display(message,display);
        flexy.calibrate();        // Make some calibration to get full output range. Takes about 6 seconds.
        break;
      case 'P':
        if(cmdVal==1){ 
            printData = true;
            }
        if(cmdVal==0){
            printData = false;
            }
        break;
      case 'L':
          if(cmdVal==0){
            filt = false;  
          }else{
            filt = true;
            filter_weight = cmdVal/100.0;
            flexy.initFilter(flexy.getBendPerc());
          }
        break;  
      case 'N':
          rand_noise_multiplier = cmdVal/10.0;
        break;  
      case 'D':
        if(cmdVal<0) cmdVal = 0;
        if(cmdVal>100) cmdVal = 100;
        flexy.setDelaySamples(cmdVal);
        break;  
      case 'F':
        OCR2B = cmdVal;
        break;
      case 'S':
        f = (float)cmdVal;
        fpfd = (int)(f/fd);
        setup_realtime_execution(cmdVal);
        break;
      case 'V':
        Serial.println("<V:OCL_Flexy_v2.0>");
        break;
      default:
        break;
      }
    }
  }

void printDataFun(){
  if(flag1 && printData){    
    Serial.print("<D:");
    if(filt){
      bend = flexy.applyFilter(flexy.getBendPerc(),filter_weight);
    }else{
      bend = flexy.getBendPerc();
    }
    Serial.print(bend+generate_random_noise());
    Serial.print(",");
    Serial.print(flexy.getUserInputLPerc());
    Serial.print(",");
    Serial.print(flexy.getUserInputRPerc());
    Serial.print(",");
    Serial.print(map(analogRead(A1),0,1023,0,10000)/100.0);
    Serial.println(">");
    flag1 = false;
  }
}

float generate_random_noise(){
  float rand_float = random(-1000,1000)/1000.0;
  return rand_float*rand_noise_multiplier;  
}

void setup_realtime_execution(float freq){  
  cli();
  //set TIMER1 CTC interrupts at {freq}Hz (Ts = 1/{freq} [s])
  TCCR1A = B00000000;   // set entire TCCR1A register to 0
  TCCR1B = B00000000;   // set entire TCCR1B register to 0
  TCNT1  = 0;           // set counter1 to 0
  // set compare match register to achieve {freq}Hz frequency
  OCR1A = (int)((16.0e6)/(freq*64.0) - 1.0);   //(must be <65536 for TIMER1)
  // enable CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  //enable global interrupts
  sei();
}


ISR(TIMER1_COMPA_vect){
  flag1=true;  // just set the flag1
  counter2++;
}

void updateDisplay(){
  if(counter2>=ceil(fpfd)){
    flag2 = true;
    counter2 = 0;
    }
  else flag2 = false;
  if(flag2){
    int pot1 = analogRead(A2);
    int pot1_diff = abs(pot1_old-pot1);
    int pot2 = analogRead(A3);
    int pot2_diff = abs(pot2_old-pot2);
    if(pot1_diff>=6){
      knob = flexy.getUserInputLPerc();
      pot1_old = pot1;
      left_right = true;
      printInt2Display(knob,left_right,display);
      hold = true;
    }
    if(pot2_diff>=6){
      knob = flexy.getUserInputRPerc();
      pot2_old = pot2;
      left_right = false;
      printInt2Display(knob,left_right,display);
      hold = true;
    } 
    if(hold&&hold_counter<40){
      hold_counter++;
    }
    else if(hold&&hold_counter>=40){
      hold_counter = 0;
      hold = false;
    }  
    if(!hold){
      printFlex2Display(flexy.getBendPerc(),display);
    }
    flag2 = false;
  }
}

void print2Display(char msg[], Adafruit_AlphaNum4 disp){
  disp.clear();
  for(int i=0,j=0;i<strlen(msg);i++){
    if(msg[j+1]=='.'){
      disp.writeDigitAscii(i,msg[j],true);
      j++;
    }else{
      disp.writeDigitAscii(i,msg[j]);
    }
  j++;
  }
  disp.writeDisplay();
}

void printFlex2Display(int number, Adafruit_AlphaNum4 disp){
  disp.clear();
  disp.writeDigitAscii(0,'F');
  sprintf (display_buffer, "%03d", number);
  int j = 0;
  for(int i=1;i<4;i++){
    disp.writeDigitAscii(i,display_buffer[j]);
    j++;
  }
  disp.writeDisplay();
}

void printInt2Display(int number,boolean lr, Adafruit_AlphaNum4 disp){
  disp.clear();
  int start;
  if(lr){
    start = 1;
    disp.writeDigitRaw(0,0b0010010011000000);
  }else{
    start = 0;
    disp.writeDigitRaw(3,0b0000100111000000);
  }
  sprintf (display_buffer, "%03d", number);
  int j = 0;
  for(int i=start;i<(start+3);i++){
    disp.writeDigitAscii(i,display_buffer[j]);
    j++;
  }
  disp.writeDisplay();
}
