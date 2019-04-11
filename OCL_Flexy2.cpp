#include "OCL_Flexy2.h"
#include <SPI.h>
#include <EEPROM.h>
#define SELPIN 10    // chip-select
#define DATAOUT 11   // MOSI 
#define DATAIN 12    // MISO 
#define SPICLOCK 13  // Clock 
#define EEPROM_ADDR_MIN 0
#define EEPROM_ADDR_MAX 4

OCL_Flexy2::OCL_Flexy2(){

	_out_min = EEPROMReadlong(EEPROM_ADDR_MIN);
	_out_max = EEPROMReadlong(EEPROM_ADDR_MAX);
	if(_out_min==0&&_out_max==0){
		_out_min = 0;
		_out_max = 4095;
	}
	_pwm_control_pin = 3;
	_terminal_input_pin = A1;
	_filt_last = 0.0;
	_filt_act = 0.0;
	_pot_l_pin = A2;
	_pot_r_pin = A3;
	_user_input_l = 0;
	_user_input_r = 0;
	_terminal_input = 0;
	_reading = 0;
	_adc_samples = 50;
	_delay_samples = 0;
	_delay_buffer[100];
	pinMode(SELPIN, OUTPUT); 
  	digitalWrite(SELPIN, HIGH);
 	SPI.setClockDivider( SPI_CLOCK_DIV8 ); // slow the SPI bus down
  	SPI.setBitOrder(MSBFIRST);
  	SPI.setDataMode(SPI_MODE0);    // SPI 0,0 as per MCP330x data sheet 
  	SPI.begin();
}

void OCL_Flexy2::begin(){

	pinMode(_pwm_control_pin,OUTPUT);
	TCCR2A = 0x23 ;
	TCCR2B = 0x09 ; // mode 7, clock prescale by 1
	OCR2A = 255 ;	// 160 clock periods = 10us per cycle
	OCR2B =0 ;
	TCNT2 =0 ;
}

void OCL_Flexy2::calibrate(){

	_out_max = 0;
	_out_min = 4095;
	for(int i=0;i<100;i++){
		_reading = getADC();
		if(_reading<_out_min) _out_min = _reading;
		delay(10);
	}
	for(int i=0;i<3;i++){
		setFanSpeedPerc(50.0);
		delay(250);
		setFanSpeedPerc(75.0);
		delay(250);
		setFanSpeedPerc(100.0);
		for(int j=0;j<100;j++){
			_reading = getADC();
			if(_reading>_out_max) _out_max = _reading;
			delay(10);
		}
	}
	setFanSpeedPerc(0.0);
	EEPROMWritelong(EEPROM_ADDR_MIN, _out_min);
	EEPROMWritelong(EEPROM_ADDR_MAX, _out_max);
	delay(2000);
}

int OCL_Flexy2::readADC(){
  int adcvalue = 0;
  uint8_t b1 = 0, b2 = 0;
  uint8_t sign = 0;
  digitalWrite(SELPIN, LOW);
  b1 = SPI.transfer(0);
  b1 |= B11100000;
  sign = b1 & B00010000;
  int hi = b1 & B00001111;
  b2 = SPI.transfer(0);
  int lo = b2;
  digitalWrite(SELPIN, HIGH);
  int reading = hi * 256 + lo;
  if (sign) {
    reading = (4096 - reading) * -1;
  }
  return (reading);
}

int OCL_Flexy2::getADC(){
    long reading = 0;
    int out_reading;
    for(int i=0;i<_adc_samples;i++){
      reading += readADC();
    }
    int new_reading = (reading/_adc_samples);
	if(_delay_samples>0){    
	    out_reading = _delay_buffer[_delay_samples-1];
	    for(int i=_delay_samples-1;i>0;i--){
	     	_delay_buffer[i] = _delay_buffer[i-1];
	  	}
	  	_delay_buffer[0] = new_reading;
	}else{
		out_reading = new_reading;
	}
    return out_reading;
}

void OCL_Flexy2::setADCSamples(int samples)
{
	_adc_samples = samples;
}

void OCL_Flexy2::setDelaySamples(uint8_t samples)
{
	_delay_samples = samples;
	for(uint8_t i = 0; i<_delay_samples; i++){
		_delay_buffer[i] = _reading; 
	}
}

void OCL_Flexy2::setFanSpeedPerc(float perc)
{
	OCR2B = (int)(2.55*perc);
}

void OCL_Flexy2::setFanSpeedNorm(float norm)
{
	OCR2B = (int)(255.0*norm);
}

void OCL_Flexy2::setFanSpeedRPM(float rpm)
{
	OCR2B = (int)(0.02326*rpm);
}

float OCL_Flexy2::getBendPerc()
{
	_reading = getADC();
	return constrain(mapToFloat(_reading,(float)_out_min,(float)_out_max,0.0,100.0),0.0,120.0);
}

float OCL_Flexy2::getBendNorm()
{
	_reading = getADC();
	return constrain(mapToFloat(_reading,(float)_out_min,(float)_out_max,0.0,1.0),0.0,1.2);
}

int OCL_Flexy2::getBendRaw(){
	return getADC();
}

float OCL_Flexy2::getTerminalInputPerc()
{
	_terminal_input = analogRead(_terminal_input_pin);
	return mapToFloat(_terminal_input,0.0,1023.0,0.0,100.0);
}

float OCL_Flexy2::getTerminalInputNorm()
{
	_terminal_input = analogRead(_terminal_input_pin);
	return mapToFloat(_terminal_input,0.0,1023.0,0.0,1.0);
}

float OCL_Flexy2::getUserInputLPerc()
{
	_user_input_l = analogRead(_pot_l_pin);
	return mapToFloat(_user_input_l,0.0,1023.0,0.0,100.0);
}

float OCL_Flexy2::getUserInputRPerc()
{
	_user_input_r = analogRead(_pot_r_pin);
	return mapToFloat(_user_input_r,0.0,1023.0,0.0,100.0);
}

float OCL_Flexy2::getUserInputLNorm()
{
	_user_input_l = analogRead(_pot_l_pin);
	return mapToFloat(_user_input_l,0.0,1023.0,0.0,1.0);
}

float OCL_Flexy2::getUserInputRNorm()
{
	_user_input_r = analogRead(_pot_r_pin);
	return mapToFloat(_user_input_r,0.0,1023.0,0.0,1.0);
}

float OCL_Flexy2::mapToFloat(int in, float in_min, float in_max, float out_min, float out_max)
{
  return ((float)in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void OCL_Flexy2::initFilter(float init)
{
	_filt_last = init;
}

float OCL_Flexy2::applyFilter(float input, float weight)
{
	_filt_act = (1.0-weight)*input+weight*_filt_last;
	_filt_last = _filt_act;
	return _filt_act;
}

void OCL_Flexy2::EEPROMWritelong(int address, long value)
{
	byte four = (value & 0xFF);
	byte three = ((value >> 8) & 0xFF);
	byte two = ((value >> 16) & 0xFF);
	byte one = ((value >> 24) & 0xFF);
	EEPROM.write(address, four);
	EEPROM.write(address + 1, three);
	EEPROM.write(address + 2, two);
	EEPROM.write(address + 3, one);
}

long OCL_Flexy2::EEPROMReadlong(long address)
{
	long four = EEPROM.read(address);
	long three = EEPROM.read(address + 1);
	long two = EEPROM.read(address + 2);
	long one = EEPROM.read(address + 3);
	return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}