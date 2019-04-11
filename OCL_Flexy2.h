#ifndef OCL_Flexy2_h
#define OCL_Flexy2_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif

// Definition of OCL_Flexy2 class header 

class OCL_Flexy2 {
	public:
		OCL_Flexy2();

		void  begin();
		void  calibrate();

		void  setFanSpeedPerc(float perc);
		void  setFanSpeedNorm(float norm);
		void  setFanSpeedRPM(float rpm);
		void  setDelaySamples(uint8_t samples);

		float getBendPerc();
		float getBendNorm();
		int   getBendRaw();
		float getUserInputLPerc();
		float getUserInputRPerc();
		float getUserInputLNorm();
		float getUserInputRNorm();
		float getTerminalInputPerc();
		float getTerminalInputNorm();

		void  initFilter(float init);
		float applyFilter(float input, float weight);

		int   readADC();
		int   getADC();
		void  setADCSamples(int samples);

		void EEPROMWritelong(int address, long value);
		long EEPROMReadlong(long address);

	private:
		long     _out_max;
		long     _out_min;
		uint8_t _pwm_control_pin;
		//uint8_t _adc_pin;
		uint8_t _pot_l_pin;
		uint8_t _pot_r_pin;
		uint8_t _terminal_input_pin;
		uint8_t _delay_samples;
		int     _adc_samples;
		int     _reading;
		int     _user_input_l;
		int     _user_input_r;
		int     _terminal_input;
		float   _filt_last;
		float   _filt_act;
		int     _delay_buffer[100];
		float   mapToFloat(int in, float in_min, float in_max, float out_min, float out_max);
};

#endif