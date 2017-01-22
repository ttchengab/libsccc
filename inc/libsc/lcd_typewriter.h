/*
 * main.cpp
 *
 * Author: Peter
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <stdlib.h>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/k60/ov7725.h>
#include <libsc/futaba_s3010.h>
#include <libsc/alternate_motor.h>
#include <libsc/lcd_typewriter.h>



namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 150000;
			return config;
		}

	}
}

using namespace libsc;
using namespace libbase::k60;
using namespace libsc::k60;

int main(void)
{
	System::Init();

	Ov7725::Config C;  //camera init;
	C.id = 0;
	C.w = 80;
	C.h = 60;
	Ov7725 cam(C);

	St7735r::Config s;
	s.is_revert = false;
	s.is_bgr = false; //screen init;
	s.fps = 100;
	St7735r screen(s);
	Timer::TimerInt t=0;

	LcdTypewriter::Config typewriter_config;
	LcdTypewriter typewriter(typewriter_config);

	FutabaS3010::Config servo_config;
	servo_config.id = 0;
	FutabaS3010 servo(servo_config);

	AlternateMotor::Config motor_config;
	motor_config.multiplier=100;
	AlternateMotor motor(motor_config);

	int v[15][80], index; //v for value
	int areaDiff=0, orgAreaDiff;
	double pControl, dControl, Kp, Kd;
	Kp = 0.05;
	Kd = 0.02;

	cam.Start();
	servo.SetDegree(9.0);

	while (true){
		while(t!=System::Time()){
			t = System::Time();

			if(t % 10 == 0){
				screen.SetRegion(Lcd::Rect(0,0,80,60));
				screen.FillBits(St7735r::kBlack,St7735r::kWhite,cam.LockBuffer(),8*cam.GetBufferSize());
/*
				orgAreaDiff = areaDiff;
				areaDiff = 0;
				for(int line=11; line<26; line++){
					index=0;
					for(int i=0; i<10 ; i++){
						for(int j=7; j>=0; j--){
							v[line][index++] = (*(cam.LockBuffer()+line+i) >> j) & 1;
						}
					}

					for(int i=1;i<79;i++){
						if(v[line][i-1]+v[line][i]+v[line][i+1] >= 2){
							if(i<40) areaDiff++;
							else areaDiff--;
						}
					}
				}
				pControl = areaDiff * Kp; //to be determined
				dControl = (areaDiff-orgAreaDiff) * Kd; //to be determined
				servo.SetDegree(9.0 + pControl + dControl);


				motor.SetPower((10-abs(servo.GetDegree()-9))*10); //motor speed depends on servo's turning deg, to be determined(maybe pid)
*/
				cam.UnlockBuffer();
			}
		}
	}

	cam.Stop();

	return 0;
}
