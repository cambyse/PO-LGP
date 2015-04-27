#include "calibrator_module.h"
#include <Hardware/gamepad/gamepad.h>
#include <Ors/ors.h>
#include <Mocap/mocapdata.h>

// ############################################################################
// Calibrator


TapRecon::TapRecon()
    : calibration_phase_taprecon(true)
    , center(3)
{
  mid.load("g4mapping.kvg");
	cout<<"Record your doubleTab, hold X"<<endl;
}

void TapRecon::step()
{
		if(calibration_phase_taprecon)
		int button = gpstate(0);
		floatA rawrecording;
		

		if (button & BTN_X)
 		{

				rawrecording(ticks) =mid.query(poses.get(), STRINGS("/human/rl/rf")).cols(3, 4);		
				ticks++;

 	  }
		 
	

}




