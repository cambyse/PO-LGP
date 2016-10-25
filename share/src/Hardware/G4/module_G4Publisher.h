#pragma once

#include <Core/thread.h>

class G4Publisher : public Module{
public:
	ACCESS(floatA, g4_poses);

	G4Publisher();

	virtual void open();
	virtual void close();
	virtual void step();

private:
	struct sG4Publisher*s;
};
