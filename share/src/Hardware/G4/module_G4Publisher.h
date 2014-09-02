#pragma once

#include <Core/module.h>

class G4Publisher : public Module{
public:
	ACCESS(floatA, poses);

	G4Publisher();
	~G4Publisher();

	virtual void open();
	virtual void close();
	virtual void step();

private:
	struct sG4Publisher*s;
};
