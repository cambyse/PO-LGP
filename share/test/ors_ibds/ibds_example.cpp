/*
 * IBDS - Impulse-Based Dynamic Simulation Library
 * Copyright (c) 2003-2007 Jan Bender http://www.impulse-based.de
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Jan Bender - Jan.Bender@impulse-based.de
 */

#include "../Common/Config.h"
#define _USE_MATH_DEFINES 
#include "math.h"
#include "MiniGL.h"
#include "GL/glut.h"
#include "../DynamicSimulation/TimeManager.h"
#include "../DynamicSimulation/Simulation.h"
#include "../DynamicSimulation/RigidBody.h"
#include "../DynamicSimulation/BallJoint.h"
#include "../Math/SimMath.h"
#include "../DynamicSimulation/MeshGeometry.h"
#include "../DynamicSimulation/SphereGeometry.h"
#include "../CollisionDetection/CollisionGeometryBullet.h"
#include "../CollisionDetection/CollisionGeometrySolid.h"
#include <time.h>

using namespace IBDS;

void timeStep ();
void buildModel ();
void render ();
void exit ();
Geometry* createCubeGeometry (Real sx, Real sy, Real sz, float r, float g, float b, float a);
void addCollisionCube (int index, Real sx, Real sy, Real sz);
Geometry* createSphereGeometry (Real radius, float r, float g, float b, float a);
void addCollisionSphere (int index, Real radius);

#define MAX_OBJECTS 50
int controllerIndex = Simulation::TIMESTEP_ITERATIVE;
RigidBody *rb [MAX_OBJECTS];


// main 
int main( int argc, char **argv )
{
	// OpenGL
	MiniGL::init (argc, argv, 800, 600, 0, 0, "MiniGL");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setExitFunc (exit);

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 1.0f, 100.0f, Vector3D (3.0, 10.0, 25.0), Vector3D (0.0, 0.0, 0.0));

	glutMainLoop ();

	return 0;
}

void exit ()
{
	delete Simulation::getCurrent ();
}

void timeStep ()
{
	for (int i=0; i < 8; i++)
		Simulation::getCurrent ()->timeStep ();
}

void buildModel ()
{
	Simulation *sim = Simulation::getCurrent ();
	sim->setCollisionDetectionMethod (CollisionDetection::CD_BULLET);
	sim->setMaxDistance (controllerIndex, 0.000001);
	sim->setMaxVelDiff (controllerIndex, 0.001);
	sim->setMaxCorrectionSteps (controllerIndex, 100);
	sim->setCRMethod (Simulation::CR_JBNEWTON);
	sim->setTimeOfImpactComputation (Simulation::TOI_BINARYSEARCH);
	(*sim->getCollisionResponseController ())[sim->getCRMethod ()]->setMaxCorrectionSteps (100); 
	(*sim->getCollisionResponseController ())[sim->getCRMethod ()]->setMaxVelDiff (0.001); 
	(*sim->getCollisionResponseController ())[sim->getCRMethod ()]->setShockPropagation (true);
	sim->setMaxDistanceContacts (controllerIndex, 0.0001);
	sim->setMaxCorrectionStepsContacts (controllerIndex, 100);
	(*sim->getTimeStepController ())[controllerIndex]->setVCorrectionContacts (false);
	sim->getCollisionDetection ()->setTolerance (0.04);
	sim->setVCorrection (controllerIndex, true);
	sim->setMaxCorrImpulse (controllerIndex, 10);
	sim->setMaxJointImpulse (controllerIndex, -1);
	sim->setIntegrationMethod (controllerIndex, TimeStepController::RUNGE_KUTTA);
	TimeManager::getCurrent ()->setTimeStepSize (0.005);

	srand( (unsigned) time (NULL));

	// Boden
	rb[0] = new RigidBody ();
	rb[0]->setMass (1);
	rb[0]->setInertiaTensor (Vector3D (1, 1, 1));
	rb[0]->setCenterOfMass (Vector3D (10,0.0,0.5));
	rb[0]->setCenterOfMassV (Vector3D (0,0.0,0.0));
	rb[0]->setDynamic (false);
	rb[0]->setRestitution (0.6);
	rb[0]->setDynamicFriction (0.1);
	rb[0]->setStaticFriction (0.1);
	rb[0]->addGeometry (createCubeGeometry (150, 1, 150, 0.0f,0.0f,0.8f,1.0f));

	Real x=-3;
	Real z=-3;
	Real y=5;
	int i;
	for (i=1; i < MAX_OBJECTS; i++)
	{
		rb[i] = new RigidBody ();

		rb[i]->setMass (6);
		rb[i]->setRotationMatrix (SimMath::rotationsmatrix (Vector3D (1,1,0), M_PI/32));
		rb[i]->setCenterOfMass (Vector3D (x,y,z));
		Real vx = 3*(Real) rand ()/ RAND_MAX;
		Real vz = 3*(Real) rand ()/ RAND_MAX;
		rb[i]->setAngularVelocity (Vector3D (vx,0.0,vz));
		if (x > 2)
		{
			x = -3;
			if (z > 2)
			{
				z = -3;
				y += 2;
			}
			else
				z += 3;
		}
		else
			x += 3;
		rb[i]->setDynamic (true);
		rb[i]->setRestitution (0.8);
		rb[i]->setDynamicFriction (0.4);
		rb[i]->setStaticFriction (0.4);
		if (i % 2 == 1)
		{
			rb[i]->addGeometry (createSphereGeometry (.5, 0.0f,0.3f,0.6f,1.0f));
			rb[i]->setInertiaTensor (SimMath::computeSphereIntertiaTensor (6, .5));
		}
		else
		{
			rb[i]->addGeometry (createCubeGeometry (1, 1, 1, 0.0f,0.6f,0.0f,1.0f));
			rb[i]->setInertiaTensor (SimMath::computeBoxIntertiaTensor (6, 1, 1, 1));
		}
	}

	for (i=0; i < MAX_OBJECTS; i++)
	{
		string name = string ("RigidBody_") + (i+1);
		rb[i]->setName (name);
		sim->addBody (controllerIndex, rb[i]);
	}

    sim->buildModel();

	addCollisionCube (0, 150, 1, 150);
	for (int i=1; i < MAX_OBJECTS; i++)
	{
		if (i % 2 == 1)
			addCollisionSphere (i, .5);
		else
			addCollisionCube (i, 1, 1, 1);
	}
}



void render ()
{
	Matrix3x3 matrix = Matrix3x3 ();
	MiniGL::coordinateSystem();
	
	float col [4] = {0,0,1,1};
	float col2 [4] = {1,0,0,1};

	Simulation *sim = Simulation::getCurrent ();

	// Bodies
	for (int i=0; i < sim->numberOfBodies (controllerIndex); i++)
	{
		RigidBody *rb = (RigidBody*) sim->getBody (controllerIndex, i);
		Vector3D *translation = rb->getCenterOfMass ();
		Matrix3x3 *rotation = rb->getRotationMatrix ();

		MiniGL::drawGeometry (translation, rotation, rb->getGeometry (0));

		// Collision Object
		//MiniGL::drawCollisionGeometry (translation, rotation, rb->getCollisionObject (0)->getGeometry (), col2);
	}
	float col3 [4] = {0.0f,.5f,.8f,1};
	string str = "time: ";
	str = str + TimeManager::getCurrent ()->getTime ();
	MiniGL::drawBitmapText (-.99f, .9f, str.c_str (), (int) str.length (), col3);
}


/** Erzeugt eine Würfel-Geometrie mit der übergebenen Skalierung und gibt sie zurück.
  */
Geometry* createCubeGeometry (Real sx, Real sy, Real sz, float r, float g, float b, float a)
{
	// Cube data
	const Real vertices[] =
	{
		-0.5,  0.5,  0.5,
		-0.5,  -0.5,  0.5,
		-0.5,  -0.5,  -0.5,
		-0.5,  0.5,  -0.5,
		0.5,  -0.5,  0.5,
		0.5,  -0.5,  -0.5,
		0.5,  0.5,  0.5,
		0.5,  0.5,  -0.5
	};

	const int faces[] =
	{
		0, 2, 1,
		0, 3, 2,
		0, 1, 4,
		6, 0, 4,
		4, 5, 6,
		5, 7, 6,
		2, 3, 5,
		5, 3, 7, 
		0, 6, 3, 
		3, 6, 7, 
		1, 2, 4, 
		4, 2, 5
	};
	MeshGeometry *geo = new MeshGeometry ();
	Vector3D scale = Vector3D (sx, sy, sz);
	geo->setTriangles (8, vertices, 12, faces, NULL, NULL, &scale);
	geo->setColor (r, g, b, a);
	return geo;
}

/** Erzeugt eine Würfel-Geometrie mit der übergebenen Skalierung und gibt sie zurück.
  */
Geometry* createSphereGeometry (Real radius, float r, float g, float b, float a)
{
	SphereGeometry *geo = new SphereGeometry ();
	geo->setRadius (radius);
	geo->setColor (r, g, b, a);
	return geo;
}

/** Erzeugt eine Würfel-Geometrie mit der übergebenen Skalierung und verwendet sie als Kollisionsobjekt.
  */
void addCollisionCube (int index, Real sx, Real sy, Real sz)
{
	// Cube data
	const Real vertices[] =
	{
		-0.5,  0.5,  0.5,
		-0.5,  -0.5,  0.5,
		-0.5,  -0.5,  -0.5,
		-0.5,  0.5,  -0.5,
		0.5,  -0.5,  0.5,
		0.5,  -0.5,  -0.5,
		0.5,  0.5,  0.5,
		0.5,  0.5,  -0.5
	};

	const int faces[] =
	{
		0, 2, 1,
		0, 3, 2,
		0, 1, 4,
		6, 0, 4,
		4, 5, 6,
		5, 7, 6,
		2, 3, 5,
		5, 3, 7, 
		0, 6, 3, 
		3, 6, 7, 
		1, 2, 4, 
		4, 2, 5
	};
	Simulation *sim = Simulation::getCurrent ();

	Vector3D scale = Vector3D (sx, sy, sz);
	sim->addCollisionObject (sim->getBody (controllerIndex, index), 8, vertices, 12, faces, &scale);
}

/** Erzeugt eine Kugel-Geometrie mit der übergebenen Skalierung und verwendet sie als Kollisionsobjekt.
  */
void addCollisionSphere (int index, Real radius)
{
	Simulation *sim = Simulation::getCurrent ();

	sim->addCollisionSphere (sim->getBody (controllerIndex, index), Vector3D(), radius);
}
