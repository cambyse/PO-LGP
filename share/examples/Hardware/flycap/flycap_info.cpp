/*
 * flycap_info.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: ingo
 */

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <flycapture/FlyCapture2.h>
#include <exception>

using namespace std;
using namespace FlyCapture2;

namespace {
void CHECK_ERROR(const Error e) {
		if(e != PGRERROR_OK) {
			cerr << e.GetDescription() << endl;
			throw "bad";
		}
	}
}

int main(int argc, char* argv[]) {
	int index = 0;
	if(argc > 1) {
		index = strtol(argv[1], NULL, 10);
	}
	cout << "Checking camera with index " << index << endl;

	BusManager bm;
	PGRGuid id;
	bm.GetCameraFromIndex(0, &id);

	GigECamera cam;
	Error e = cam.Connect(&id);


	Format7Info f7info;
	f7info.mode = MODE_0;
	bool supported;

	for(Mode m : { MODE_0, MODE_7}) {
		f7info.mode = m;
		if(cam.GetFormat7Info(&f7info, &supported) == PGRERROR_OK) {
			cout << setw(20) << "Mode " << f7info.mode << " supported: " << supported << endl
					<< setw(20) << "max width: " << f7info.maxWidth << endl
					<< setw(20) << "max height: " << f7info.maxHeight << endl
					<< setw(20) << "Max packet size: " << f7info.maxPacketSize << endl
					<< setw(20) << "Max percent size: " << f7info.percentage << endl
					<< setw(20) << "Pixel formats:" << f7info.pixelFormatBitField << endl;

		}
	}
	GigEConfig gc;
	GigEStreamChannel channel;
	if(cam.GetGigEStreamChannelInfo(0, &channel) == PGRERROR_OK) {
		cout << "Stream channel info" << endl
				<< "packet size: " << channel.packetSize << endl
				<< "inter packet delay: " << channel.interPacketDelay << endl;
	}
	channel.packetSize = 9000;
	channel.interPacketDelay = 100;
	cam.SetGigEStreamChannelInfo(0, channel);


	return 0;
}
