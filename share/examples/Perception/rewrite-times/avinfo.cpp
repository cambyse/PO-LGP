#include <climits>
#include <cstdio>
#include <cstring>

extern "C"{
#include <libavformat/avio.h>
#include <libavutil/dict.h>
#include <libavutil/rational.h>
#include <libavcodec/avcodec.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
#include <libavformat/avformat.h>
}

#include <iostream>
#include <fstream>
#include "av_out.hpp"

using namespace std;

int main(int argc, char* argv[])
{
	AVFormatContext *in_context = NULL;
	AVStream *ins = NULL;

	register_libav();

	if(argc < 2) {
		cerr << "Syntax: " << argv[0] << " <input-video>" << endl;
		return -1;
	}

	// open input and scan it
	CHECK_GE_0(avformat_open_input(&in_context, argv[1], NULL, NULL));
	CHECK_GE_0(avformat_find_stream_info(in_context, NULL));
	if(in_context->nb_streams != 1) {
		cerr << "Expected one input video stream, but got " << in_context->nb_streams << ", aborting" << endl;
		return -1;
	}
	ins = in_context->streams[0];

	clog << *ins << endl;

	avformat_close_input(&in_context);
}
