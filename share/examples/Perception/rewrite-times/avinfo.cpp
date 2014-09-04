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

	register_libav();

	if(argc < 2) {
		cerr << "Syntax: " << argv[0] << " <input-video>" << endl;
		return -1;
	}

	// open input and scan it
	CHECK_GE_0(avformat_open_input(&in_context, argv[1], NULL, NULL));
	CHECK_GE_0(avformat_find_stream_info(in_context, NULL));
	for(unsigned int i = 0; i < in_context->nb_streams; ++i) {
		cout << *(in_context->streams[i]) << endl;
	}

	if(in_context->metadata) {
		cout << "Container metadata " << *(in_context->metadata) << endl;
	}

	avformat_close_input(&in_context);
}
