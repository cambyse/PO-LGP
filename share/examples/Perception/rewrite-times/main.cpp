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
	AVFormatContext *in_context = NULL, *out_context = NULL;
	AVCodecContext *out_codec;
	AVStream *ins = NULL, *outs = NULL;

	AVPacket packet;
	int frame_no;
	double timestamp;

	register_libav();

	if(argc < 4) {
		cerr << "Syntax: " << argv[0] << " <input-video> <input-times> <output-video>" << endl;
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

	// open timestamp file
	ifstream times(argv[2]);
	times >> frame_no >> timestamp;
	double start_time = timestamp;

	// open output file and set it up
	out_context = avformat_alloc_context();
	check_notnull(out_context, "Could not allocate output container");
	CHECK_GE_0(avio_open(&out_context->pb, argv[3], AVIO_FLAG_WRITE));
 	out_context->oformat = mt_guess_format(argv[3], in_context->iformat->extensions);
	check_notnull(out_context->oformat, "Could not guess format");

	// open codec (needed for headers, even if not for actual encoding)
	//out_codec = avcodec_find_encoder(ins->codec->codec_id);
	//check_notnull(out_codec, "Could not allocate output codec context");
	out_codec = ins->codec;

	AVCodecContext save;
	outs = avformat_new_stream(out_context, NULL);
	check_notnull(outs, "Could not allocate video stream in container");
	// we save codec, because its managed by the stream and we need to restore it, to prevent double-free
	save = *(outs->codec);

	// now selectively copy over information
	outs->id = ins->id;
	*(outs->codec) = *(ins->codec);
	outs->codec->codec_tag = 0; // this must be unset because the header writer wants to choose it
	outs->codec->stream_codec_tag = 0; // likewise

	outs->r_frame_rate = ins->r_frame_rate;
	outs->priv_data = ins->priv_data;
	outs->pts = ins->pts;
	outs->sample_aspect_ratio = ins->sample_aspect_ratio;

	clog << *outs << endl;

	// replace some values
	outs->codec->time_base = av_d2q(1./1e3, INT_MAX);
    if(!strcmp(out_context->oformat->name, "mp4"))
            outs->codec->flags |= CODEC_FLAG_GLOBAL_HEADER;

    AVDictionary *opts = NULL;
    char opt_str[4];
    sprintf(opt_str,"%d", 0);
    av_dict_set(&opts, "qp", opt_str, 0);
    av_dict_set(&opts, "preset", "ultrafast", 0);

    /* open it */
    //CHECK_GE_0(avcodec_open2(out_video_stream->codec, NULL, &opts));

    CHECK_GE_0(avformat_write_header(out_context, &opts));
    clog << "time base " << outs->time_base << " (hint was " << av_d2q(1./1e3, INT_MAX) << ")" << endl;

	// copy over packets, while changing pts
	int result = 0, count = 0;
	while((result = av_read_frame(in_context, &packet)) == 0) {
		int new_pts = (int)((timestamp - start_time)*1000); // start from 0!;
		//cerr << "pts: " << packet.pts << "-> " << new_pts << ", dts: " << packet.dts << endl;
		packet.pts = packet.dts = new_pts;
		CHECK_GE_0(av_write_frame(out_context, &packet));

		times >> frame_no >> timestamp;
		if(++count != frame_no) {
			clog << "Warning, times file at sequence number " << frame_no << " but only read " << count << " frames so far." << endl;
		}
	}


	// done
	av_write_trailer(out_context);
	avio_close(out_context->pb);
	*(outs->codec) = save;
	//avformat_close_input(&in_context);
}
