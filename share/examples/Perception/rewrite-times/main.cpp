extern "C"{
#include <libavcodec/avcodec.h>
#include <libavutil/mathematics.h>
#include <libavutil/samplefmt.h>
#include <libavformat/avformat.h>
}
#include <Perception/avutil.h>
#include <Core/util.h>
#include <fstream>

using namespace std;
using namespace MT;
using namespace MLR;

namespace {
	template<typename T>
	void check_notnull(const T* v, const std::string& msg) {
		if(v == NULL) {
			HALT(msg.c_str());
		}
	}

	void check_ge_0(int result, int line) {
		if(result < 0) {
			static char error_buffer[1024];
			av_strerror(result, error_buffer, sizeof(error_buffer));
			HALT(STRING(error_buffer << " at line " << line));
		}
	}

	ostream& operator<<(ostream& out, const AVRational& ar) {
		out << ar.num << "/" << ar.den;
		return out;
	}
	ostream& operator<<(ostream& out, const AVCodec& c) {
		out << "codec name " << c.name << endl
			<< "long name " << c.long_name << endl
			<< "media type " << c.type << endl
			<< "codec id " << c.id  << endl
			<< "caps " << c.capabilities << endl
			<< "private data size " << c.priv_data_size << endl
			<< "  not printing supported_framerates, pix_fmt, supported_samplerates, sample_fmts, channel_layouts" << endl
			<< "  max_lowres, priv_class, profiles, next" << endl
			;

		return out;
	}
	ostream& operator<<(ostream&out, const AVClass& c) {
		out << "av class " << c.class_name << endl;
		return out;
	}
	ostream& operator<<(ostream& out, const AVCodecContext& cc) {
		clog << "codec context " << endl
			<< *cc.av_class << endl
			<< "codec type " << cc.codec_type << endl
			<< "codec ";
		if(cc.codec != NULL) {
			out << *(cc.codec);
		} else {
			out <<  "NULL";
		}
		out << endl
			<< "codec tag " << hex << cc.codec_tag << endl
			<< "stream codec tag " << hex << cc.stream_codec_tag << dec << endl
			<< "bitrate " << cc.bit_rate << endl
			<< "bit_rate_tolerance " << cc.bit_rate_tolerance << endl
			<< "global_quality " << cc.global_quality << endl
			<< "compression level " << cc.compression_level << endl
			<< "time base " << cc.time_base << endl
			<< "ticks per frame " << cc.ticks_per_frame << endl
			<< "width " << cc.width << endl
			<< "height " << cc.height << endl
			<< "coded width " << cc.coded_width << endl
			<< "coded height " << cc.coded_height << endl
			<< "gop size " << cc.gop_size << endl
			<< "pixel format " << cc.pix_fmt << endl
			<< "me method " << cc.me_method << endl
			<< "max B frames " << cc.max_b_frames << endl
			<< "B quant factor " << cc.b_quant_factor << endl
			<< "B quant offset " << cc.b_quant_offset << endl
			<< "has B frames " << (bool)cc.has_b_frames << endl
			<< "mpeq quantizer " << cc.mpeg_quant << endl
			<< "I quant factor " << cc.i_quant_factor << endl
			<< "luminance masking " << cc.lumi_masking << endl
			<< "slice count " << cc.slice_count << endl
			<< "sample aspect ratio " << cc.sample_aspect_ratio << endl
			<< "me comparison " << cc.me_cmp << endl
			<< "me_sub_cmp " << cc.me_sub_cmp << endl
			<< "mb cmp " << cc.mb_cmp << endl
			<< "il DCT cmp " << cc.ildct_cmp << endl;
		return out;

 	}

	ostream& operator<<(ostream& out, const AVFrac& pts) {
		out << pts.num << "/" << pts.den;
		return out;
	}
	ostream& operator<<(ostream& out, AVDictionary& dict) {
		out << "dictionary " << endl; // av_dict_count is not always available with " << av_dict_count(&dict) << " entries" << endl;
		AVDictionaryEntry *t = NULL;
		while ((t = av_dict_get(&dict, "", t, AV_DICT_IGNORE_SUFFIX)) != NULL) {
		     out << t->key << " -> " << t->value << endl;
	 }
		return out;
	}

	ostream& operator<<(ostream&out, const AVStream& ins) {
		out << "av stream" << endl
			<< *ins.codec
			<< "Frame rate " << ins.r_frame_rate << endl
			<< "private data " << ins.priv_data << endl
			<< "Meta data " << *ins.metadata
			<< "pts fraction " << ins.pts << endl
			<< "time base " << ins.time_base << endl
			<< "start time " << ins.start_time << endl
			<< "duration   " << ins.duration << endl
			<< "nb frames " << ins.nb_frames << endl
			<< "reference dts " << ins.reference_dts << endl
			<< "first dts " << ins.first_dts << endl
			<< "current dts " << ins.cur_dts << endl
			<< "last IP pts " << ins.last_IP_pts << endl;

		return out;
	}

}
#define CHECK_GE_0(result) check_ge_0(result, __LINE__)

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
