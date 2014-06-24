#include "av_out.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/dict.h>
#include <libavutil/error.h>
#include <libavutil/log.h>
#include <libavutil/rational.h>
}
#include <ccomplex>

using namespace std;

void register_libav()
{
   av_register_all();
   avcodec_register_all();
}
AVOutputFormat* mt_guess_format(const char* filename, const char* DEF_FORMAT) {
 AVOutputFormat* fmt = av_guess_format(NULL, filename, NULL);
    if(!fmt) {
        std::cerr << "Could not determine container format from filename '" << filename << "', attempting " << DEF_FORMAT;
        fmt = av_guess_format(DEF_FORMAT, NULL, NULL);
        if(!fmt) {
            std::cerr << "Could not open container format for " << DEF_FORMAT << endl;
            return NULL;
        }
    }
    return fmt;
}


void check_ge_0(int result, int line) {
	if(result < 0) {
		static char error_buffer[1024];
		av_strerror(result, error_buffer, sizeof(error_buffer));
		cerr << error_buffer << " at line " << line << endl;
		abort();
	}
}

ostream& operator<<(ostream& out, const AVRational& ar) {
	out << (double)ar.num /(double) ar.den;
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
