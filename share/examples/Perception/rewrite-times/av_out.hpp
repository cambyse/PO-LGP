#pragma once

#include <cstdlib>
#include <iostream>
#include <string>

extern "C" {
struct AVOutputFormat;
struct AVClass;
struct AVCodec;
struct AVCodecContext;
struct AVDictionary;
struct AVFrac;
struct AVRational;
struct AVStream;
}

template<typename T>
void check_notnull(const T* v, const std::string& msg) {
	if(v == NULL) {
		std::cerr << msg << std::endl;
		abort();
	}
}

void check_ge_0(int result, int line);

std::ostream& operator<<(std::ostream& out, const AVRational& ar);
std::ostream& operator<<(std::ostream& out, const AVCodec& c);
std::ostream& operator<<(std::ostream&out, const AVClass& c);
std::ostream& operator<<(std::ostream& out, const AVCodecContext& cc);
std::ostream& operator<<(std::ostream& out, const AVFrac& pts);
std::ostream& operator<<(std::ostream& out, AVDictionary& dict);
std::ostream& operator<<(std::ostream&out, const AVStream& ins);

#define CHECK_GE_0(result) check_ge_0(result, __LINE__)

AVOutputFormat* mt_guess_format(const char* filename, const char* DEF_FORMAT);
void register_libav();
