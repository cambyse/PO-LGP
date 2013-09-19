#include <iostream>
#include "videowriter.h"

bool VideoWriter_x264::initialized = false;
QMutex VideoWriter_x264::initMutex;

VideoWriter_x264::VideoWriter_x264(const char *filename,
                                    int width,
                                    int height,
                                    int frame_rate,
                                    int crf,
                                    const char *preset) {
  init();

  /* auto detect the output format from the name */
  AVOutputFormat *fmt = av_guess_format(NULL, filename, NULL);
  if(!fmt) {
    fprintf(stderr, "Could not find suitable output format\n");
    exit(EXIT_FAILURE);
  }
  /* allocate the output media context */
  out = avformat_alloc_context();
  if(!out) {
    fprintf(stderr, "Memory error\n");
    exit(EXIT_FAILURE);
  }
  out->oformat = fmt;
  snprintf(out->filename, sizeof(out->filename), "%s", filename);

  if(avio_open2(&out->pb, out->filename, AVIO_FLAG_WRITE, NULL, NULL) < 0) {
    fprintf(stderr, "Could not open '%s'\n", out->filename);
    exit(EXIT_FAILURE);
  }

  /////////// OPEN ENCODER

  AVCodec *codec = avcodec_find_encoder_by_name("libx264");
  if(!codec) {
    fprintf(stderr, "Encoder libx264 not found\n");
    exit(EXIT_FAILURE);
  }

  AVStream *st = avformat_new_stream(out, codec);
  if(!st) {
    fprintf(stderr, "Could not alloc stream\n");
    exit(EXIT_FAILURE);
  }

  st->sample_aspect_ratio.den = 1;
  st->sample_aspect_ratio.num = 1;

  enc = st->codec;

  enc->width = width;
  enc->height = height;

  enc->time_base.den = frame_rate;
  printf("frame_rate = %d\n", frame_rate);
  enc->time_base.num = 1;
  enc->pix_fmt = PIX_FMT_YUV420P;

  enc->sample_aspect_ratio.den = 1;
  enc->sample_aspect_ratio.num = 1;

  enc->thread_count = 0; /* use several threads for encoding */

  AVDictionary *opts = NULL;
  char crf_str[4];
  sprintf(crf_str,"%d",crf);
  av_dict_set(&opts, "crf", crf_str, 0);
//  av_dict_set(&opts, "preset", "superfast", 0);
  av_dict_set(&opts, "preset", preset, 0);

  /* open the codec */
  if(avcodec_open2(enc, codec, &opts) < 0) {
    fprintf(stderr, "could not open codec\n");
    exit(EXIT_FAILURE);
  }

  /* allocate output buffer */
  video_outbuf_size = enc->width * enc->height * 4; /* upper bound */
  video_outbuf = (uint8_t *)av_malloc(video_outbuf_size);
  if(!video_outbuf) {
    fprintf(stderr, "Alloc outbuf fail\n");
    exit(EXIT_FAILURE);
  }
  av_dump_format(out, 0, out->filename, 1);

  int r;
  r = avformat_write_header(out, NULL);
  if(r) {
    fprintf(stderr, "write out file fail\n");
    exit(r);
  }

  AVRational ratio = av_div_q(out->streams[0]->time_base, enc->time_base);
  pts_step = ratio.den / ratio.num;

  frames_in = 0;
  frames_out = 0;

  //sws_ctx = sws_getContext(enc->width, enc->height, PIX_FMT_UYVY422, enc->width, enc->height, enc->pix_fmt, SWS_BILINEAR, NULL, NULL, NULL);
  /*
  sws_ctx = sws_alloc_context();
  sws_ctx->srcW = enc->width;
  sws_ctx->srcH = enc->height;
  sws_ctx->srcFormat = PIX_FMT_UYVY422;
  sws_ctx->dstFormat = enc->pix_fmt;
  sws_ctx->flags = SWS_BILINEAR;
  sws_ctx->param = NULL;
  sws_init_context(sws_ctx , NULL, NULL);
  */
  sws_ctx = sws_getCachedContext(NULL, enc->width, enc->height, PIX_FMT_UYVY422, enc->width, enc->height, enc->pix_fmt, SWS_BILINEAR, NULL, NULL, NULL);

  pFrame = avcodec_alloc_frame();
  int numBytes = avpicture_get_size(enc->pix_fmt, enc->width, enc->height);
  buffer=(uint8_t *)av_malloc(numBytes*sizeof(uint8_t));
  avpicture_fill((AVPicture *)pFrame, buffer, enc->pix_fmt, enc->width, enc->height);
}


VideoWriter_x264::~VideoWriter_x264() {
  av_free(buffer);
  av_free(pFrame);

  // flush encoder
  AVPacket packet;

  int got_packet, fail;
  for(;;) {
    // flush buffered remainings
    av_init_packet(&packet);
    packet.data = NULL;
    packet.size = 0;
    fail = avcodec_encode_video2(enc, &packet, NULL, &got_packet);

    if(fail)
      fprintf(stderr, "Error while encoding frame\n");

    if(!(got_packet > 0 && packet.size))
      break;

    writeEncodedFrame(&packet);
  }

  av_write_trailer(out);
  av_dump_format(out, 0, out->filename, 1);

  avcodec_close(enc);
  av_free(video_outbuf);
  avio_close(out->pb);
  avformat_free_context(out);
}

void VideoWriter_x264::addFrame(uint8_t *buffer) {
  AVPacket packet;
  av_init_packet(&packet);
  packet.data = NULL;
  packet.size = 0;

  // convert frame to encoder format (PIX_FMT_YUV420P)
  int src_stride = enc->width*2;
  sws_scale(sws_ctx, &buffer, &src_stride, 0, enc->height, pFrame->data,pFrame->linesize);

  pFrame->width = enc->width;
  pFrame->height = enc->height;
  pFrame->pts = frames_in++;

  // encode the image
  int got_packet;
  int fail = avcodec_encode_video2(enc, &packet, pFrame, &got_packet);

  if(fail)
    fprintf(stderr, "Error while encoding frame\n");

  // if zero size, it means the image was buffered
  if(got_packet > 0 && packet.size) 
    writeEncodedFrame(&packet);
}

int VideoWriter_x264::writeEncodedFrame(AVPacket *pPacket) {
  // removed encoded size

  /*
  // write to stream
  AVPacket pkt;
  av_init_packet(&pkt);

  pkt.pts = frames_out++ * pts_step;
  pkt.dts = pkt.pts;
  pkt.duration = pts_step;
  if(enc->coded_frame->key_frame)
    pkt.flags |= AV_PKT_FLAG_KEY;
  pkt.data = video_outbuf;
  pkt.size = encoded_size;

//  printf("stream pts %d - stream.curr_dts %d - packet pts %d\n",out->streams[0]->pts.val,out->streams[0]->cur_dts,pkt.dts);



  // write the compressed frame in the media file
  int r = av_interleaved_write_frame(out, &pkt);
  av_free_packet(&pkt);

  if (r) {
    fprintf(stderr, "Error while writing video frame\n");
    return -1;
  }

  return(r);
  */

  // write to stream
  pPacket->pts = frames_out++ * pts_step;
  pPacket->dts = pPacket->pts;

//  printf("stream pts %d - stream.curr_dts %d - packet pts %d\n",out->streams[0]->pts.val,out->streams[0]->cur_dts,pkt.dts);

  // write the compressed frame in the media file
  int r = av_interleaved_write_frame(out, pPacket);
  av_free_packet(pPacket);

  if(r) {
    fprintf(stderr, "Error while writing video frame\n");
    return -1;
  }
  return 0;
}

void VideoWriter_x264::init() {
  // needs to be protected since the library could be initialized from multiple threads simultaneously
  VideoWriter_x264::initMutex.lock();
  if(!initialized) {
    printf("initializing lock manager\n");
    av_lockmgr_register(&lockManagerQt);
    av_register_all();
    initialized = true;
  }
  VideoWriter_x264::initMutex.unlock();
}

int VideoWriter_x264::lockManagerQt(void **mutex, enum AVLockOp op) {
  if(NULL == mutex)
    return -1;

  switch(op) {
    case AV_LOCK_CREATE:
      {
      *mutex = NULL;
      QMutex *m = new QMutex();
      *mutex = static_cast<void*>(m);
      break;
      }
    case AV_LOCK_OBTAIN:
      {
      QMutex *m = static_cast<QMutex*>(*mutex);
      m->lock();
      break;
      }
    case AV_LOCK_RELEASE:
      {
      QMutex *m = static_cast<QMutex*>(*mutex);
      m->unlock();
      break;
      }
    case AV_LOCK_DESTROY:
      {
      QMutex *m = static_cast<QMutex*>(*mutex);
      delete m;
      break;
      }
    default:
      break;
  }
  return 0;
}

