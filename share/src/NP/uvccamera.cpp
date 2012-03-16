#include <iostream>
#include <cassert>
#include <cerrno>
#include <string>

extern "C" 
{
   #include <linux/videodev2.h>
   #include <sys/types.h>
   #include <sys/time.h>
   #include <sys/stat.h>
   #include <sys/mman.h>
   #include <sys/ioctl.h>
   #include <fcntl.h>
}

#include "uvccamera.h"
//#include "common.h"

/******************************************************************************
 * common stuff
 ******************************************************************************/
#define ERROR(msg) { std::cerr << "ERROR: " << __FILE__ << ':' \
             << __LINE__ << " : " << msg << std::endl; \
             throw STRING(msg);}
inline int min(int a, int b) { return (a < b ? a : b); }
//inline int clip(int v) { return (v>255) ? 255 : ((v<0) ? 0 : v); }
inline byte clip(int v) { return (v&0xf00)?(byte)0xff:(byte)v; } //cheaper version


/******************************************************************************
 * Default camera settings                                                    *
 ******************************************************************************/
#define DEFAULT_NUM_BUFFERS     2
#define DEFAULT_WIDTH         320 //640 160
#define DEFAULT_HEIGHT        240 //480 120

/******************************************************************************
 * YUV to RGB conversion coefficients                                         *
 *                                                                            *
 * taken from <http://www.fourcc.org/fccyvrgb.php> and                        *
 *            <http://www.fourcc.org/yuv2ppm.c>                               *
 *                                                                            *
 * Hint: if you're bored, play w/ these and enjoy the world of shifted colors *
 *       (but back-up the ones below beforehand).                             *
 ******************************************************************************/
#define COEF_RV               1.370705
#define COEF_GV               0.698001
#define COEF_GU               0.337633
#define COEF_BU               1.732446

namespace camera {
struct UVCCameraBuffer
{
   void* start;
   uint  length;
};

struct UVCCameraWorkspace
{
   MT::Array<UVCCameraBuffer> buffers_;               // pointers to frame data (mmap'd)
   uint                 frame_size_;

   bool                 is_initialized_;
   int                  device_file_desc_;            // file descriptor of camera
   uint                 width_;                       // frame width
   uint                 height_;                      // frame height
   
   struct v4l2_capability     capability;             // device capabilities
   struct v4l2_streamparm     stream_param;           // streaming parameters
   struct v4l2_cropcap        cropcap;                // cropping & scaling
   struct v4l2_crop           crop;                   // selected crop in cropcap
   struct v4l2_format         fmt;                    // request video format
                                                      // (cf. V4L2 API specs 0.24, 1.11, p. 23)
   struct v4l2_requestbuffers req;                    // data transfer negotiations

   int *conv_rv, *conv_gv, *conv_gu, *conv_bu;
   UVCCameraWorkspace();
   ~UVCCameraWorkspace();
};

// -----------------------------------------------------------------------------
//                                                            utility functions
// -----------------------------------------------------------------------------
void yuyv2rgb(byteA& frame, const UVCCameraBuffer& src, const UVCCameraWorkspace& ws, bool swap=false);
inline int stat_device(const UVCCameraWorkspace* const ws);
} // namespace camera

camera::UVCCameraWorkspace::UVCCameraWorkspace()
{
   // Look-up table w/ coefficients to convert YUV buffer to RGB
   conv_rv = new int[256];
   conv_gv = new int[256];
   conv_gu = new int[256];
   conv_bu = new int[256];

   for(int i = 0; i < 256; i++)
   {
      conv_rv[i] = (i-128) * COEF_RV;
      conv_bu[i] = (i-128) * COEF_BU;
      conv_gu[i] = (i-128) * COEF_GU;
      conv_gv[i] = (i-128) * COEF_GV;
   }
};

camera::UVCCameraWorkspace::~UVCCameraWorkspace()
{
   delete [] conv_rv;
   delete [] conv_gv;
   delete [] conv_gu;
   delete [] conv_bu;
};


// -----------------------------------------------------------------------------
//                                                                    UVCCamera
// -----------------------------------------------------------------------------
camera::UVCCamera::UVCCamera():
  Process("UVCCameraProcess"),workspace_(NULL){
  device_name = "/dev/video0";

   workspace_ = new UVCCameraWorkspace();
}

camera::UVCCamera::~UVCCamera()
{
   delete workspace_;
}

void camera::yuyv2rgb(byteA& frame, const UVCCameraBuffer& src, const UVCCameraWorkspace& ws, bool swap)
{
  int len = min(frame.N, src.length);
  int half_img_size = len / 4;                       // 4 b/c of (Y0 U Y1 V)
  unsigned char* rgb_p;                              // pointer to output buf
  unsigned char* buf = (unsigned char*) src.start;   // pointer to buffer data
  int y0 = 0, y1 = 0, u = 0, v = 0;                  // YUYV pixel values
  if(swap){                                          // Swap x-/y-coordinates
    for(int i=frame.d0-1; i>=0; i--){
      rgb_p = &(frame(i,0,0));
      for(uint j=0; j<frame.d1/2; j++){
	y0 = buf[0];
	u  = buf[1];
	y1 = buf[2];
	v  = buf[3];
	buf +=4;

	/* Replaced block (1) with block (2), because the clipping costs a lot of CPU */
	// block (1)
#if 1
	*rgb_p++ = clip(y0              + ws.conv_rv[v]);
	*rgb_p++ = clip(y0 - ws.conv_gu[u] - ws.conv_gv[v]);
	*rgb_p++ = clip(y0 + ws.conv_bu[u]);

	*rgb_p++ = clip(y1              + ws.conv_rv[v]);
	*rgb_p++ = clip(y1 - ws.conv_gu[u] - ws.conv_gv[v]);
	*rgb_p++ = clip(y1 + ws.conv_bu[u]);
#else
	// block (2)
	*rgb_p++ = y0              + ws.conv_rv[v];
	*rgb_p++ = y0 - ws.conv_gu[u] - ws.conv_gv[v];
	*rgb_p++ = y0 + ws.conv_bu[u];

	*rgb_p++ = y1              + ws.conv_rv[v];
	*rgb_p++ = y1 - ws.conv_gu[u] - ws.conv_gv[v];
	*rgb_p++ = y1 + ws.conv_bu[u];
#endif
      }
    }
  }else{                                             // Do not swap coordinates
    rgb_p = frame.p;
    for(int i = 0; i < half_img_size; i++){         // 1/2 size, since 2 px
                                                    // per iter (y0, y1)
      /**********************************************************************
       * The YUYV stores the YCbCr info in the order:                       *
       *                                                                    *
       * start+0: Y_00  Cb_00 Y_01 Cr_00 Y_02 Cb_01 Y_03 Cr_01              *
       * start+8: Y_10  Cb_10 ...                                           *
       *                                                                    *
       * (cf. V4L2 API, p. 42)                                              *
       **********************************************************************/
      y0 = buf[0];
      u  = buf[1];
      y1 = buf[2];
      v  = buf[3];
      buf +=4;
      
      /* Replaced block (1) with block (2), because the clipping costs a lot of CPU */
      // block (1)
#if 1
      *rgb_p++ = clip(y0              + ws.conv_rv[v]);
      *rgb_p++ = clip(y0 - ws.conv_gu[u] - ws.conv_gv[v]);
      *rgb_p++ = clip(y0 + ws.conv_bu[u]);
	 
      *rgb_p++ = clip(y1              + ws.conv_rv[v]);
      *rgb_p++ = clip(y1 - ws.conv_gu[u] - ws.conv_gv[v]);
      *rgb_p++ = clip(y1 + ws.conv_bu[u]);
#else
      // block (2)
      *rgb_p++ = y0              + ws.conv_rv[v];
      *rgb_p++ = y0 - ws.conv_gu[u] - ws.conv_gv[v];
      *rgb_p++ = y0 + ws.conv_bu[u];

      *rgb_p++ = y1              + ws.conv_rv[v];
      *rgb_p++ = y1 - ws.conv_gu[u] - ws.conv_gv[v];
      *rgb_p++ = y1 + ws.conv_bu[u];
#endif
    }
  }
}

inline int camera::stat_device(const UVCCameraWorkspace* const ws)
{
   struct stat buf;
   int ret = fstat(ws->device_file_desc_, &buf);
   if(ret == -1)
   {
      switch(errno)
      {
         case EIO:
            ERROR("I/O error occurred while reading from the file system");
         case EBADF:
            ERROR("The file descriptor argument is not a valid one");
         default:
            ERROR("fstat() failed on device");
      }
   }
   else
      // TODO Output device information, if verbose mode is actived.
      return 0;
}



void camera::UVCCamera::open(){
   if (workspace_->is_initialized_)
      ERROR("device has been initialized already");

   workspace_->width_ = DEFAULT_WIDTH;
   workspace_->height_ = DEFAULT_HEIGHT;

   workspace_->device_file_desc_ = ::open(device_name, O_RDWR, 0);
   if (workspace_->device_file_desc_ == -1)
      ERROR("device cannot be opened");

   //
   // set up initial parameters
   //
   if (stat_device(workspace_) == -1)
      ERROR("Cannot stat device");
   // query camera capabilities
   if (ioctl(workspace_->device_file_desc_, VIDIOC_QUERYCAP, &workspace_->capability) == -1)
      ERROR("Cannot query device: VIDIOC_QUERYCAP");  // not a V4L2 device
   if (!(workspace_->capability.capabilities & V4L2_CAP_VIDEO_CAPTURE))
      ERROR("this is not a video capture device");

   //
   // set up data format
   //
   memset(&(workspace_->fmt), 0, sizeof(struct v4l2_format));
   workspace_->fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   workspace_->fmt.fmt.pix.width       = workspace_->width_;
   workspace_->fmt.fmt.pix.height      = workspace_->height_;
   workspace_->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
//    workspace_->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
   workspace_->fmt.fmt.pix.field       = V4L2_FIELD_ANY;

   int ret = ioctl(workspace_->device_file_desc_, VIDIOC_S_FMT, &workspace_->fmt);
   if (ret == -1)
      switch(errno)
      {
         case EAGAIN:
            ERROR("cannot set video format: VIDIOC_S_FMT + EAGAIN");
         case EIO:
            ERROR("cannot set video format: VIDIOC_S_FMT + EIO");
         default:
            ERROR("cannot set video format: VIDIOC_S_FMT");
            break;
      }

   if ((workspace_->width_ != workspace_->fmt.fmt.pix.width) |
       (workspace_->height_ != workspace_->fmt.fmt.pix.height))
   {
      workspace_->width_  = workspace_->fmt.fmt.pix.width;
      workspace_->height_ = workspace_->fmt.fmt.pix.height;
      std::cout << "device dictates change of resolution:" << workspace_->width_ <<'x' <<workspace_->height_ <<std::endl;
   }

   switch (workspace_->fmt.fmt.pix.pixelformat)                // Determine buffer size
   {
      case V4L2_PIX_FMT_YUYV:
         workspace_->frame_size_ = workspace_->width_ * workspace_->height_ * 2;
         break;
      default:
         ERROR("Cannot set pixel format: unknown format");
         break;
   }

   //
   // set up memory buffers
   //
   if (!(workspace_->capability.capabilities & V4L2_CAP_STREAMING))
      ERROR("Cannot initialize MMAP: device does not support streaming");

   // requesting the default number of buffers for mmapping.
   memset(&workspace_->req, 0, sizeof(v4l2_requestbuffers));
   workspace_->req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   workspace_->req.count  = DEFAULT_NUM_BUFFERS;
   workspace_->req.memory = V4L2_MEMORY_MMAP;

   if (ioctl(workspace_->device_file_desc_, VIDIOC_REQBUFS, &workspace_->req) == -1)
      ERROR("Cannot initialize MMAP: VIDIOC_REQBUFS");
   if (workspace_->req.count < 2)
      ERROR("Cannot initialize MMAP: VIDIOC_REQBUFS + insufficient memory");

   // Enqueue the number of buffers which were requested (and granted), followed
   // by memory mapping them into the address space of our application.
   workspace_->buffers_.resize(workspace_->req.count);
   for (unsigned int i = 0; i < workspace_->req.count; ++i)
   {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(struct v4l2_buffer));

      buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory      = V4L2_MEMORY_MMAP;
      buf.index       = i;

      if (ioctl(workspace_->device_file_desc_, VIDIOC_QUERYBUF, &buf) == -1)
         ERROR("Cannot initialize MMAP: VIDIOC_QUERYBUF");

      void* start = mmap(
                         NULL,                        // let mmap choose address
                         buf.length,                  // buffer length
                         PROT_READ | PROT_WRITE,      // allow read and write
                         MAP_SHARED,                  // data changes are shared
                         workspace_->device_file_desc_, // the devices's fd
                         buf.m.offset                 // offset in dev memory
                        );

      if (start == MAP_FAILED)
         ERROR("Cannot initialize MMAP: memory could not be mapped");

      workspace_->buffers_.p[i].start = start;
      workspace_->buffers_.p[i].length = buf.length;
   }

   enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   if (ioctl(workspace_->device_file_desc_, VIDIOC_STREAMON, &type))
      ERROR("cannot start streaming");

   workspace_->is_initialized_ = true;

   step();
}

void camera::UVCCamera::close(){
   if(workspace_->is_initialized_)
   {
      // stop capturing
      enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      int ret = ioctl(workspace_->device_file_desc_, VIDIOC_STREAMOFF, &type);
      if(ret < 0)
         ERROR("Cannot stop capturing: VIDIOC_STREAMOFF");

      // undo mmap
      for(uint i = 0; i < workspace_->buffers_.N; i++)
         munmap(workspace_->buffers_.p[i].start, workspace_->buffers_.p[i].length);

      ret = ::close(workspace_->device_file_desc_);
      if(ret == 0)
         workspace_->device_file_desc_ = -1;

      std::cout << "UVCCamera is clean!" << std::endl;
   }
}

void camera::UVCCamera::step()
{
   int ret;                                           // return value of iotcl
   fd_set fds;                                        // FD set for select()
   struct timeval tv;                                 // to specify a timeout

   // Add <fd> to a file descriptor set <fds> and make select() notify the
   // application as soon as a buffer becomes ready. Then, read this buffer
   // according to the applied I/O method.
   FD_ZERO(&fds);
   FD_SET(workspace_->device_file_desc_, &fds);
   tv.tv_sec  = 2;                                    // Set the timeout to 2s
   tv.tv_usec = 0;
   ret = select(workspace_->device_file_desc_+1, &fds, NULL, NULL, &tv);
   if (ret == 0)
   {ERROR("Cannot capture rawframe: timeout");}
   else if(ret < 0 && errno != EINTR)                 // Something bad happened
     ERROR("Cannot capture rawframe: select(...) failed");

   // retrieve frame data
   struct v4l2_buffer buf;
   memset(&buf, 0, sizeof(struct v4l2_buffer));
   buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   buf.memory = V4L2_MEMORY_MMAP;
   ret = ioctl(workspace_->device_file_desc_, VIDIOC_DQBUF, &buf);     // dequeue buffer
   
   output.writeAccess(this);
   // check size of output memory
   if (output.rgbL.N != workspace_->width_ * workspace_->height_ * 3)
     output.rgbL.resize(workspace_->height_, workspace_->width_, 3);
   yuyv2rgb(output.rgbL, workspace_->buffers_.p[buf.index], *workspace_);   // convert + copy to output
   output.rgbR=output.rgbL;
   output.deAccess(this);
   
   if (ioctl(workspace_->device_file_desc_, VIDIOC_QBUF, &buf) == -1)
     ERROR("cannot queue buffer");
}


