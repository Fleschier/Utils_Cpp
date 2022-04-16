#ifndef AVH264ENCODER_H
#define AVH264ENCODER_H

#include "avh264encoder_global.h"

#include <opencv2/opencv.hpp>

#define __STDC_CONSTANT_MACROS
//#ifdef __cplusplus
extern "C" {
//#endif
#include <libavutil/time.h>
#include <libavutil/mathematics.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
//#include <libavdevice/avdevice.h>
#include <libswresample/swresample.h>
//#ifdef __cplusplus
}
//#endif

typedef struct AvH264EncConfig_T {

	int width = 1920;
	int height = 1080;
	int frame_rate = 30;
	int64_t bit_rate = 1000000;
	int gop_size = 250;
	int max_b_frames = 0;
}AvH264EncConfig;


class AVH264ENCODERSHARED_EXPORT AvH264Encoder
{

public:
	AvH264Encoder();
	int open(AvH264EncConfig h264_config);
	AVPacket *encode(cv::Mat mat);
	void close();
private:
	const AVCodec *cdc_;
	AVCodecContext *cdc_ctx_;
	AVFrame *avf_;
	AVPacket *avp_;
	int frame_size_;
	int pts_;
};


#endif // AVH264ENCODER_H
