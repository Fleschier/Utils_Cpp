#ifndef RTSPSERVER_H
#define RTSPSERVER_H

#include<opencv2/opencv.hpp>

#include "xop/RtspServer.h"

// if the header only use pointers of the class, then the include part could be in cpp
class AvH264Encoder;
class AVPacket;


class RTSPServer
{
private:
  AvH264Encoder* m_h264_encoder;

  std::shared_ptr<xop::EventLoop> event_loop;
  std::shared_ptr<xop::RtspServer> server;
  xop::MediaSession* session;

  xop::MediaSessionId session_id;

  std::string port = "8888";

  AVPacket* encoded_pkt;

public:
  RTSPServer();
  ~RTSPServer();
  void Init(std::string m_port = "8888");
  void sendFrameThread(cv::Mat mat);
};

#endif // RTSPSERVER_H
