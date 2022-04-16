#include "mrtspserver.h"

#include "net/Timer.h"
#include <thread>
#include <memory>
#include <iostream>
#include <string>

#include<QDebug>

#include "avh264encoder.h"

RTSPServer::RTSPServer(){
  AvH264EncConfig conf;
  conf.bit_rate = 1000000;
  conf.width = 1920;
  conf.height = 1080;
  conf.gop_size = 250;
  conf.max_b_frames = 0;
  conf.frame_rate = 30;

  m_h264_encoder = new AvH264Encoder();
  m_h264_encoder->open(conf);

  event_loop.reset(new xop::EventLoop());

}

RTSPServer::~RTSPServer(){
  if(m_h264_encoder != nullptr){
      m_h264_encoder->close();
      delete m_h264_encoder;
      m_h264_encoder = nullptr;
    }
}

void RTSPServer::Init(std::string m_port){
  std::string suffix = "live";
  std::string ip = "127.0.0.1";
  if(!m_port.empty())
    port = m_port;
  std::string rtsp_url = "rtsp://" + ip + ":" + port + "/" + suffix;

//  std::shared_ptr<xop::EventLoop> event_loop(new xop::EventLoop());
//  std::shared_ptr<xop::RtspServer> server = xop::RtspServer::Create(event_loop.get());

//  if (!server->Start("0.0.0.0", atoi(port.c_str()))) {
//          printf("RTSP Server listen on %s failed.\n", port.c_str());
//          return;
//  }

//  #ifdef AUTH_CONFIG
//          server->SetAuthConfig("-_-", "admin", "12345");
//  #endif

//  xop::MediaSession *session = xop::MediaSession::CreateNew("live");

  server = xop::RtspServer::Create(event_loop.get());
  if (!server->Start("0.0.0.0", atoi(port.c_str()))) {
          printf("RTSP Server listen on %s failed.\n", port.c_str());
          return;
  }

  session = xop::MediaSession::CreateNew("live");

  session->AddSource(xop::channel_0, xop::H264Source::CreateNew());
  //session->StartMulticast();
  session->AddNotifyConnectedCallback([] (xop::MediaSessionId sessionId, std::string peer_ip, uint16_t peer_port){
          printf("RTSP client connect, ip=%s, port=%hu \n", peer_ip.c_str(), peer_port);
  });

  session->AddNotifyDisconnectedCallback([](xop::MediaSessionId sessionId, std::string peer_ip, uint16_t peer_port) {
          printf("RTSP client disconnect, ip=%s, port=%hu \n", peer_ip.c_str(), peer_port);
  });

  session_id = server->AddSession(session);

//  std::thread t1(&RtspServer::sendFrameThread,this, server.get(), session_id,);
//          t1.detach();

  std::cout << "Play URL: " << rtsp_url << std::endl;

//  while (1) {
//          xop::Timer::Sleep(100);
//  }

//  getchar();
//  return;
}

void RTSPServer::sendFrameThread(cv::Mat mat){
  int buf_size = 2000000;

  // different unique ptr couldn't point to the same memory area
  // get() return the address the unique ptr actually point to
  // unique ptr could make sure the memory area could only be accessed by this unique_ptr in this thread
  std::unique_ptr<uint8_t> frame_buf(new uint8_t[buf_size]);    // recommended way

//  cv::imshow("test", mat);
//  char c = cv::waitKey(1);
//  qDebug() << "begin to encode one frame";
  encoded_pkt = m_h264_encoder->encode(mat);


//  bool end_of_frame = false;
//  int frame_size = h264_file->ReadFrame((char*)frame_buf.get(), buf_size, &end_of_frame);
  int frame_size = encoded_pkt->size;
  memcpy(frame_buf.get(), encoded_pkt->data, frame_size);

  if(frame_size > 0) {
//      qDebug() << "success got one frame";
      xop::AVFrame videoFrame = {0};
      videoFrame.type = 0;
      videoFrame.size = frame_size;
      videoFrame.timestamp = xop::H264Source::GetTimestamp();
      videoFrame.buffer.reset(new uint8_t[videoFrame.size]);
//          memcpy(videoFrame.buffer.get(), frame_buf.get(), videoFrame.size);
      memcpy(videoFrame.buffer.get(), frame_buf.get(), videoFrame.size);
      server.get()->PushFrame(session_id, xop::channel_0, videoFrame);
//      qDebug() << "success push one frame";

  }
  else {
      return;
  }

  xop::Timer::Sleep(25);

}
