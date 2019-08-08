#include "application.h"
#include "read_files.h"
#include "message_publish.h"

Application::Application()
{
  read_files_ptr_.reset(new ReadFiles);
  msg_publish_ptr_.reset(new MessagePublish);

  boost::function<void(const perception_msg_publish::TrackTargetColl &)> fun =
      boost::bind(&MessagePublish::publish_track_target,
                  msg_publish_ptr_,
                  _1);
  read_files_ptr_->setPubTrackFun(fun);

  boost::function<void(const perception_msg_publish::UltrasonicTargetColl &)> fun_ultrasonic =
      boost::bind(&MessagePublish::publish_ultrasonic_target,
                  msg_publish_ptr_,
                  _1);
  read_files_ptr_->setPubUltrasonicFun(fun_ultrasonic);

  boost::function<void(const perception_msg_publish::msflOutput &)> fun_msfloutput =
      boost::bind(&MessagePublish::publish_msfl_output,
                  msg_publish_ptr_,
                  _1);
  read_files_ptr_->setPubMsflOutputFun(fun_msfloutput);
}

void Application::start()
{
  read_files_ptr_->start_read();
}

void Application::stop()
{
  read_files_ptr_->stop_read();
}
