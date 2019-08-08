#ifndef READ_FILES_H
#define READ_FILES_H

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include "perception_msg_publish/TrackTargetColl.h"
#include "perception_msg_publish/TrackTarget.h"
#include "perception_msg_publish/UltrasonicTarget.h"
#include "perception_msg_publish/UltrasonicTargetColl.h"
#include "perception_msg_publish/msflOutput.h"

class ReadFiles
{
public:
  ReadFiles();
  void start_read();
  void stop_read();

  void setPubTrackFun(boost::function<void(const perception_msg_publish::TrackTargetColl &)>);
  void setPubUltrasonicFun(boost::function<void(const perception_msg_publish::UltrasonicTargetColl &)>);
  void setPubMsflOutputFun(boost::function<void(const perception_msg_publish::msflOutput &)>);

protected:
  void on_read_files();
  void parse_json_data(int, const std::string &);
  void parse_gps_data(int, const std::string &);

private:
  bool run_read_flag_;
  boost::scoped_ptr<boost::thread> read_files_thread_;

  boost::function<void(const perception_msg_publish::TrackTargetColl &)> m_funPubTrack;
  boost::function<void(const perception_msg_publish::UltrasonicTargetColl &)> m_funPubUltrasonic;
  boost::function<void(const perception_msg_publish::msflOutput &)> m_funPubMsflOutput;
};

#endif // READ_FILES_H
