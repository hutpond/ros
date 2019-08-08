#ifndef APPLICATION_H
#define APPLICATION_H

#include <boost/smart_ptr/shared_ptr.hpp>

class ReadFiles;
class MessagePublish;

class Application
{
public:
  Application();
  void start();
  void stop();

private:
  boost::shared_ptr<ReadFiles> read_files_ptr_;
  boost::shared_ptr<MessagePublish> msg_publish_ptr_;
};

#endif // APPLICATION_H
