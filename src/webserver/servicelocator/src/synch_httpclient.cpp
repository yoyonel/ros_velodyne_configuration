#include "synch_httpclient.h"

#define ROS_WARN_STREAM(...)
#define ROS_INFO_STREAM(...)

void SynchHTTPClient::get(const std::string &_server,
                          const std::string &_path)
{
    try
    {
        ptrClientImp_->handle_request_for_GET(_server, _path);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
    }
}

void SynchHTTPClient::post(const std::string &_server,
                           const std::string &_path,
                           const std::string &_xwwwformcoded)
{
    try
    {
        ptrClientImp_->handle_request_for_POST(_server, _path, _xwwwformcoded);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
    }
}
