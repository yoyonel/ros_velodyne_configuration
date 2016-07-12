#include "synch_httpclient.h"

#include "synch_httpclient_imp.h"   // for: SynchHTTPClientImp

#define ROS_WARN_STREAM(...)
#define ROS_INFO_STREAM(...)


SynchHTTPClient::SynchHTTPClient(boost::asio::io_service& _io_service)
    : ptrClientImp_(new SynchHTTPClientImp(_io_service))
{

}

const std::string & SynchHTTPClient::get_response() const
{
    ptrClientImp_->get_response();
}

void SynchHTTPClient::get(const std::string &_server,
                          const std::string &_path)
{
    try
    {
        ptrClientImp_->get(_server, _path);
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
        ptrClientImp_->post(_server, _path, _xwwwformcoded);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
    }
}
