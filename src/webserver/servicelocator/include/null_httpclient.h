#ifndef NULL_HTTPCLIENT_H
#define NULL_HTTPCLIENT_H

#include "httpclient.h" // for: HTTPClient

// ----------
// Interfaces
// ----------
class NullHTTPClient : public HTTPClient
{
public:    
    virtual void get(const std::string &_server, const std::string &_path) {}
    virtual void post(const std::string &_server, const std::string &_path, const std::string &_xwwwformcoded) {}
    virtual const std::string & get_response() const { return nullString; }
private:
    static const std::string nullString;
};

const std::string NullHTTPClient::nullString = std::string("");

#endif // NULL_HTTPCLIENT_H
