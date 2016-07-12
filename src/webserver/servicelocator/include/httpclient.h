#ifndef HTTPCLIENT_H
#define HTTPCLIENT_H

#include <iostream>

// ----------------
// SERVICE
// ----------------
// ----------------
// INTERFACE
// ----------------
class HTTPClient
{
public:
    // ----------------
    // VIRTUALS
    // ----------------
    virtual ~HTTPClient() {}

    // ----------------
    // PURES VIRTUALS
    // ----------------
    virtual void get(const std::string& _server, const std::string& _path) = 0;
    virtual void post(const std::string& _server, const std::string& _path, const std::string& _xwwwformcoded) = 0;
    virtual const std::string & get_response() const = 0;
    // ----------------
};

#endif // HTTPCLIENT_H
