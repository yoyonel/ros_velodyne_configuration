#ifndef LOCATOR_HTTPCLIENT_H
#define LOCATOR_HTTPCLIENT_H

#include "httpclient.h" // for: HTTPClient
#include "null_httpclient.h"    // for: NullHTTPClient

// ----------------
// Locator
// ----------------

class LocatorHTTPClient
{
public:
    static void initialize() { service_ = &nullService_; }

    static HTTPClient* getHTTPClient() { return service_; }

    static void provide(HTTPClient* service)
    {
        if(!service) service_ = &nullService_;

        service_ = service;
    }

private:
    static HTTPClient* service_;
    static NullHTTPClient nullService_;
};

#endif // LOCATOR_HTTPCLIENT_H
