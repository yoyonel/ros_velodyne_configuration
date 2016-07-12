#ifndef SYNCH_HTTPCLIENT_IMP_H
#define SYNCH_HTTPCLIENT_IMP_H

#include "boost_httpclient.h"

// ----------------
// SERVICE Providers
// ----------------

// ----------
// Implementations
// ----------
class SynchHTTPClientImp : public BoostHTTPClient
{
public:
    SynchHTTPClientImp(boost::asio::io_service& _io_service) : BoostHTTPClient(_io_service) {}

    int get(const std::string& _server, const std::string& _path) override;
    int post(const std::string& server, const std::string& path, const std::string& _xwwwformcoded) override;

protected:
    int perform_request(const std::string& _server);
};

#endif // SYNCH_HTTPCLIENT_IMP_H
