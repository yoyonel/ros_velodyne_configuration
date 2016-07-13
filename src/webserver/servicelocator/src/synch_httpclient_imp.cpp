#include "synch_httpclient_imp.h"

#define ROS_WARN_STREAM(...)
#define ROS_INFO_STREAM(...)

int SynchHTTPClientImp::perform_request(const std::string& _server)
{
    int retour = 1;

    try {
        // Get a list of endpoints corresponding to the server name.
        tcp::resolver::query query(_server, "http");
        tcp::resolver::iterator endpoint_iterator = resolver_.resolve(query);

        // Try each endpoint until we successfully establish a connection.
        boost::asio::connect(socket_, endpoint_iterator);

        // Send the request.
        boost::asio::write(socket_, request_);

        // Read the response status line. The response streambuf will automatically
        // grow to accommodate the entire line. The growth may be limited by passing
        // a maximum size to the streambuf constructor.
        boost::asio::streambuf response;
        boost::asio::read_until(socket_, response, "\r\n");

        // Check that response is OK.
        std::istream response_stream(&response);
        std::string http_version;
        response_stream >> http_version;
        unsigned int status_code;
        response_stream >> status_code;
        std::string status_message;
        std::getline(response_stream, status_message);
        if (!response_stream || http_version.substr(0, 5) != "HTTP/")
        {
            ROS_WARN_STREAM("Invalid response\n");
            //                return response_json;
            return -1;
        }
        if (status_code != 200)
        {
            ROS_WARN_STREAM("Response returned with status code " << status_code << "\n");
            //                return response_json;
            return -2;
        }

        // Read the response headers, which are terminated by a blank line.
        boost::asio::read_until(socket_, response, "\r\n\r\n");

        // Process the response headers.
        std::string header;
        while (std::getline(response_stream, header) && header != "\r")
            //             ROS_INFO_STREAM("header: " << header << "\n");
            ;
        ROS_INFO_STREAM("\n");

        ROS_INFO_STREAM("response.size(): " << response.size());
        // Write whatever content we already have to output.
        if (response.size() > 0)
        {
            //             ROS_INFO_STREAM("response: " << &response);   // ps: ca consumme les donnees !

            // urls:
            // - http://stackoverflow.com/questions/1899750/how-do-i-convert-a-boostasiostreambuf-into-a-stdstring
            // - http://stackoverflow.com/a/2546953
            //            std::istream(&response) >> str_response_;
            //
            // url: http://stackoverflow.com/a/1899908
            std::istream buffer( &response );
            std::stringstream string_buffer;
            buffer >> string_buffer.rdbuf();
            str_response_ = string_buffer.str();

            ROS_INFO_STREAM("str_response_: " << str_response_);
        }

        ROS_INFO_STREAM("response.size(): " << response.size());

        // Read until EOF, writing data to output as we go.
        boost::system::error_code error;
        while (boost::asio::read(socket_, response,
                                 boost::asio::transfer_at_least(1), error))
            ROS_INFO_STREAM(&response);
        ;

        ROS_INFO_STREAM("response.size(): " << response.size());

        if (error != boost::asio::error::eof)
            throw boost::system::system_error(error);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
        retour = 0;
    }
    return retour;
}

int SynchHTTPClientImp::get(const std::string& _server,
                            const std::string& _path)
{
    BUILD_REQUEST_GET(request_stream_, _server, _path);
    ROS_INFO_STREAM("Request: " << request_stream_);

    return perform_request(_server);

}

int SynchHTTPClientImp::post(const std::string& _server,
                             const std::string& _path,
                             const std::string& _xwwwformcoded)
{
    BUILD_REQUEST_POST(request_stream_, _server, _path, _xwwwformcoded);
    ROS_INFO_STREAM("Request: " << request_stream_);

    return perform_request(_server);

}
