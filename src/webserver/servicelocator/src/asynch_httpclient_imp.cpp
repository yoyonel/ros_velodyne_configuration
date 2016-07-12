#include "asynch_httpclient_imp.h"
#include <boost/bind.hpp>   // for: boost::bind
#include <iostream>

int ASynchHTTPClientImp::perform_request(const std::string& _server)
{
    // Start an asynchronous resolve to translate the server and service names
    // into a list of endpoints.
    tcp::resolver::query query(_server, "http");
    resolver_.async_resolve(query,
                            boost::bind(&ASynchHTTPClientImp::handle_resolve, this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::iterator));
    this->run_io_service();
}

int ASynchHTTPClientImp::handle_request_for_GET(const std::string& _server, const std::string& _path)
{
    BUILD_REQUEST_GET(request_stream_, _server, _path);
    perform_request(_server);
}

int ASynchHTTPClientImp::handle_request_for_POST(const std::string& _server,
                                                 const std::string& _path,
                                                 const std::string& _xwwwformcoded)
{
    BUILD_REQUEST_POST(request_stream_, _server, _path, _xwwwformcoded);
    perform_request(_server);
}

void ASynchHTTPClientImp::handle_resolve(const boost::system::error_code& err,
                                         tcp::resolver::iterator endpoint_iterator)
{
    if (!err)
    {
        // Attempt a connection to each endpoint in the list until we
        // successfully establish a connection.
        boost::asio::async_connect(socket_, endpoint_iterator,
                                   boost::bind(&ASynchHTTPClientImp::handle_connect, this,
                                               boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err.message() << "\n";
    }
}

void ASynchHTTPClientImp::handle_connect(const boost::system::error_code& err)
{
    if (!err)
    {
        // The connection was successful. Send the request.
        boost::asio::async_write(socket_, request_,
                                 boost::bind(&ASynchHTTPClientImp::handle_write_request, this,
                                             boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err.message() << "\n";
    }
}

void ASynchHTTPClientImp::handle_write_request(const boost::system::error_code& err)
{
    if (!err)
    {
        // Read the response status line. The response_ streambuf will
        // automatically grow to accommodate the entire line. The growth may be
        // limited by passing a maximum size to the streambuf constructor.
        boost::asio::async_read_until(socket_, response_, "\r\n",
                                      boost::bind(&ASynchHTTPClientImp::handle_read_status_line, this,
                                                  boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err.message() << "\n";
    }
}

void ASynchHTTPClientImp::handle_read_status_line(const boost::system::error_code& err)
{
    if (!err)
    {
        // Check that response is OK.
        std::istream response_stream(&response_);
        std::string http_version;
        response_stream >> http_version;
        unsigned int status_code;
        response_stream >> status_code;
        std::string status_message;
        std::getline(response_stream, status_message);
        if (!response_stream || http_version.substr(0, 5) != "HTTP/")
        {
            std::cout << "Invalid response\n";
            return;
        }
        if (status_code != 200)
        {
            std::cout << "Response returned with status code ";
            std::cout << status_code << "\n";
            return;
        }

        // Read the response headers, which are terminated by a blank line.
        boost::asio::async_read_until(socket_, response_, "\r\n\r\n",
                                      boost::bind(&ASynchHTTPClientImp::handle_read_headers, this,
                                                  boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err << "\n";
    }
}

void ASynchHTTPClientImp::handle_read_headers(const boost::system::error_code& err)
{
    if (!err)
    {
        // Process the response headers.
        std::istream response_stream(&response_);
        std::string header;
        while (std::getline(response_stream, header) && header != "\r")
            //            std::cout << "HEADER: " << header << "\n";
            ;
        //          std::cout << "\n";

        // Write whatever content we already have to output.
        if (response_.size() > 0)
        {
            //            std::cout << &response_;
//            std::istream(&response_) >> str_response_;
//            std::istream buffer( &response );
            // url: http://stackoverflow.com/a/1899908
            std::stringstream string_buffer;
            response_stream >> string_buffer.rdbuf();
            str_response_ = string_buffer.str();
        }

        // Start reading remaining data until EOF.
        boost::asio::async_read(socket_, response_,
                                boost::asio::transfer_at_least(1),
                                boost::bind(&ASynchHTTPClientImp::handle_read_content, this,
                                            boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err << "\n";
    }
}

void ASynchHTTPClientImp::handle_read_content(const boost::system::error_code& err)
{
    if (!err)
    {
        // Write all of the data that has been read so far.
                  std::cout << &response_;
        //            std::istream(&response_) >> str_response_;

        // Continue reading remaining data until EOF.
        boost::asio::async_read(socket_, response_,
                                boost::asio::transfer_at_least(1),
                                boost::bind(&ASynchHTTPClientImp::handle_read_content, this,
                                            boost::asio::placeholders::error));
    }
    else if (err != boost::asio::error::eof)
    {
        std::cout << "Error: " << err << "\n";
    }
}
