#include <velodyne_tools_boost_asio.h>

namespace velodyne_tools {
namespace boost_asio {

client_synch::client_synch(
        boost::asio::io_service& io_service,
        const std::string& server,
        const std::string& path
        )
    : resolver_(io_service),
      socket_(io_service),
      request_stream_(&request_)
{
    str_response_ = "";
    try
    {
        handle_request_for_GET(server, path);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
    }
}

client_synch::client_synch(
        boost::asio::io_service& io_service,
        const std::string& server,
        const std::string& path,
        const std::string& xwwwformcoded
        )
    : resolver_(io_service),
      socket_(io_service),
      request_stream_(&request_)
{
    try
    {
        handle_request_for_POST(server, path, xwwwformcoded);
    }
    catch (std::exception& e)
    {
        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
    }
}

int client_synch::perform_request(const std::string& _server)
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
            // ROS_INFO_STREAM("header: " << header << "\n");
            ;
        // ROS_INFO_STREAM("\n");

        // Write whatever content we already have to output.
        if (response.size() > 0)
        {
            // ROS_INFO_STREAM("response: " << &response);   // ps: ca consumme les donnees !

            // urls:
            // - http://stackoverflow.com/questions/1899750/how-do-i-convert-a-boostasiostreambuf-into-a-stdstring
            // - http://stackoverflow.com/a/2546953
            std::istream(&response) >> str_response_;
            // ROS_INFO_STREAM("str_response_: " << str_response_);
        }

        // Read until EOF, writing data to output as we go.
        boost::system::error_code error;
        while (boost::asio::read(socket_, response,
                                 boost::asio::transfer_at_least(1), error))
            //ROS_INFO_STREAM(&response);
            ;

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

int client_synch::handle_request_for_GET(const std::string& server, const std::string& path)
{
    BUILD_REQUEST_GET(request_stream_, server, path);
    //        ROS_INFO_STREAM("Request: " << request_stream);

    return perform_request(server);
}

int client_synch::handle_request_for_POST(const std::string& server, const std::string& path, const std::string& xwwwformcoded)
{
    BUILD_REQUEST_POST(request_stream_, server, path, xwwwformcoded);
    ROS_INFO_STREAM("Request: " << request_stream_);

    return perform_request(server);
}

client_asynch::client_asynch(boost::asio::io_service& io_service,
                             const std::string& server,
                             const std::string& path
                             )
    : resolver_(io_service), socket_(io_service)
{
    std::ostream request_stream(&request_);
    BUILD_REQUEST_GET(request_stream, server, path);

    // Start an asynchronous resolve to translate the server and service names
    // into a list of endpoints.
    tcp::resolver::query query(server, "http");
    resolver_.async_resolve(query,
                            boost::bind(&client_asynch::handle_resolve, this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::iterator));
}

client_asynch::client_asynch(boost::asio::io_service& io_service,
                             const std::string& server,
                             const std::string& path,
                             const std::string& xwwwformcoded
                             )
    : resolver_(io_service), socket_(io_service)
{
    //    ROS_INFO_STREAM("xwwwformcoded: " << xwwwformcoded);

    std::ostream request_stream(&request_);
    BUILD_REQUEST_POST(request_stream, server, path, xwwwformcoded);

    // Start an asynchronous resolve to translate the server and service names
    // into a list of endpoints.
    tcp::resolver::query query(server, "http");
    resolver_.async_resolve(query,
                            boost::bind(&client_asynch::handle_resolve, this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::iterator));
}

void client_asynch::handle_resolve(const boost::system::error_code& err,
                                   tcp::resolver::iterator endpoint_iterator)
{
    if (!err)
    {
        // Attempt a connection to each endpoint in the list until we
        // successfully establish a connection.
        boost::asio::async_connect(socket_, endpoint_iterator,
                                   boost::bind(&client_asynch::handle_connect, this,
                                               boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err.message() << "\n";
    }
}

void client_asynch::handle_connect(const boost::system::error_code& err)
{
    if (!err)
    {
        // The connection was successful. Send the request.
        boost::asio::async_write(socket_, request_,
                                 boost::bind(&client_asynch::handle_write_request, this,
                                             boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err.message() << "\n";
    }
}

void client_asynch::handle_write_request(const boost::system::error_code& err)
{
    if (!err)
    {
        // Read the response status line. The response_ streambuf will
        // automatically grow to accommodate the entire line. The growth may be
        // limited by passing a maximum size to the streambuf constructor.
        boost::asio::async_read_until(socket_, response_, "\r\n",
                                      boost::bind(&client_asynch::handle_read_status_line, this,
                                                  boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err.message() << "\n";
    }
}

void client_asynch::handle_read_status_line(const boost::system::error_code& err)
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
                                      boost::bind(&client_asynch::handle_read_headers, this,
                                                  boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err << "\n";
    }
}

void client_asynch::handle_read_headers(const boost::system::error_code& err)
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
            std::istream(&response_) >> str_response_;
        }

        // Start reading remaining data until EOF.
        boost::asio::async_read(socket_, response_,
                                boost::asio::transfer_at_least(1),
                                boost::bind(&client_asynch::handle_read_content, this,
                                            boost::asio::placeholders::error));
    }
    else
    {
        std::cout << "Error: " << err << "\n";
    }
}

void client_asynch::handle_read_content(const boost::system::error_code& err)
{
    if (!err)
    {
        // Write all of the data that has been read so far.
        //          std::cout << &response_;
        //            std::istream(&response_) >> str_response_;

        // Continue reading remaining data until EOF.
        boost::asio::async_read(socket_, response_,
                                boost::asio::transfer_at_least(1),
                                boost::bind(&client_asynch::handle_read_content, this,
                                            boost::asio::placeholders::error));
    }
    else if (err != boost::asio::error::eof)
    {
        std::cout << "Error: " << err << "\n";
    }
}

}
}
