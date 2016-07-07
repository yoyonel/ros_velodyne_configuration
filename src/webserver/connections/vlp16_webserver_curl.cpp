#include <vlp16_webserver.h>


using namespace vlp16_webserver;

// ----------------------------------
// CURL
// ----------------------------------
std::string VLP16_WebServer_CURL::request(
        const WebServerCommands & _cmd
        ) const
{
    // url: http://stackoverflow.com/questions/23936246/error-invalid-operands-of-types-const-char-35-and-const-char-2-to-binar
    const std::string cmd_curl =                    \
            "curl -s http://" +                     \
            network_sensor_ip_ +                   \
            "/cgi/" + _cmd._to_string() + ".json";

    const std::string cmd_bash =
            (max_delay_for_cmd_ != 0.0f ?                                  \
                "timeout " + std::to_string(max_delay_for_cmd_) + "s " :   \
                "")                                                        \
            + cmd_curl;
    //
    ROS_INFO_STREAM("Commande bash: " << cmd_bash);

    return velodyne_tools::exec_cmd(cmd_bash.c_str());
}

int VLP16_WebServer_CURL::send(
        const VLP16_settingsConfig& _config
        ) const
{
    const std::string curl_settings = convert_config_to_xwwwformcoded(_config);

    //     From: man curl
    //    -d, --data <data>
    //            (HTTP)  Sends  the  specified data in a POST request to the HTTP server, in the same way that a browser does when a user has filled in an HTML
    //            form and presses the submit button. This will cause curl to pass the data to the server using the  content-type  application/x-www-form-urlen?
    //                coded.  Compare to -F, --form.
    //
    //            -d,  --data  is the same as --data-ascii. To post data purely binary, you should instead use the --data-binary option. To URL-encode the value
    //            of a form field you may use --data-urlencode.
    //
    //            If any of these options is used more than once on the same command line, the data pieces specified will be merged together with  a  separating
    //            &-symbol. Thus, using '-d name=daniel -d skill=lousy' would generate a post chunk that looks like 'name=daniel&skill=lousy'.
    //
    //            If  you  start  the  data with the letter @, the rest should be a file name to read the data from, or - if you want curl to read the data from
    //            stdin. Multiple files can also be specified. Posting data from a file named 'foobar' would thus be done with --data @foobar.  When  --data  is
    //            told to read from a file like that, carriage returns and newlines will be stripped out.

    const std::string cmd_curl = \
            "curl -X POST http://" + \
            network_sensor_ip_ + \
            "/cgi/setting --data '" + \
            curl_settings + \
            "'"
            ;

    const std::string cmd =
            (max_delay_for_cmd_ != 0 ?                                     \
                "timeout " + std::to_string(max_delay_for_cmd_) + "s " :   \
                "")                                                         \
            + cmd_curl;
    //
    ROS_INFO_STREAM("Commande bash: " << cmd);

    const std::string ret_cmd = velodyne_tools::exec_cmd(cmd.c_str());

    return 1;
}

