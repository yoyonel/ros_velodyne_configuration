#include <vlp16_webserver_services_diagnostics.h>

using namespace vlp16_webserver_services;


Velodyne_WebServer_Diagnostics::Velodyne_WebServer_Diagnostics(const std::string& _name) : Velodyne_WebServer_Services(_name )
{
}

void Velodyne_WebServer_Diagnostics::run()
{
    Velodyne_WebServer_Services<
            VLP16_DiagnosticsService,
            VLP16_DiagnosticsMessage,
            VLP16_DiagnosticsRawServiceResponse>::run_with_test_sub();
}

bool Velodyne_WebServer_Diagnostics::get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse& _res) const
{
    const std::string res_request = request(velodyne_webserver::Velodyne_WebServer::WebServerCommands::diag);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );
    return parse_JSON(res_request, _res);
}

bool Velodyne_WebServer_Diagnostics::get_response(velodyne_configuration::VLP16_DiagnosticsServiceResponse& _res)
{
    velodyne_configuration::VLP16_DiagnosticsRawServiceResponse res_raw;
    get_diagnostics_raw(res_raw);
    return scale_volt_temp(res_raw.msg, _res.msg);
}

bool Velodyne_WebServer_Diagnostics::parse_JSON(const std::string & _res_request, VLP16_DiagnosticsRawServiceResponse & _res) const
{
    // ------------------------------------
    // Manual JSON file parsing
    // ------------------------------------
    // urls:
    // - http://zenol.fr/blog/boost-property-tree/en.html
    // - https://gist.github.com/mloskot/1509935
    try
    {
        JSON_INIT(root, _res_request);

        JSON_READ_UINT16(root, volt_temp.bot.i_out, _res, bot_i_out);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_1_2v, _res, bot_pwr_1_2v);
        JSON_READ_UINT16(root, volt_temp.bot.lm20_temp, _res, bot_lm20_temp);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_5v, _res, bot_pwr_5v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_2_5v, _res, bot_pwr_2_5v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_3_3v, _res, bot_pwr_3_3v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_v_in, _res, bot_pwr_v_in);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_1_25v, _res, bot_pwr_1_25v);

        JSON_READ_UINT16(root, volt_temp.top.lm20_temp, _res, top_lm20_temp);
        JSON_READ_UINT16(root, volt_temp.top.hv, _res, top_hv);
        JSON_READ_UINT16(root, volt_temp.top.ad_temp, _res, top_ad_temp);
        JSON_READ_UINT16(root, volt_temp.top.pwr_5v, _res, top_pwr_5v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_2_5v, _res, top_pwr_2_5v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_3_3v, _res, top_pwr_3_3v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_vccint, _res, top_pwr_vccint);

        JSON_READ_UINT16(root, vhv, _res, vhv);
        JSON_READ_UINT16(root, adc_nf, _res, adc_nf);
        JSON_READ_UINT16(root, ixe, _res, ixe);
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    // ------------------------------------

    return true;
}

bool Velodyne_WebServer_Diagnostics::scale_volt_temp(
        VLP16_DiagnosticsRawMessage &_msg_raw,
        VLP16_DiagnosticsMessage    &_msg
        ) const
{
    const float scale_2x_vref = 5.0/4096;

    try
    {
        //        volt_temp.top.hv = hdltop_volts_to_hv(volt_temp.top.hv);
        //        volt_temp.top.lm20_temp = lm20_volts_to_degCel(volt_temp.top.lm20_temp);
        //        volt_temp.top.pwr_5v *= 2.0;
        //        volt_temp.top.pwr_5v_raw *= 2.0;
        _msg.top_hv = hdltop_volts_to_hv(_msg_raw.top_hv * scale_2x_vref);
        _msg.top_lm20_temp = lm20_volts_to_degCel(_msg_raw.top_lm20_temp * scale_2x_vref);
        _msg.top_pwr_5v = _msg_raw.top_pwr_5v * 2.0 * scale_2x_vref;
        _msg.top_pwr_5v_raw = _msg_raw.top_pwr_5v_raw * 2.0 * scale_2x_vref;
        _msg.top_ad_temp = _msg_raw.top_ad_temp * scale_2x_vref;
        _msg.top_pwr_2_5v = _msg_raw.top_pwr_2_5v * scale_2x_vref;
        _msg.top_pwr_3_3v = _msg_raw.top_pwr_3_3v * scale_2x_vref;
        _msg.top_pwr_vccint = _msg_raw.top_pwr_vccint * scale_2x_vref;

        //        volt_temp.bot.i_out = acs17_volts_to_amps(volt_temp.bot.i_out);
        //        volt_temp.bot.lm20_temp = lm20_volts_to_degCel(volt_temp.bot.lm20_temp);
        //        volt_temp.bot.pwr_5v *= 2.0;
        //        volt_temp.bot.pwr_v_in*= 11.0;
        _msg.bot_i_out = acs17_volts_to_amps(_msg_raw.bot_i_out * scale_2x_vref);
        _msg.bot_lm20_temp = lm20_volts_to_degCel(_msg_raw.bot_lm20_temp * scale_2x_vref);
        _msg.bot_pwr_5v = _msg_raw.bot_pwr_5v * 2.0 * scale_2x_vref;
        _msg.bot_pwr_v_in = _msg_raw.bot_pwr_v_in * 11.0 * scale_2x_vref;
        _msg.bot_pwr_2_5v = _msg_raw.bot_pwr_2_5v * scale_2x_vref;
        _msg.bot_pwr_3_3v = _msg_raw.bot_pwr_3_3v * scale_2x_vref;
        _msg.bot_pwr_1_25v = _msg_raw.bot_pwr_1_25v * scale_2x_vref;
        _msg.bot_pwr_1_2v = _msg_raw.bot_pwr_1_2v * scale_2x_vref;

        _msg.vhv = _msg_raw.vhv;
        _msg.ixe = _msg_raw.ixe;
        _msg.adc_nf = _msg_raw.adc_nf;
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}
