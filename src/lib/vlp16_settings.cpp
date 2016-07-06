#include <vlp16_settings.h>

using namespace velodyne_configuration;

VLP16_Settings::VLP16_Settings() :
    vlp16_webserver_services::Velodyne_WebServer_Settings()
{
    //----------------------------------------------------
    // Dynamic Parameter server
    //----------------------------------------------------
    // urls:
    // - http://answers.ros.org/question/57498/notify-changes-to-dynamic_reconfigure/
    // - http://www.boost.org/doc/libs/1_61_0/libs/thread/example/recursive_mutex.cpp
    dyn_reconf_server_ = new dynamic_reconfigure::Server<VLP16_settingsConfig>(dynamic_reconfigure_mutex_);
    dyn_reconf_server_->setCallback(boost::bind(&VLP16_Settings::dyn_reconf_server_cb, this, _1, _2));
    dyn_reconf_server_->getConfigDefault(config_);
    //
    ROS_INFO("Ready to set velodyne settings.\t[DYNAMIC_RECONF]");
    //----------------------------------------------------

    //----------------------------------------------------
    // Subscriber
    //----------------------------------------------------
    velodyne_settings_sub_ = nh_.subscribe(get_topic_name_pub(), 10,
                                           &VLP16_Settings::velodyne_settings_cb,
                                           (VLP16_Settings *) this
                                           );
    ROS_INFO("Subscribe to velodyne settings messages.\t[SUBSCRIBER]");
    //----------------------------------------------------
}

void VLP16_Settings::dyn_reconf_server_cb(VLP16_settingsConfig &config, uint32_t level)
{
//    const int return_set_configs = /**webserver_.**/send_settings_to_webserver(config);
    const int return_set_configs = send_settings_to_webserver(config);
    ROS_INFO("Reconfigure Request: %d", return_set_configs);
}

void VLP16_Settings::velodyne_settings_cb(VLP16_SettingsMessage _msg)
{
    ROS_INFO("velodyne_settings_cb");
    synch_Laser_ROS(_msg);
}

bool VLP16_Settings::synch_Laser_ROS(const VLP16_SettingsMessage& _msg)
{
    bool ret = true;

    // On verifie qu'il y a eu un changement entre l'état du laser reporté dans le message
    // et l'état du laser stocké localement dans la config.
    if( state_has_changed(_msg) ) {
        ROS_INFO("synch_Laser_ROS: maj [debut]");   // pour le debug des mutex

        //------------------------------------------
        // MAJ de la config (local) du laser
        //------------------------------------------
        update_config(_msg);

        //------------------------------------------
        // MAJ du Dynamic Parameter Server
        //------------------------------------------
        dyn_reconf_server_->updateConfig(config_);
        //------------------------------------------

        ROS_INFO("synch_Laser_ROS: maj [fin]");     // pour le debug des mutex
    }

    return ret;
}

bool VLP16_Settings::synch_Laser_ROS()
{
    VLP16_SettingsServiceResponse laser_settings;
    // On effectue une requete au webserver pour récupérer l'état du laser
    if( get_response(laser_settings) ) {
        // Si on a récupéré l'état,
        // on met à jour (si besoin) l'enregistrement local (ROS) de l'état du laser
        return synch_Laser_ROS(laser_settings.msg);
    }
    else
    {
        ROS_ERROR_STREAM("Can't get settings from VLP-16 webserver !");
        return false;
    }
}
