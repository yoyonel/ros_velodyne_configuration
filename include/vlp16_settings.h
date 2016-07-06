#ifndef VELODYNE_SETTINGS_H
#define VELODYNE_SETTINGS_H

#include <dynamic_reconfigure/server.h>
//
#include <velodyne_configuration/VLP16_settingsConfig.h>
#include <velodyne_configuration/VLP16_SettingsService.h>
//
#include <vlp16_webserver_services.h>
//
#include <boost/thread.hpp>


/**
 * @brief The CVelodyneSettings class
 */
class VLP16_Settings :
        public vlp16_webserver_services::Velodyne_WebServer_Settings
{
public:
    VLP16_Settings();

protected:
    /**
     * @brief dyn_reconf_server_cb
     * @param config
     * @param level
     */
    void dyn_reconf_server_cb(
            velodyne_configuration::VLP16_settingsConfig &config,
            uint32_t level
            );
    /**
     * @brief velodyne_settings_cb
     * @param _msg
     */
    void velodyne_settings_cb(
            velodyne_configuration::VLP16_SettingsMessage _msg
            );

    /**
     * @brief synch_Laser_ROS
     * @return
     */
    bool synch_Laser_ROS();
    /**
     * @brief synch_Laser_ROS
     * @param _msg
     * @return
     */
    bool synch_Laser_ROS(
            const velodyne_configuration::VLP16_SettingsMessage& _msg
            );

    //--------------------
    // INLINES
    //--------------------
    /**
     * @brief update_config
     * @param _msg
     */
    inline void update_config(const velodyne_configuration::VLP16_SettingsMessage& _msg) {
        config_.laser_state = _msg.laser_state;
        config_.rpm = _msg.rpm;
        config_.return_type = atoi(_msg.returns.c_str());
    }

    /**
     * @brief state_has_changed
     * @param _msg
     * @return
     */
    inline bool state_has_changed(const velodyne_configuration::VLP16_SettingsMessage& _msg) const {
        return
                config_.laser_state != _msg.laser_state ||
                config_.rpm != _msg.rpm ||
                config_.return_type != atoi(_msg.returns.c_str());
    }
    //--------------------

private:
    // url: http://answers.ros.org/question/10709/assertion-pthread_mutex_lockm-failed-runtime-error-while-working-with-custom-message-and-kinect/
    // => l'ordre de déclaration entre le mutex et le dynamic_reconfigure_server est important !
    // url: http://www.boost.org/doc/libs/1_58_0/doc/html/thread/synchronization.html#thread.synchronization.mutex_types.recursive_mutex
    /**
     * @brief dynamic_reconfigure_mutex_
     */
    boost::recursive_mutex dynamic_reconfigure_mutex_;
    /**
     * @brief dyn_reconf_server_
     */
    dynamic_reconfigure::Server<velodyne_configuration::VLP16_settingsConfig> *dyn_reconf_server_;

    /**
     * @brief config_
     */
    velodyne_configuration::VLP16_settingsConfig config_;

    /**
     * @brief laser_settings_
     */
    velodyne_configuration::VLP16_SettingsServiceResponse laser_settings_;

    /**
     * @brief velodyne_settings_sub_
     */
    ros::Subscriber velodyne_settings_sub_;
};

#endif // VELODYNE_SETTINGS_H
