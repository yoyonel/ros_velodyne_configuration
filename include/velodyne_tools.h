#ifndef __VLP_WEBCLIENT_H
#define __VLP_WEBCLIENT_H

#include <string>
#include <velodyne_tools_boost_asio.h>


namespace velodyne_tools
{

#define hdltop_volts_to_hv(volts)   \
    101.0 * (volts - 5.0)

#define lm20_volts_to_degCel(volts) \
    -1481.96 + sqrt(2.1962E6 + (1.8639 - volts)/3.88E-6)

#define acs17_volts_to_amps(volts)  \
    10.0 * (volts - 2.5);


#define getter_cst_ref(type, name)                          \
    inline const type&  get_##name() const { return name; }
#define getter(type, name)      \
    getter_cst_ref(type, name)
#define setter(type, name)                                      \
    inline void set_##name(const type& _##name) { name = _##name; }

#define defaults_getter_setter(type, name)  \
    getter(type, name)                      \
    setter(type, name)

#define DEFAULT_LOOP_RATE_VALUE     1
#define DEFAULT_MAX_DELAY_FOR_CMD   0
#define DEFAULT_NETWORK_SENSOR_IP   "192.168.1.201"

//---------------------------------------------------------------------------
// url: http://www.alexonlinux.com/gcc-macro-language-extensions
#define JSON_INIT(_root, _input)                                \
    std::stringstream _input ## _stream;                        \
    _input ## _stream << _input;                                \
    pt::ptree _root;                                            \
    boost::property_tree::read_json(_input ## _stream, _root);

#define JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, _type)   \
    _ros_res.msg._ros_msg_child = _json_root.get < _type > ( #_json_child )

#define JSON_READ_STRING(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, std::string)

#define JSON_READ_UINT16(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, uint16_t)

#define JSON_READ_UINT8(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, uint8_t)

#define JSON_READ_BOOL(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ_STRING(_json_root, _json_child, _ros_res, _ros_msg_child) == "On"

#define JSON_READ_STATE(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ_STRING(_json_root, _json_child, _ros_res, _ros_msg_child) == "Enabled"

#define JSON_READ_STRING(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, std::string)
//---------------------------------------------------------------------------

std::string exec_cmd(const char* cmd);

} // namespace velodyne_tools


//#include <functional>

//template <
//        typename OwnerType_,
//        typename ValueType_,
//        ValueType_ (OwnerType_::*getter_)( ),
//        void (OwnerType_::*setter_)( ValueType_ )
//     >
//struct Property
//{
//    Property( OwnerType_* owner )
//        : _owner( owner ) { }

//    const ValueType_& operator=( const ValueType_& value )
//    {
//        ( _owner->*setter_ )( value );
//        return value;
//    }

//    ValueType_&& operator=( ValueType_&& value )
//    {
//        ( _owner->*setter_ )( std::forward< ValueType_ >( value ) );
//        return std::move( ( _owner->*getter_ )( ) );
//    }

//    operator ValueType_( )
//    {
//        (_owner->*getter_)( );
//    }

//    Property( Property&& ) = default;
//    Property( const Property& ) = delete;
//    Property& operator=( const Property& ) = delete;
//    Property& operator=( Property&& ) = delete;

//private:
//    OwnerType_* const _owner;
//};

//#define CREATE_PROPERTY(className, accessSpec, valueType, propName, getBlock, setBlock) \
//    private: valueType _auto_g_get##propName() getBlock \
//    private: void _auto_g_set##propName(valueType value) setBlock \
//    public: typedef Property<className, valueType, \
//    &className::_auto_g_get##propName, \
//    &className::_auto_g_set##propName> _auto_g_propType##propName; \
//    accessSpec: _auto_g_propType##propName propName = _auto_g_propType##propName(this)

#endif
