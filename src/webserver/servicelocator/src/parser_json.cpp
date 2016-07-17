#include <map>
#include <string>   // for: std::string
#include <sstream>  // for: std::stringstream
#include <stdint.h> // for uint16_t
#include <iostream> // for: cerr
#include <boost/property_tree/ptree.hpp>    // for:boost:: property_tree::ptree
#include <boost/property_tree/json_parser.hpp>  // for: boost::property_tree::read_json
#include <boost/variant.hpp>    // for: boost::variant
#include <functional>   // for: std::function
#include <tuple>    // for: std::tuple

// url: https://github.com/nlohmann/json
#define private public
#include "../third/json/src/json.hpp"
using nlohmann::json;
//
//using namespace boost::property_tree;


class AnyBase
{
public:
    virtual ~AnyBase() = 0;
};
inline AnyBase::~AnyBase() {}

template<class T>
class Any : public AnyBase
{
public:
    typedef T Type;
    explicit Any(const Type& data) : data(data) {}
    Any() {}
    Type data;
};

template<int id_type>
class AnyIdType : public AnyBase
{
public:
    AnyIdType() {}
    static const uint8_t id_type_ = id_type;
};

template<>
class AnyIdType<0> : public AnyBase
{
public:
    typedef uint16_t Type;
    explicit AnyIdType(const Type& data) : data(data) {}
    AnyIdType() {}
    Type data;
};

template<>
class AnyIdType<1> : public AnyBase
{
public:
    typedef uint8_t Type;
    explicit AnyIdType(const Type& data) : data(data) {}
    AnyIdType() {}
    Type data;
};

template<>
class AnyIdType<2> : public AnyBase
{
public:
    typedef std::string Type;
    explicit AnyIdType(const Type& data) : data(data) {}
    AnyIdType() {}
    Type data;
};

template<>
class AnyIdType<3> : public AnyBase
{
public:
    typedef bool Type;
    explicit AnyIdType(const Type& data) : data(data) {}
    AnyIdType() {}
    Type data;
};

template<>
class AnyIdType<4> : public AnyBase
{
public:
    typedef bool Type;
    explicit AnyIdType(const Type& data) : data(data) {}
    AnyIdType() {}
    Type data;
};

template<typename T, int id> struct JSON_TYPE_META    { typedef T    type; static const uint8_t id_type = id; };
struct JSON_UINT16  : public JSON_TYPE_META<uint16_t, 0>    { };
struct JSON_UINT8   : public JSON_TYPE_META<uint8_t, 1>     { };
struct JSON_STRING  : public JSON_TYPE_META<std::string, 2> { };
struct JSON_BOOL    : public JSON_TYPE_META<bool, 3>        { };
struct JSON_STATE   : public JSON_TYPE_META<bool, 4>        { };

class ParserJSON {
public:
    ParserJSON(const std::string &_string_input);
    ParserJSON(const std::stringstream &_ss_input);

    // urls:
    // - http://stackoverflow.com/questions/15911890/overriding-return-type-in-function-template-specialization
    // - http://stackoverflow.com/a/15912228
    // - https://ideone.com/NSGksm
    template<typename T>
    struct item_return{ typedef typename T::type type; };

    template<typename T>
    typename item_return<T>::type read(const std::string &path_)
    {
        return read_imp<typename item_return<T>::type>(path_);
    }

    template<typename T>
    void read_and_save(const std::string &path_)
    {
        using type_return = typename item_return<T>::type;
        const type_return & value = read<T>(path_);
        //
        map_childs_values[path_].reset(new Any<type_return>(value));
        //
        const uint8_t idtype_return = T::id_type;
        std::get<0>(map_childs_values2[path_]).reset(new AnyIdType<idtype_return>(value));
        std::get<1>(map_childs_values2[path_]) = idtype_return;
        //
        map_results_with_any_[path_] = Any<type_return>(value);
//        save(path_, value);
    }

//    template<typename T>
//    void save(const std::string &path_, const T & value_);

    template<typename T>
    void add_child(const std::string & _name)
    {
        map_functions_[_name] = std::bind( & ParserJSON::read_and_save<T>, this, _name);
        map_types_[_name] = typeid(T).name();

        using type_return = typename item_return<T>::type;
        map_results_with_any_[_name] = Any<type_return>();
        const uint8_t idtype_return = T::id_type;
        map_types_results_[_name] = idtype_return;
    }

    template<typename T>
    T get_child_value(const std::string & _name)
    {
//        map_functions_.at(_name)();
//        return static_cast<Any<T>&>(*map_childs_values[_name]).data;
        return boost::get<Any<T>>(map_results_with_any_[_name]).data;
    }

    template<typename T>
    void get_child_value(const std::string & _name, T & _value)
    {
//        map_functions_.at(_name)();
        _value = boost::get<Any<T>>(map_results_with_any_[_name]).data;
    }

    void update()
    {
        for (const auto& kv : map_results_with_any_) {
            const std::string & name = kv.first;
            map_functions_.at(name)();
        }
    }


//    void get_child_value2(const std::string & _name, bool * _value)
//    {
//        map_functions_.at(_name)();
//        switch(std::get<1>(map_childs_values2[_name])) {
//        case 0:
//        {
//            uint16_t * ptrValue = reinterpret_cast<uint16_t*>(_value);
//            *ptrValue = static_cast<Any<uint16_t>&>(*map_childs_values[_name]).data;
//        }
//            break;
//        case 1:
//        {
//            uint8_t * ptrValue = reinterpret_cast<uint8_t*>(_value);
//            *ptrValue = static_cast<Any<uint8_t>&>(*map_childs_values[_name]).data;
//        }
//            break;
//        case 2:
//        {
//            std::string * ptrValue = reinterpret_cast<std::string*>(_value);
//            *ptrValue = static_cast<Any<std::string>&>(*map_childs_values[_name]).data;
//        }
//            break;
//        case 3:
//        {
//            bool * ptrValue = reinterpret_cast<bool*>(_value);
//            *ptrValue = static_cast<Any<bool>&>(*map_childs_values[_name]).data;
//        }
//            break;
//        case 4:
//        {
//            bool * ptrValue = reinterpret_cast<bool*>(_value);
//            *ptrValue = static_cast<Any<bool>&>(*map_childs_values[_name]).data;
//        }
//        }

//        //        return static_cast<Any<T>&>(*map_childs_values[_name]).data;
//    }


    void cout_map()
    {
        //        for (const auto& kv : map_functions_) {
        //            //
        //            auto func_update = kv.second;
        //            func_update();
        ////            map_childs_values[child_name];
        //            //
        //            const std::string & child_name = kv.first;

        ////            std::cout << child_name << ": " << (*).data << std::endl;
        //        }
    }

protected:
    template<typename T>
    T read_imp(const std::string &path_);

    void init();

private:
    std::stringstream ss_input_;
    boost::property_tree::ptree pt_;

    std::string path_cur_;

    std::map<std::string, std::function<void ()>> map_functions_;
    std::map<std::string, std::string> map_types_;

    std::map<std::string, std::unique_ptr<AnyBase>> map_childs_values;
    std::map<std::string, std::pair<std::unique_ptr<AnyBase>, uint8_t>> map_childs_values2;

    typedef boost::variant< Any<uint8_t>, Any<uint16_t>, Any<std::string>, Any<bool> > tVariantAny;
    std::map<std::string, tVariantAny> map_results_with_any_;
    std::map<std::string, uint8_t> map_types_results_;
    //
    std::map<std::string, uint8_t> map_results_uint8_t_;
    std::map<std::string, uint16_t> map_results_uint16_t_;
    std::map<std::string, bool> map_results_bool_;
    std::map<std::string, std::string> map_results_string_;
};

void ParserJSON::init()
{
    try {
        boost::property_tree::read_json(ss_input_, pt_);
    }
    catch (std::exception const& e) {
        std::cerr << e.what() << std::endl;
    }
    //
    path_cur_ = "";
}

ParserJSON::ParserJSON(const std::string &_string_input)
{
    ss_input_ << _string_input;
    init();
}

ParserJSON::ParserJSON(const std::stringstream &_ss_input)
{
    ss_input_ << _ss_input;
    init();
}

template<typename T>
T ParserJSON::read_imp(const std::string &path_)
{
    try {
        return pt_.get<T>(path_);
    }
    catch (std::exception const& e) {
        std::cerr << e.what() << std::endl;
        return T();
    }
}

template<>
bool ParserJSON::read<JSON_BOOL>(const std::string &path_)
{
    return ParserJSON::read_imp<std::string>(path_) == "On";
}

template<>
bool ParserJSON::read<JSON_STATE>(const std::string &path_)
{
    return ParserJSON::read_imp<std::string>(path_) == "Enabled";
}

template<>
bool ParserJSON::get_child_value(const std::string & _name)
{
    map_functions_.at(_name)();
    return static_cast<Any<bool>&>(*map_childs_values[_name]).data;
}

//template<>
//void ParserJSON::save(const std::string &path_, const bool & value_)
//{
//    map_results_bool_[path_] = value_;
//}
//template<>
//void ParserJSON::save(const std::string &path_, const uint8_t & value_)
//{
//    map_results_uint8_t_[path_] = value_;
//}
//template<>
//void ParserJSON::save(const std::string &path_, const uint16_t & value_)
//{
//    map_results_uint16_t_[path_] = value_;
//}
//template<>
//void ParserJSON::save(const std::string &path_, const std::string & value_)
//{
//    map_results_string_[path_] = value_;
//}

//template<>
//void ParserJSON::retrieve_child_value(const std::string & _name, bool & _value)
//{
//    map_functions_.at(_name)();
//    _value = boost::get<bool>(map_results_[_name]);
//}

void test_parser_json()
{
    std::string settings_json = " {\"laser\":\"On\",\"returns\":\"Strongest\",\"rpm\":300,\"fov\":{\"start\":0,\"end\":359},\"phaselock\":{\"enabled\":\"Off\",\"offset\":\"0\"},\
                                \"host\":{\"addr\":\"172.20.0.215\",\"dport\":\"2368\",\"tport\":\"8308\"},\
                                \"net\":{\"addr\":\"192.168.1.201\",\"mask\":\"255.255.255.0\",\"gateway\":\"192.168.1.1\",\"dhcp\":\"Off\"} }";

    ParserJSON parser(settings_json);

    // Manual Parsing: on specifie le type des childs a lire
    std::cout << "laser: " << parser.read<JSON_BOOL>("laser") << std::endl;
    std::cout << "returns: " << parser.read<JSON_STRING>("returns") << std::endl;
    std::cout << "fov.start: " << parser.read<JSON_UINT16>("fov.start") << std::endl;
    std::cout << "fov.end: " << parser.read<JSON_UINT16>("fov.end") << std::endl;


    parser.add_child<JSON_BOOL>("laser");
    parser.add_child<JSON_STRING>("returns");
    parser.add_child<JSON_UINT16>(std::string("fov") + "." + "start");
    parser.add_child<JSON_UINT16>(std::string("fov") + "." + "end");

    parser.update();

    std::cout << "laser: " << parser.get_child_value<bool>("laser") << std::endl;
//    std::cout << "laser: " << parser.get_child_value<int>("laser") << std::endl;
    std::cout << "returns: " << parser.get_child_value<std::string>("returns") << std::endl;
    std::cout << "fov.start: " << parser.get_child_value<uint16_t>("fov.start") << std::endl;
    std::cout << "fov.end: " << parser.get_child_value<uint16_t>("fov.end") << std::endl;

    //
    bool b_laser;
    std::string s_returns;
    uint16_t ui_fov_start, ui_fov_end;
    parser.get_child_value("laser", b_laser);
    parser.get_child_value("returns", s_returns);
    parser.get_child_value("fov.start", ui_fov_start);
    parser.get_child_value("fov.end", ui_fov_end);
    std::cout << "laser: " << b_laser << std::endl;
    std::cout << "returns: " << s_returns << std::endl;
    std::cout << "fov.start: " << ui_fov_start << std::endl;
    std::cout << "fov.end: " << ui_fov_end << std::endl;
    std::cout << std::endl;

//    auto j2 = " {\"laser\":\"On\",\"returns\":\"Strongest\",\"rpm\":300,\"fov\":{\"start\":0,\"end\":359},\"phaselock\":{\"enabled\":\"Off\",\"offset\":\"0\"},\
//              \"host\":{\"addr\":\"172.20.0.215\",\"dport\":\"2368\",\"tport\":\"8308\"},\
//              \"net\":{\"addr\":\"192.168.1.201\",\"mask\":\"255.255.255.0\",\"gateway\":\"192.168.1.1\",\"dhcp\":\"Off\"} }"_json;
//    auto j2 = json::parse(reinterpret_cast<const nlohmann::json::string_t::value_type*>(settings_json.c_str()));
    auto j2 = json::parse(settings_json);

    // range-based for
    for (json::iterator it = j2.begin(); it != j2.end(); ++it) {
        const auto & j_value = it.value();
        std::cout << "# " << it.key() << "\n\tValue: " << j_value << "\n\tType: " << j_value.type_name();
        std::cout << std::endl;
    }
    std::cout << j2.dump(4) << std::endl;

    std::cout << "fov.start: " << j2["/fov/start"_json_pointer] << std::endl;
    std::cout << "fov.end: " << j2["/fov/end"_json_pointer] << std::endl;
}
