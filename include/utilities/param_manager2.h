//
// Created by redwan on 11/21/22.
//

#ifndef CREATE3_CONTROLLER_PARAM_MANAGER2_H
#define CREATE3_CONTROLLER_PARAM_MANAGER2_H
#include <yaml-cpp/yaml.h>
#include <memory>
#include <iostream>
#define DEBUG(x) std::cout << x << std::endl

class param_manager2;
typedef std::shared_ptr<param_manager2> ParamPtr2;

class param_manager2: public std::enable_shared_from_this<param_manager2>{
public:
    param_manager2(const std::string& file)
    {
        DEBUG("loading " << file);
        config_ = YAML::LoadFile(file);
    }



    template<class T>
    T get_param(const std::string& field)
    {
        T value = config_[field].template as<T>();
        DEBUG(field << " = " << value);
        return value;
    }

    template<class T>
    T get_param(const std::string& field1, const std::string& field2)
    {
        return config_[field1][field2].template as<T>();
    }

    template<class T>
    T get_param(const std::string& field1, const std::string& field2, const std::string& field3)
    {
        return config_[field1][field2][field3].template as<T>();
    }
private:
    YAML::Node config_;
};

#endif //CREATE3_CONTROLLER_PARAM_MANAGER2_H
