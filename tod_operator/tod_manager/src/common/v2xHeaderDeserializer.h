#include <string>
#include <vector>
#include <map>
#include "tod_core/YamlLoader.h"

namespace v2xHeaderDeserializer {

std::vector<std::string> vectorLoad(const std::string& filePath, const std::string& key)  {
    YamlLoader loader;
    loader.load_from_path(filePath);
    if ( !loader.has_node(key) ) {
        printf("No Sequence provided under key %s in %s\n",
        key.c_str(), filePath.c_str());
        return {};
    }
    else{
        return loader.get_param<std::vector<std::string>>(key);
    }
}

std::map<int, std::string> mapLoad(const std::string& filePath, const std::string& key)  {
    YamlLoader loader;
    loader.load_from_path(filePath);
    if ( !loader.has_node(key) ) {
        printf("No Mapping provided under key %s in %s\n",
        key.c_str(), filePath.c_str());
        return {};
    }
    else{
        return loader.get_param<std::map<int, std::string>>(key);
    }
}
}
