#ifndef YAMLSIMPLELOAD_H
#define YAMLSIMPLELOAD_H
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

class LOAD_YAML{
  private:
    bool __debug;
  public:
    YAML::Node yamlNode;
    LOAD_YAML(const std::string &yamlFile);
    template<typename T> void load(const std::string& key, T& val);
    void loadStringList(const std::string& key, std::vector<std::string>& vec);
};
bool __test();


#endif