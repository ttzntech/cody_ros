#pragma once
#include <yaml-cpp/yaml.h>
#include <iostream>


/* #define CONFIG_SET(filename)                                                   \ */
/* private:                                                                       \ */
/*   class ConfigLoader                                                           \ */
/*   {                                                                            \ */
/*   private:                                                                     \ */
/*     YAML::Node node_ = YAML::LoadFile((std::string)WORKSPACE_PATH +            \ */
/*                                       "/config/" + #filename + ".yaml");       \ */
/*                                                                                \ */
/*   public: */

/* #define CONFIG_END                                                             \ */
/*   }                                                                            \ */
/*   ;                                                                            \ */
/*   std::unique_ptr<ConfigLoader> config_; */

namespace cody_driver {

class ConfigLoader
{
    public:
    ConfigLoader(const std::string& filename)
    {
        node_ = YAML::LoadFile((std::string)WORKSPACE_PATH + "/config/" + filename +
                            ".yaml");
        std::cerr << "load config: " << (std::string)WORKSPACE_PATH + "/config/" + filename +
                            ".yaml" << std::endl;
    }

    YAML::Node node_;
};
}
