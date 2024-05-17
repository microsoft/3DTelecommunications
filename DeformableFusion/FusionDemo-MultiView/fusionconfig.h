#pragma once


#include MCLIB2_CONFIG_INC //this is defined in LibUtility property sheet
#include <memory>

struct FusionConfig
{
public:
	static bool initialize_config(const std::string& path);
	static bool reload(const std::string& new_path);
	static CConfig* current();
	static const std::string& get_config_path();
private:
	static std::string s_config_path;
	static bool s_config_loaded;
	static std::unique_ptr<CConfig> s_config;
};
