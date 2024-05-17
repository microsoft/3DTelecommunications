#include "FusionConfig.h"
#include <iostream>

using namespace std;

string FusionConfig::s_config_path;
bool FusionConfig::s_config_loaded = false;
unique_ptr<CConfig> FusionConfig::s_config = nullptr;

const std::string& FusionConfig::get_config_path() { return s_config_path; }

bool FusionConfig::initialize_config(const string& path)
{
	s_config_path = path;
	s_config.reset(new CConfig());
	s_config_loaded = s_config->Load(s_config_path);

	if (!s_config_loaded) {
		cerr << "config " << s_config_path << " failed to load" << endl;
		return false;
	}

	return s_config_loaded;
}

bool FusionConfig::reload(const std::string& new_path)
{
	auto newptr = make_unique<CConfig>();
	auto loaded = s_config->Load(new_path);

	if (loaded)
	{
		s_config_path = new_path;
		swap(s_config, newptr);
		s_config_loaded = loaded;// s_config->Load(s_config_path);
		return true;
	}

	cerr << "config " << s_config_path << " failed to load" << endl;
	return false;
}

CConfig* FusionConfig::current()
{
	if (!s_config_loaded) {
		std::cerr << "config not loaded, returning null" << std::endl;
		return nullptr;
	}
	return s_config_loaded ? s_config.get() : nullptr;
}



