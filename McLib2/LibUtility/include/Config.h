#pragma once
#ifndef _MSC_VER
#include <string.h>
#include <iostream>
#else
#include <string>
#include <iostream>
#endif
#include <vector>
#include <sstream>
#include <map>

class CConfig
{
public:
	CConfig();
	CConfig(const std::string& filename);
	bool Load(const std::string& filename);

	void SetErrorIfNameNotFound(bool error) { m_bErrorIfNameNotFound = error; }
	
public:
	std::vector<std::string> GetKeyList() const;
	std::vector<std::string> GetKeyList(const std::string& section) const;
	bool			HasKey(const std::string& section, const std::string& name) const; //presence of key

public:
	template <typename T> 
	T GetValue(const std::string& section, const std::string& name) const;
	template <typename T> 
	T GetValue(const std::string& overrideSection, const std::string& section, const std::string& name) const;
	template <typename T>
	T GetValueWithDefault(const std::string& section, const std::string& name, const T defaultValue) const;
	template <typename T>
	T GetValueWithDefault(const std::string& overrideSection, const std::string& section, const std::string& name, const T defaultValue) const;
	template <typename T>
	std::vector<T> GetValueArray(const std::string& section, const std::string& name) const;

	template <typename T>
	bool TryParseValue(const std::string& section, const std::string& name) const
	{
		T hint{};
		auto sect = m_mEntries.find(section);
		if (sect != m_mEntries.end())
		{
			auto kvp = sect->second.find(name);
			if (kvp != sect->second.end())
			{
				return TryParseValue(kvp->second, hint);
			}
			else if (m_bErrorIfNameNotFound)
			{
				throw std::exception("Value not found in config file!");
			}
		}
		return false;
	}

private:
	bool m_bErrorIfNameNotFound;

protected:
	std::map<std::string, std::map<std::string, std::string>> m_mEntries;
	
	int ParseValue(const std::string& value, int typeHint) const;
	double ParseValue(const std::string& value, double typeHint) const;
	float ParseValue(const std::string& value, float typeHint) const;
	bool ParseValue(const std::string& value, bool typeHint) const;
	std::string ParseValue(const std::string& value, std::string& typeHint) const;
	std::wstring ParseValue(const std::string& value, std::wstring& typeHint) const;
	const char * ParseValue( const std::string& value, const char * typeHint ) const;

	bool TryParseValue(const std::string& value, int typeHint) const;
	bool TryParseValue(const std::string& value, float typeHint) const;
	bool TryParseValue(const std::string& value, double typeHint) const;
};

template<typename T>
T CConfig::GetValue(const std::string& section, const std::string& name) const
{
	T hint{};
	auto sect = m_mEntries.find(section);
	if (sect != m_mEntries.end())
	{
		auto kvp = sect->second.find(name);
		if (kvp != sect->second.end())
		{			
			return ParseValue(kvp->second, hint);
		}
		else if (m_bErrorIfNameNotFound)
		{
			std::cout << "Failed to find '" << name << "' in section '" << section << "'." << std::endl;
		}
	}
	else if (m_bErrorIfNameNotFound)
	{
		std::cout << "Failed to find section '" << section << "' in config." << std::endl;
	}
	return hint;
}

template<typename T>
T CConfig::GetValue(const std::string& overrideSection, const std::string& section, const std::string& name) const
{
	return GetValueWithDefault(overrideSection, name, GetValue<T>(section, name));
}

template <typename T>
T CConfig::GetValueWithDefault(const std::string& section, const std::string& name, const T defaultValue) const
{
	if (HasKey(section, name)) {
		return GetValue<T>(section, name);
	}
	else
	{
		if (m_bErrorIfNameNotFound)
		{
			std::cout << "Failed to find [" << section << "][" << name << "] in config.  Returning default value " << defaultValue << std::endl;
		}
		return defaultValue;
	}
}


template <typename T>
T CConfig::GetValueWithDefault(const std::string& overrideSection, const std::string& section, const std::string& name, const T defaultValue) const
{
	return GetValueWithDefault(overrideSection, name, GetValueWithDefault(section, name, defaultValue));
}

template<typename T>
std::vector<T> CConfig::GetValueArray(const std::string& section, const std::string& name) const
{
	std::vector<T> array;
	T hint{};
	auto sect = m_mEntries.find(section);
	if (sect != m_mEntries.end())
	{
		auto kvp = sect->second.find(name);
		if (kvp != sect->second.end())
		{
			// config arrays should be in the format {val,val,...}
			std::string arrayString = GetValue<std::string>(section, name);
			// remove the ()
			arrayString = arrayString.substr(1, arrayString.length() - 2);
			std::stringstream ss(arrayString);
			std::string token;
			while (std::getline(ss, token, ','))
			{
				array.push_back(ParseValue(token, hint));
			}
			return array;
		}
	}
	return array;
}
