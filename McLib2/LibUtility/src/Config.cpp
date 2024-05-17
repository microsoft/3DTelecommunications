// MSR CCS Copyright 2011
//----------------------------------------------

#include "Config.h"
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;

CConfig::CConfig()
{
	m_bErrorIfNameNotFound = true;
}

CConfig::CConfig(const string& filename)
{
	Load(filename);
}

static string Trim(string str)
{
	int first = 0;
	while (isspace(str[first])) first++;
	// Get rid of quotes around strings with spaces as well
	while (str[first] == '\"') first++;
	int last = (int)str.length() - 1;
	while (isspace(str[last])) last--;
	// Get rid of quotes as well
	while (str[last] == '\"') last--;
	return str.substr(first, last - first + 1);
}

bool CConfig::Load(const string& filename)
{
	m_mEntries.clear();
	// Open file.
	ifstream in(filename.c_str());
	if (!in.is_open())
	{
		cout << "Failed to open " << filename.c_str() << endl;
		return false;
	}

	// Process every line.
	char linebuf[1024];
	// This is our "default" or "undefined" section, old systems will use this, but modern users
	// of CConfig should utilize sections
	string sectionName = ""; 
	m_mEntries[""].clear();

	while (in.getline(linebuf, sizeof(linebuf))) {
		// '#' denotes a comment until the end of the line.
		if (strchr(linebuf, '#') != NULL)
			* strchr(linebuf, '#') = '\0';
		if (strchr(linebuf, ';') != NULL)
			* strchr(linebuf, ';') = '\0';

		// [name] denotes a new section
		if (strchr(linebuf, '[') != NULL && strchr(linebuf, ']') != NULL)
		{
			*strchr(linebuf, ']') = '\0';
			char* name = &linebuf[1];
			sectionName = Trim(name);
			m_mEntries[sectionName].clear();
		}

		string line = linebuf;
		// Get name and value, separated by "=" symbol.
		size_t eq = line.find('=');
		if (eq != string::npos) {
			string name = line.substr(0, eq);
			string value = line.substr(eq + 1, string::npos);
			m_mEntries[sectionName][Trim(name)] = Trim(value);
		}
	}

	return true;
}

vector<string> CConfig::GetKeyList() const
{
	return GetKeyList("");
}

vector<string> CConfig::GetKeyList(const string& section) const
{
	vector<string> out; 
	auto sect = m_mEntries.find(section);
	if (sect != m_mEntries.end())
	{
		for (auto it2 : sect->second)
		{
			out.push_back(it2.first);
		}
	}
	return out;
}

bool CConfig::HasKey(const string& section, const string& name) const 
{
	auto it = m_mEntries.find(section);
	if ( it != m_mEntries.end())
	{
		return (it->second.find(name) != it->second.end());
	}
	return false;
}


int CConfig::ParseValue(const std::string& value, int typeHint) const
{
	try
	{
		return std::stoi(value);
	}
	catch (const std::invalid_argument& ia)
	{
		std::cerr << "ParseValue<int> Exception caught when trying to parse [" << value << "] as an int" << std::endl;
		return -1;
	}
}
double CConfig::ParseValue(const std::string& value, double typeHint) const
{
	return std::stod(value);
}
float CConfig::ParseValue(const std::string& value, float typeHint) const
{
	return std::stof(value);
}
std::string CConfig::ParseValue(const std::string& value, std::string& typeHint) const
{
	return value;
}
std::wstring CConfig::ParseValue(const std::string& value, std::wstring& typeHint) const
{
	return std::wstring(value.begin(), value.end());
}

const char* CConfig::ParseValue( const std::string& value, const char* typeHint ) const
{
	return value.c_str( );
}


bool CConfig::ParseValue(const std::string& value, bool typeHint) const
{
	if (value == "t"
		|| value == "T"
		|| value == "Y"
		|| value == "y"
		|| value == "true"
		|| value == "yes"
		|| value == "TRUE"
		|| value == "YES") return true;
	try
	{
		return (std::stoi(value) != 0);
	}
	catch (const std::invalid_argument& ia)
	{
		return false;
	}
}

bool CConfig::TryParseValue(const std::string& value, int typeHint) const
{
	try
	{
		auto parsed_val = std::stoi(value);
		parsed_val = parsed_val * 0;	// We don't really need to use the value of parsed_val, just check for exceptions, but I need to use parsed_val to avoid compilation fails when warnings are
										// set to fail compilation
		return true;
	}
	catch (const std::invalid_argument& ia)
	{
		std::cerr << "TryParseValue<int> Exception caught when trying to parse [" << value << "] as an int" << std::endl;
		return false;
	}
}

bool CConfig::TryParseValue(const std::string& value, float typeHint) const
{
	try
	{
		auto parsed_val = std::stof(value);
		parsed_val = parsed_val * 0;	// We don't really need to use the value of parsed_val, just check for exceptions, but I need to use parsed_val to avoid compilation fails when warnings are
										// set to fail compilation
		return true;
	}
	catch (const std::invalid_argument& ia)
	{
		std::cerr << "TryParseValue<float> Exception caught when trying to parse [" << value << "] as an float" << std::endl;
		return false;
	}
}

bool CConfig::TryParseValue(const std::string& value, double typeHint) const
{
	try
	{
		auto parsed_val = std::stod(value);
		parsed_val = parsed_val * 0;	// We don't really need to use the value of parsed_val, just check for exceptions, but I need to use parsed_val to avoid compilation fails when warnings are
										// set to fail compilation
		return true;
	}
	catch (const std::invalid_argument& ia)
	{
		std::cerr << "TryParseValue<double> Exception caught when trying to parse [" << value << "] as an double" << std::endl;
		return false;
	}
}