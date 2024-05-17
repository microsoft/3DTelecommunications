#include "stdafx.h"
/************************************************************
 * iniFileParser.cpp
 * Write by Mingsong Dou, Dec., 2010
 * doums@cs.unc.edu
 * 
 ***********************************************************/
#include "iniFileParser.h"
#include "Utility.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
using namespace std;

iniFile::iniFile(const WCHAR* filename)
{
	this->filename = filename;
	this->own_name_data = false;
}

iniFile::iniFile(const char* filename)
{
	this->filename = convert2WCHAR(filename);
	this->own_name_data = true;
}

void iniFile::setFileName(const WCHAR* filename)
{
	this->filename = filename;
}

char* iniFile::getAString(const WCHAR *section, const WCHAR *key)
{
	int num = GetPrivateProfileString(section, key, L"", this->buf, 1000, this->filename );
	char* ret = new char[num+1];
	for(int i=0; i<num; i++)
	{
		ret[i] = this->buf[i];
	}
	ret[num] = '\0';

	return ret;
}

double* iniFile::getDoubleList(const WCHAR *section, const WCHAR *key, int &len)
{
	char* str = this->getAString(section, key);
	if(str == NULL)
	{
		len = 0;
		return NULL;
	}

	vector<double> double_list;
	char tbuf[1000];
	memset(tbuf, 0, 1000);
	bool empty = true;
	int j=0;
	int i=0;
	while(str[i] != '\0')
	{
		if( str[i] == ' '||
			str[i] == '\t' ||
			str[i] == '\n'
		   )
		{
			if(!empty)
			{
				tbuf[j] = '\0';
				double tmp = atof(tbuf);
				double_list.push_back(tmp);
				empty = true;
				j=0;
			}
			i++;
			continue;
		}

		tbuf[j] = str[i];
		empty = false;
		j++;
		i++;
	}
	if( !empty )
	{
		tbuf[j] = '\0';
		double tmp = atof(tbuf);
		double_list.push_back(tmp);
		empty = true;
		j=0;
	}

	len = double_list.size();
	double* ret = new double[len];
	for(int i=0; i<len; i++)
	{
		ret[i] = double_list[i];
	}
	
	delete [] str;
	return ret;
}

int* iniFile::getIntList(const WCHAR *section, const WCHAR *key, int &len)
{
	char* str = this->getAString(section, key);
	if(str == NULL)
	{
		len = 0;
		return NULL;
	}

	vector<int> int_list;
	char tbuf[1000];
	memset(tbuf, 0, 1000);
	bool empty = true;
	int j=0;
	int i=0;
	while(str[i] != '\0')
	{
		if( str[i] == ' '||
			str[i] == '\t' ||
			str[i] == '\n'
		   )
		{
			if(!empty)
			{
				tbuf[j] = '\0';
				int tmp = atoi(tbuf);
				int_list.push_back(tmp);
				empty = true;
				j=0;
			}
			i++;
			continue;
		}

		tbuf[j] = str[i];
		empty = false;
		j++;
		i++;
	}
	if(!empty)
	{
		tbuf[j] = '\0';
		int tmp = atoi(tbuf);
		int_list.push_back(tmp);
		empty = true;
		j=0;
	}

	len = int_list.size();
	int* ret = new int[len];
	for(int i=0; i<len; i++)
	{
		ret[i] = int_list[i];
	}
	
	delete [] str;
	return ret;
}

bool iniFile::writeAString(const WCHAR *section, const WCHAR *key, const WCHAR *str)
{
	if( section == NULL)
	{
		LOGGER()->warning("iniFile::writeAString", "Section should not be null");
		return false;
	}
	if( key == NULL)
	{
		LOGGER()->warning("iniFile::writeAString", "Since Key is null, Section(%S) in \"%S\" is deleted!", section, this->filename);
	}
	if( str == NULL )
	{
		LOGGER()->warning("iniFile::writeAString", "Since Key is null, Key(%S) in Section(%S) in \"%S\" is deleted!", key, section, this->filename);
	}
	int ret = WritePrivateProfileString(section, key, str, this->filename);
	if( ret !=0 )
		return true;
	else
	{
		LOGGER()->error("iniFile::writeAString", "Writing Section(%S) Key(%S) to \"%S\" failed", section, key, this->filename);
		return false;
	}
}

bool iniFile::writeAString(const WCHAR *section, const WCHAR *key, const char *str)
{
	int num=0;
	WCHAR *buf=NULL;

	if( str != NULL)
	{
		while( str[num]!='\0' )
		{
			num++;
		}
		buf = new WCHAR[num+1];
		for(int i=0; i<num; i++)
		{
			buf[i] = str[i];
		}
		buf[num] = '\0';
	}

	bool ret = this->writeAString(section, key, buf);
	if( buf ) 
		delete [] buf;
	
	return ret;
}

bool iniFile::writeDouble(const WCHAR *section, const WCHAR *key, double val)
{
	char str[100];
	sprintf(str, "%g", val);
	return this->writeAString(section, key, str);
}

bool iniFile::writeInt(const WCHAR *section, const WCHAR *key, int val)
{
	char str[100];
	sprintf(str, "%d", val);
	return this->writeAString(section, key, str);
}

bool iniFile::writeDoubleList(const WCHAR *section, const WCHAR *key, double *arr, int len)
{
	char* str = NULL;
	if( arr!= NULL && len > 0)
	{
		str = new char[len*100];
		str[0] = '\0';
		for( int i=0; i<len; i++)
		{
			sprintf(str, "%s %.15g", str, arr[i]);
		}
	}
	bool ret = this->writeAString(section, key, str);
	if( str )
	delete [] str;

	return ret;
}

bool iniFile::writeIntList(const WCHAR *section, const WCHAR *key, int *arr, int len)
{
	char *str = NULL;
	if( arr!= NULL && len > 0)
	{
		str = new char[len*100];
		str[0] = '\0';
		for( int i=0; i<len; i++)
		{
			sprintf(str, "%s %d", str, arr[i]);
		}
	}
	bool ret = this->writeAString(section, key, str);
	if( str )
		delete [] str;

	return ret;
}