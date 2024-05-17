/************************************************************
 * iniFileParser.h
 * Write by Mingsong Dou, Oct., 2010
 * doums@cs.unc.edu
 * 
 ***********************************************************/

#ifndef __INIFILEPARSE_H__
#define __INIFILEPARSE_H__
#ifndef UNICODE
#define UNICODE
#endif
#include <windows.h>
#include <wchar.h>
#undef UNICODE
class iniFile
{
public:
	iniFile(const WCHAR* filename);
	iniFile(const char* filename);
	~iniFile()
	{
		if( own_name_data)
			delete [] filename;
	}

public:
	void setFileName(const WCHAR* filename);
	char* getAString(const WCHAR* section, const WCHAR* key);
	double* getDoubleList(const WCHAR* section, const WCHAR* key, int& len);
	int* getIntList(const WCHAR* section, const WCHAR* key, int& len);

	/* Note: if there is already a (section, key) pair with the same name, the old data will be
	 *       replaced.
	 */
	bool writeAString(const WCHAR* section, const WCHAR* key, const WCHAR* str);
	bool writeAString(const WCHAR* section, const WCHAR* key, const char* str);
	bool writeDoubleList(const WCHAR* section, const WCHAR* key, double *arr, int len);
	bool writeIntList(const WCHAR* section, const WCHAR* key, int *arr, int len);
	bool writeInt(const WCHAR *section, const WCHAR* key, int val);
	bool writeDouble(const WCHAR *section, const WCHAR* key, double val);

private:
	const WCHAR* filename;
	bool own_name_data;
	WCHAR buf[2000];	

private:
	WCHAR* convert2WCHAR( const char* str)
	{
		int len = strlen(str);
		WCHAR* w_str = new WCHAR[len+1];
		for(int i=0; i<len; i++)
		{
			w_str[i] = str[i];
		}
		w_str[len] = '\0';
		return w_str;
	}
	
};



#endif