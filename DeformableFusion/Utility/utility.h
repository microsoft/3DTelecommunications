//===============================================
//			utility.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef	 __UTILITY_H__
#define __UTILITY_H__
#include <windows.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstdarg>
#include <string>

#include "Logger.h"

using namespace std;

int fprintf2(WORD attrib, FILE* file, const char* format, ...);
int printf2(WORD attrib, const char* format, ...);

//return the index if val is found in val_list
// return -1 otherwise
template<class T>
int search_val_list(std::vector<T> const& val_list, T const&val)
{
	for(int i=0; i<val_list.size(); i++)
	{
		if( val_list[i] == val )
			return i;
	}

	return -1;
}


template<class T>
bool search_lists_overlap(std::vector<T> const&val_list1, std::vector<T> const&val_list2, std::vector<T> &list_overlap)
{
	list_overlap.clear();
	for(int i=0; i<val_list1.size(); i++)
	{
		T val = val_list1[i];
		if( search_val_list(val_list2, val) >= 0)
		{
			list_overlap.push_back(val);
		}
	}

	return list_overlap.size()>0;
}

WCHAR* convert2WCHAR(const char* str);

int  chgFN(char word[], char special, char fill, char add[]);
int  chgFN(char word[], char special, int num);

/* each line in the file should be a file name
 * Usage: char** name_list = NULL;
 *        int num_of_names = 0;
 *		  readNameFile("../xxx.txt", num_of_names, name_list); 
 */
bool readNameFile(const char* name, int &num, char** &file_name_array);

/* Pure file name is the string after getting rid of extension and directory path
 */
char* getPureFileName(const char* filename);
/* the numbers of the file name is replaced by '*'
 */
char* getBaseFileName(const char* filename);

/* assuming file index locates at the end of the name string
 * return -1 if no index found
 */
int getFileIndex(const char* pure_file_name);
bool isADigit(char c);

bool isStrAllWhite(const char* str);

;void __tic__();
double __toc__();

#endif