#include "stdafx.h"
#include "utility.h"
#include <string.h>
#include "windows.h"
#include <stack>
#include <time.h>

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

int  chgFN(char word[], char special, char fill, char add[])
{
   int a, b, s, t, x, y;

   a = (int) strlen(word);
   b = (int) strlen(add);

   for(x = 0; x < a && word[x] != special; x++)   {}
   t = x; //t is the first special char
   if (x == a)  return 0;   // didn't find character

   s = 0;
   do
   {
      s++;
      x++;
   }  while (word[x] == special); //x-1 is last special char; s is the number of special char

   if( b > s ) // add has more characters than special chars in word
   {
	   //assume the array word always has enough space
	   char *word_bak = new char[a+1];
	   strcpy(word_bak, word);
	   for(int i=0; i<b; i++)
		   word[t+i] = add[i];
	   for(int i=0; i<a-x; i++)
		   word[t+b+i] = word_bak[x+i];
	   word[t+b+a-x] = '\0';
	   delete [] word_bak;
	   return 1;
   }

   if(b > x)    return 0;   // not enough space

   for(y = b - 1; y >= 0; y--)
   {
      x--;
      word[x] = add[y];
   }

   while(x > t)
   {
      x--;
      word[x] = fill;
   }

   return 1;
}

int  chgFN(char word[], char special, int num)
{
   char temp1[500], temp2[500];
   int x,y,t;

   for(y = 0; num > 0 && y < 50; y++)
   {
      temp1[y] = (char) (num % 10 + 48);
      num /= 10;
   }
   temp1[y] = '\0';

   t = 0;
   for(x = y; x >= 0; x--)
   {
      temp2[x] = temp1[x];
      t++;
   }

   chgFN(word, special, '0', _strrev(temp1) );

   return 1;
}

bool isStrAllWhite(const char* str)
{
	for(int i=0; i<strlen(str); i++)
	{
		if( str[i] != ' ' &&
			str[i] != '\n' &&
			str[i] != '\r' &&
			str[i] != '\t' )
			return false;
	}
	return true;
}

bool readNameFile(const char* name, int &num, char** &file_name_array)
{
	FILE *file = fopen(name, "r");
	if( !file )
	{
		printf("Error when Reading name list File '%s' \n", name);
		return false;
	}

	//find out how many points
	int line_num = 0;
	char line[1000];
	while( 1 )
	{
		if (feof(file) )
		{
			rewind(file);
			break;
		}

		char* ret = fgets(line, 1000, file); //read a line

		if(isStrAllWhite(line))
			continue;

		line_num ++;
	}

	if( line_num <= 0)
	{
		num = 0;
		file_name_array = NULL;
		fclose(file);
		return false;
	}

	file_name_array = new char*[line_num];
	num = line_num;

	int idx = 0;
	while( 1 )
	{
		if (feof(file) != 0)
		{
			rewind(file);
			break;
		}

		fgets(line, 1000, file); //read a line

		if (isStrAllWhite(line))
			continue;

		for(int i=strlen(line)-1; i>=0; i--)
		{
			if( line[i] == '\n' || line[i] == ' ' || 
				line[i] == '\r' || line[i] == '\t' )
			{
				line[i] = '\0';
			}
			else
				break;
		}

		file_name_array[idx] = new char[strlen(line)+1];
		strcpy(file_name_array[idx], line);

		idx++;
	}

	fclose(file);

	return true;
}

char* getPureFileName(const char* filename)
{
	if( filename == NULL)
		return NULL;

	int k1=0, k2=0;
	bool separator_found = false;
	for(k1=strlen(filename)-1; k1>0; k1--)
	{
		if( filename[k1] == '/' ||
			filename[k1] == '\\' )
		{
			separator_found = true;
			break;
		}
	}
	if( !separator_found )
		k1 = -1;

	for(k2=strlen(filename)-1; k2>0; k2--)
	{
		if( filename[k2] == '.' )
			break;
	}
	if( k1 >= k2 )
		return NULL;
	char *img_name_only = new char[k2-k1];
	for(int j=k1+1; j<k2; j++)
		img_name_only[j-k1-1] = filename[j];
	img_name_only[k2-k1-1] = '\0';

	return img_name_only;
}

char* getBaseFileName(const char* filename)
{
	if( filename == NULL)
		return NULL;

	char *base_file_name = new char[strlen(filename)+1];
	strcpy(base_file_name, filename);

	int k1=0, k2=0;
	bool separator_found = false;
	for(k1=strlen(filename)-1; k1>0; k1--)
	{
		if( filename[k1] == '/' ||
			filename[k1] == '\\' )
		{
			separator_found = true;
			break;
		}
	}
	if( !separator_found )
		k1 = -1;

	for(k2=strlen(filename)-1; k2>0; k2--)
	{
		if( filename[k2] == '.' )
			break;
	}
	if( k1 >= k2 )
		return NULL;
	for(int j=k1+1; j<k2; j++)
	{
		if( isADigit( base_file_name[j] ) )
			base_file_name[j] = '*';
	}
	return base_file_name;
}

bool isADigit(char c)
{
	if( c >= '0' &&
		c <= '9' )
		return true;
	else
		return false;
}

int getFileIndex(const char* pure_file_name)
{
	int k1 = 0, k2 = 0;
	for(k1=strlen(pure_file_name)-1; k1>0; k1--)
	{
		if( isADigit(pure_file_name[k1]) )
			break;
	}

	if( k1 == 0 && !isADigit(pure_file_name[0]) )
		return -1;

	for(k2=k1; k2>0; k2--)
	{
		if( !isADigit(pure_file_name[k2]) )
			break;
	}

	return atoi( &pure_file_name[k2+1] );
}

//=============Time Counting==========

std::stack<LARGE_INTEGER> g_start_t_;

void __tic__()
{
	LARGE_INTEGER t;
	::QueryPerformanceCounter(&t);
	g_start_t_.push(t);
}

double __toc__()
{
	LARGE_INTEGER end_t;
	::QueryPerformanceCounter(&end_t);
	LARGE_INTEGER freq;
	::QueryPerformanceFrequency(&freq);
	LARGE_INTEGER& start_t= g_start_t_.top();
	g_start_t_.pop();
	return (end_t.QuadPart-start_t.QuadPart)*1000.0/(double)freq.QuadPart;
}