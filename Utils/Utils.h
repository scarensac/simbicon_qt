#ifndef UTILS_H
#define UTILS_H

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>


#define DynamicArray std::vector
typedef unsigned int uint;


//define a prototype to a generic output-to log file function.
void logPrint(const char *format, ...);

//this one is an easy printer
void debugLog(std::string msg);

/**
	This method throws an error with a specified text and arguments 
*/
inline void throwError(const char *fmt, ...){		// Custom error creation method

	char		text[256];								// Holds Our String
	va_list		ap;										// Pointer To List Of Arguments

	if (fmt == NULL)									// If There's No Text
		return;											// Do Nothing

	va_start(ap, fmt);									// Parses The String For Variables
	    vsprintf(text, fmt, ap);						// And Converts Symbols To Actual Numbers
	va_end(ap);											// Results Are Stored In Text

	throw text;
}


/**
	This method reads all the doubles from the given file and stores them in the array of doubles that is passed in
*/
inline void readDoublesFromFile(FILE* f, DynamicArray<double> *d){
	double temp;
	while (fscanf(f, "%lf\n", &temp) == 1)
		d->push_back(temp);
}


/**
	This method returns a pointer to the first non-white space character location in the provided buffer
*/
inline char* lTrim(char* buffer){
	while (*buffer==' ' || *buffer=='\t' || *buffer=='\n' || *buffer=='\r')
		buffer++;
	return buffer;
}

inline char* rTrim(char* buffer){
	int index = (int)strlen(buffer) - 1;
	while (index>=0){
		if (buffer[index]==' ' || buffer[index]=='\t' || buffer[index]=='\n' || buffer[index]=='\r'){
			buffer[index] = '\0';
			index--;
		}
		else
			break;
	}
	return buffer;
}

inline char* trim(char* buffer){
	return rTrim(lTrim(buffer));
}

/**
	This method reads a line from a file. It does not return empty lines or ones that start with a pound key - those are assumed to be comments.
	This method returns true if a line is read, false otherwise (for instance the end of file is met).
*/
inline bool readValidLine(char* line, FILE* fp){
	while (!feof(fp)){
		fgets(line, 100, fp);
		char* tmp = trim(line);
		if (tmp[0]!='#' && tmp[0]!='\0')
			return true;
	}

	return false;
}

/**
	This method returns a DynamicArray of char pointers that correspond to the addressed
	of the tokens that are separated by white space in the string that is passed in as a pointer.
*/
inline DynamicArray<char*> getTokens(char* input){
	DynamicArray<char*> result;
	input = lTrim(input);
	//read in the strings one by one - assume that each tokens are less than 100 chars in length
	while (input[0]!='\0'){
		result.push_back(input);
		char tempStr[100];
		sscanf(input, "%s", tempStr);
		input = lTrim(input + strlen(tempStr));
	}
	return result;
}



/**
those method are helper to split a string
*/
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);

/**
this function interpret the path (using the global configuration data path
*/
std::string interpret_path(std::string path);

/**
 this function look for the deta folder
 */
std::string get_folder_path(std::string name, int lookup_nbr, std::string delim="/", std::string start_folder="./");


void rmv_white_spaces(std::string& s);


std::string tab_string(int nb_tabulation);


//*
#include <algorithm>

template <class T>
class IdxSorter
{
private:
    const T mparr;
public:
    IdxSorter(const T parr) : mparr(parr) {}
    bool operator()(int i, int j) const { return mparr[i]<mparr[j]; }
};
//*/


double generate_random_0_1();
#include <MathLib/mathLib.h>

//a function that returns the signed distance between two radiasn value
double radian_distance_signed(double from, double to);


#endif
