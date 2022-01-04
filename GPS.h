#ifndef GPS_H
#define GPS_H

#include<string>
#include<iostream>
#include<vector>

using std::vector;
using std::string;
using std::cout;
using std::endl;

vector<string> testSplit(string srcStr, const string& delim);
unsigned char hexToDec(unsigned char hex);

class GPS
{
public:
	GPS();
	~GPS();
	void parseNAME(string);
	
	bool valid;
	double lat, lon;
	float velocity, course, altitude;
	uint8_t SVs;
	float HDOP;

private:
	const string header = "$GN";  
	const string footer = "\r\n";
	string payload;

    bool parse(string str, int &parseLen);
	bool checkCRC();
};

#endif