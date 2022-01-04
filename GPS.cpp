#include "GPS.h"

#define GPSDEBUG

vector<string> testSplit(string srcStr, const string& delim)
{
	int nPos = 0;
	vector<string> vec;
	nPos = srcStr.find(delim.c_str());
	while (nPos != string::npos)
	{
		string temp = srcStr.substr(0, nPos);
		vec.push_back(temp);
		srcStr = srcStr.substr(nPos + 1);
		nPos = srcStr.find(delim.c_str());
	}
	vec.push_back(srcStr);
	return vec;
}

unsigned char hexToDec(unsigned char hex)
{
	if (hex >= 'A')
		return (hex - 'A') + 10;
	else
		return hex - '0';
}

GPS::GPS()
{
}


GPS::~GPS()
{
}

bool GPS::checkCRC()  //CRC校验
{
	int pos = payload.find('*');
	if(pos != string::npos)
	{
		unsigned char checkSum = 0;
		unsigned char CRC = (hexToDec(payload[pos + 1]) << 4) + hexToDec(payload[pos + 2]);
		for (int i = 1; i < pos; i++)
			checkSum ^= payload[i];
		if (checkSum == CRC)
			return true;
		else
			return false;
	}
	else
		return false;
}

void GPS::parseNAME(string parseStr)
{
	int parseLength = 1;
	int len = static_cast<int>(parseStr.length());
	
	while (len > parseLength)
	{
		string str = parseStr.substr(parseLength - 1);
#ifdef GPSDEBUG
		cout << str << endl;
#endif // GPSDEBUG
		if (parse(str, parseLength))  //成立则获取到一条完整的NMEA消息
		{
			if (checkCRC()) //进行CRC校验，校验该条NMEA消息数据是否准确。
			{
				vector<string> strs = testSplit(payload, ",");
#ifdef GPSDEBUG
				cout << "goodCRC" << "  "<< strs.size() << endl;
#endif // GPSDEBUG
				//若校验通过，以下代码提取经纬度，航速和航向信息
				if (strs[0] == "$GNRMC") //"$GNRMC":GNSS推荐定位消息；
				{
#ifdef GPSDEBUG
				cout << "gotGNRMC" << endl;
#endif // GPSDEBUG
					if(strcmp(strs[2].c_str(),"A") == 0)
						valid = true;
					else
						valid = false;
					int pos = strs[3].find('.');
					if(pos != string::npos)
						lat = atof(strs[3].substr(0, 2).c_str()) + atof(strs[3].substr(pos - 2).c_str()) / 60;
					pos = strs[5].find('.');
					if(pos != string::npos)
						lon = atof(strs[5].substr(0, 3).c_str()) + atof(strs[5].substr(pos - 2).c_str()) / 60;
					velocity = atof(strs[7].c_str());
					course = atof(strs[8].c_str());	
				}
				else if (strs[0] == "$GNGGA")
				{
#ifdef GPSDEBUG
					cout << "gotGNGGA" << endl;
#endif // GPSDEBUG
					int pos = strs[2].find('.');
					if(pos != string::npos)
						lat = atof(strs[2].substr(0, 2).c_str()) + atof(strs[2].substr(pos - 2).c_str()) / 60;
					pos = strs[4].find('.');
					if(pos != string::npos)
						lon = atof(strs[4].substr(0, 3).c_str()) + atof(strs[4].substr(pos - 2).c_str()) / 60;
					SVs = atoi(strs[7].c_str());
					HDOP = atof(strs[8].c_str());
					altitude = atof(strs[9].c_str());
				}
			}
		}
	}
}

bool GPS::parse(string str, int &parseLen)
{
#ifdef GPSDEBUG
	cout << "parsing..." << endl;
#endif // GPSDEBUG
	static bool get_header = false;
	if (get_header)
	{
		int pos_footer = str.find(footer);
		if (pos_footer != string::npos)
		{
#ifdef GPSDEBUG
			cout << "gotFooter" << endl;
#endif // GPSDEBUG
			payload += str.substr(0, pos_footer + 2);
			get_header = false;
			parseLen += pos_footer + 2;
			return true;
		}
		else
		{
			payload += str;
			if (payload.length() >= 100)
			{
				payload.clear();
				get_header = false;
			}
			parseLen += str.length();
			return false;
		}
	}
	else
	{
		int pos_header = str.find(header);
		if (pos_header != string::npos)
		{
#ifdef GPSDEBUG
			cout << "gotHeader" << endl;
#endif // GPSDEBUG
			payload.clear();
			get_header = true;
			int pos_footer = str.find(footer, pos_header);
			if (pos_footer != string::npos)
			{
#ifdef GPSDEBUG
				cout << "gotFooter" << endl;
#endif // GPSDEBUG
				payload = str.substr(pos_header, pos_footer - pos_header + 2);
				get_header = false;
				parseLen += pos_footer + 2;
				return true;
			}
			else
			{
				payload += str.substr(pos_header);
				parseLen += str.length();
				return false;
			}
		}
		else
		{
			parseLen += str.length();
			return false;
		}
	}
}
