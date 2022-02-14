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
	valid = false;
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
					velocity = atof(strs[7].c_str())*0.5144;
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

/* Description: initialize EKF. 
 * Regard the current position (from GNSS) and yaw (from IMU) as the init state of EKF;
 * @param yaw(deg) The heading angle of vehicle(May obtain from IMU).
 */
void GPS::initEKF(double yaw)
{
	x_hat << lat/d2r,lon/d2r,0,yaw/d2r,0;
	P_hat = Matrix5d::Identity() * 1e-3;
	
	oldTime = micros();
}

/* Descripiton: A five state EKF to filter the course angle
 * State: [lat(rad),lon(rad),velocity(m/s),chi(rad),r(rad)];
 * Measurement: [lat_GNSS(rad),lon_GNSS(rad),vel_GNSS(rad),r(IMU)];
 * Reference: Five-state extended kalman filter for estimation of speed over ground (SOG), course over ground (COG), and course rate of unmanned surface vehicles (USVs): Experimental results
 * 			 (DOI: 10.3390/s21237910)
 * @param: r(deg/s)
 */
void GPS::gnssEKF(double r)
{
	newTime = micros();
	deltaT = (newTime-oldTime)*0.001*0.001;
	
	double midValue = 1-ee*sin(x_hat(2))*sin(x_hat(2));
	double Rn = a/sqrt(midValue);
	double Rm = Rn*(1-ee)/midValue;
	
	Vector5d f(x_hat(3)*cos(x_hat(4))/Rm,
			   x_hat(3)*sin(x_hat(4))/Rn/cos(x_hat(1)),
			   -alpha_1*x_hat(3),
			   x_hat(3),
			   -alpha_2*x_hat(5));
	Matrix5d Acom{{0,0,cos(x_hat(4))/Rm,-x_hat(3)*sin(x_hat(4))/Rm,0},
				  {x_hat(3)*sin(x_hat(4))*tan(x_hat(1))/Rn/cos(x_hat(1)),0,sin(x_hat(4))/Rn/cos(x_hat(1)),x_hat(3)*cos(x_hat(4))/Rn/cos(x_hat(1)),0},
				  {0,0,-alpha_1,0,0},
				  {0,0,0,0,1},
				  {0,0,0,0,-alpha_2}};
	
	Matrix5d A = Matrix5d::Identity()+deltaT*Acom;
	Vector5d x_prd = x_hat+ deltaT * f;
	x_prd(4) = satCourse(x_prd(4));
	
	Matrix5d p_prd = A*P_hat*A.transpose()+E*Q*E.transpose();
	
	Vector4d y(lat/d2r,lon/d2r,velocity,r/d2r);
	Matrix<double,5,4> K = p_prd*C.transpose()*((C*p_prd*C.transpose()+R).inverse());
	P_hat = (Matrix5d::Identity()-K*C)*p_prd;
	x_hat = x_prd+K*(y-C*x_prd);
	x_hat(4) = satCourse(x_hat(4));
	
	oldTime = newTime;
}

double GPS::satCourse(double course)
{
	while(course<0)
	{
		course+=2*M_PI;
	}
	while(course>2*M_PI)
	{
		course-=2*M_PI;
	}
	return course;
}
/* Description: Parse NAME and run EKF if get valid position infomation;
 * Call this functuon must after GPS::initEKF.
 * @param parseStr The original string from serial or network.
 * @param r The rate of yaw in uints deg/s
 */
void GPS::parseNAME(string parseStr, double r)
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
					{
						valid = true;
						int pos = strs[3].find('.');
						if(pos != string::npos)
							lat = atof(strs[3].substr(0, 2).c_str()) + atof(strs[3].substr(pos - 2).c_str()) / 60;
						pos = strs[5].find('.');
						if(pos != string::npos)
							lon = atof(strs[5].substr(0, 3).c_str()) + atof(strs[5].substr(pos - 2).c_str()) / 60;
						velocity = atof(strs[7].c_str())*0.5144;
						course = atof(strs[8].c_str());
						gnssEKF(r);
					}
					else
						valid = false;
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
