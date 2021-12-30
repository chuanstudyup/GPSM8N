// testMain.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
// g++ GPS.cpp testMain.cpp -o testMain

/* NMEA examples
$GNRMC,083712.40,A,3030.83159,N,11424.56558,E,0.150,,291221,,,A*65\r\n
$GNRMC,083712.60,A,3030.83159,N,11424.56559,E,0.157,,291221,,,A*61\r\n
$GNRMC,083712.80,A,3030.83158,N,11424.56558,E,0.033,,291221,,,A*6C\r\n
$GNGGA,083712.80,3030.83158,N,11424.56558,E,1,10,1.00,49.7,M,-10.6,M,,*5B\r\n
*/

#include <iostream>
#include <math.h>
#include "GPS.h"

using std::cout;
using std::endl;

int main()
{

	char buf0[200] = "$GNRMC,083708.20,A,3030.";
	char buf1[200] = "83171,N,11424.56579,E,0.094,,291221,,,A*68\r\n$GNGGA,0";
	char buf2[200] = "83712.80,3030.83158,N,11424.56558,E,1";
	char buf3[200] = ",10,1.00,49.7,M,-10.6,M,,*5B\r\n";
	
	GPS gps;
	gps.read(buf0);
	gps.read(buf1);
	gps.read(buf2);
	gps.read(buf3);

	cout <<"Lat:"<< gps.lat << endl;
	cout <<"Lng:"<< gps.lon << endl;
	cout <<"Velocity:"<< gps.velocity << endl;
	cout <<"Course:"<< gps.course << endl;
	cout << "SVs:" << static_cast<int>(gps.SVs) << endl;
	cout << "Altitude:" << gps.altitude << endl;
	cout << "HDOP:" << gps.HDOP << endl;
	return 0;
}