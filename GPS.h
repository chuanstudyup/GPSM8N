#ifndef GPS_H
#define GPS_H

#include <string>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "../ArduTime.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

using namespace Eigen;
typedef Matrix<double, 5, 5> Matrix5d;
typedef Matrix<double, 5, 1> Vector5d;

vector<string> testSplit(string srcStr, const string& delim);
unsigned char hexToDec(unsigned char hex);

class GPS
{
public:
	GPS();
	~GPS();
	void parseNAME(string);
	void parseNAME(string,double);
	void initEKF(double yaw);
	
	bool valid;
	double lat, lon;
	float velocity, course, altitude;
	uint8_t SVs;
	float HDOP;
	
	double getEKFLat() const {return x_hat(0)*d2r;};
	double getEKFLon() const {return x_hat(1)*d2r;};
	double getEKFVel() const {return x_hat(2);};
	double getEKFCourse() const{return x_hat(3)*d2r;};
	double getEKFr() const{return x_hat(4)*d2r;};

private:
	const string header = "$GN";  
	const string footer = "\r\n";
	string payload;
	
    bool parse(string str, int &parseLen);
	bool checkCRC();
	
	const double a = 6378137;
	const double ee = 0.00669437999;
	const double d2r = 57.2957795130;
	
	/*GNSS EKF*/
	double deltaT{0.};
    uint32_t newTime{0}, oldTime{0};
	Vector5d x_hat;
	float alpha_1 = 0.01;
	float alpha_2 = 0.1;
	Matrix5d P_hat = Matrix5d::Identity() * 0.01;
    Matrix2d Q{{5,0},{0,1}};
	Matrix4d R{{1e-14,0,0,0},
			   {0,1e-14,0,0},
			   {0,0,0.2,0},
			   {0,0,0,0.001}};
	Matrix<double,4,5> C{{1,0,0,0,0},
						 {0,1,0,0,0},
						 {0,0,1,0,0},
						 {0,0,0,0,1}};
	Matrix<double,5,2> E{{0,0},{0,0},{1,0},{0,0},{0,1}};
	void gnssEKF(double r);
	double satCourse(double course);
};

#endif
