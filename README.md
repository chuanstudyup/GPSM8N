# GPSM8N
A class to parse NAME protocol  from GPS such as ublox M8N using c++

# Add a five state EKF to filter the course angle
State: [lat(rad),lon(rad),velocity(m/s),chi(rad),r(rad)];

Measurement: [lat_GNSS(rad),lon_GNSS(rad),vel_GNSS(m/s),r_IMU(rad/s)];

Reference: Five-state extended kalman filter for estimation of speed over ground (SOG), course over ground (COG), and course rate of unmanned surface vehicles (USVs): Experimental results(DOI: 10.3390/s21237910)
