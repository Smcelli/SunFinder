#include "SunFinder.h"

//gets current time, elapsed julian days, converts latitude/longitude to radians, then starts algorithm
std::tuple<double, double, double> sunFinderCartesian(double latitude, double longitude, bool degrees = 0) {
	double latitudeRadians, longitudeRadians;
	double elapsedJulianDays;
	double decimalHours;
	time_t timer = time(NULL);
	struct tm timeinfo;
	localtime_s(&timeinfo, &timer);

	if (degrees) {
		latitudeRadians = latitude * RADIANS;
		longitudeRadians = longitude * RADIANS;
	}
	else {
		latitudeRadians = latitude;
		longitudeRadians = longitude;
	}

	elapsedJulianDays = getDaysSince01012000(timer);
	decimalHours = getDecimalHoursSystem(&timeinfo);

	

	return sunFinderCartesian(latitudeRadians, longitudeRadians, elapsedJulianDays, decimalHours);
}

//main function assembling all steps of algorithm
std::tuple<double, double, double> sunFinderCartesian(double latitudeRadians, double longitudeRadians, double elapsedJulianDays, double decimalHours) {

	double eclipticLongitude, obliquityOfEcliptic;
	double rightAscension, declination;
	double azimuth, zenithAngle;
	double east, north, zenith;



	std::tie(eclipticLongitude, obliquityOfEcliptic) = calcSunElipticCoords(elapsedJulianDays);
	std::tie(rightAscension, declination) = calcCelestialCoords(eclipticLongitude, obliquityOfEcliptic);
	std::tie(azimuth, zenithAngle) = calcLocalCoords(decimalHours, elapsedJulianDays, latitudeRadians, longitudeRadians, rightAscension, declination);
	std::tie(east, north, zenith) = calculateCartesianCoords(azimuth, zenithAngle);
	std::tuple<double, double, double> cartesianCoords(east, north, zenith);
	return cartesianCoords;
}

//obtain elapsed julian Days
double getJulianDateSystem(time_t timer) {
	return JD_UNIX_EPOCH + ((double)timer / SECONDS_PER_DAY); 
}


double getDaysSince01012000(time_t timer) {
	return getJulianDateSystem(timer) - JD_0101200;
}

double getDaysSince01012000(double julianDate) {
	return julianDate - JD_0101200;
}

double getDecimalHoursSystem(tm * timeinfo) {
	double decimalHours;

	decimalHours = (double)timeinfo->tm_hour
		+ ((double)timeinfo->tm_min / 60)
		+ ((double)timeinfo->tm_sec / 3600);
	return decimalHours;
}

//get eliptic coordinates of sun (ecliptic longitude, obliquity of the ecliptic)
std::tuple<double, double> calcSunElipticCoords(double elapsedJulianDates) {
	double omega;
	double longitude;
	double anomaly;
	double eclipticLongitude;
	double obliquityOfEcliptic;

	omega = P[0] + (P[1] * elapsedJulianDates);
	longitude = P[2] + (P[3] * elapsedJulianDates);
	anomaly = P[4] + (P[5] * elapsedJulianDates);

	eclipticLongitude = longitude 
						+ (P[6] * sin(anomaly)) 
						+ (P[7] * sin(2 * anomaly)) 
						+ P[8] 
						+ (P[9] * sin(omega));

	obliquityOfEcliptic = P[10]
						+ P[11] * elapsedJulianDates
						+ (P[12] * cos(omega));

	std::tuple <double, double> elipticCoords(eclipticLongitude, obliquityOfEcliptic);
	return elipticCoords;
}

//calculate celesctial coordinates: right ascension, declination
std::tuple<double, double> calcCelestialCoords(double eclipticLongitude, double obliquityOfEcliptic) {
	double rightAscension;
	double declination;
	double dY, dX;

	dY = cos(obliquityOfEcliptic) * sin(eclipticLongitude);
	dX = cos(eclipticLongitude);
	rightAscension = atan2(dY, dX);
	while (rightAscension < 0)
		rightAscension += M_PI;
	while (rightAscension > PIx2)
		rightAscension -= PIx2;

	declination = asin(sin(obliquityOfEcliptic) * sin(eclipticLongitude));

	std::tuple <double, double> celestialCoords(rightAscension, declination);
	return celestialCoords;
}

//calculate local coordinates: azimuth, zenith angle
std::tuple<double,double> calcLocalCoords(double decimalHours, double elapsedJulianDates, double localLatitudeRadians, double localLongitudeRadians, double rightAscension, double declination) {
	double greenwichMeanSiderealTime;
	double hourAngle;
	double dY, dX;
	double azimuth;
	double zenithAngle;
	
	greenwichMeanSiderealTime = P[13]
								+ (P[14] * elapsedJulianDates)
								+ decimalHours;
	hourAngle = (greenwichMeanSiderealTime * M_PI / 12)
				+ localLongitudeRadians;
	
	//calculate azimuth where 0 <= azimuth < 2*PI 
	dY = sin(hourAngle) * -1;
	dX = (tan(declination) * cos(localLatitudeRadians))
		- (sin(localLatitudeRadians) * cos(hourAngle));
	azimuth = atan2(dY, dX);
	while (azimuth < 0)
		azimuth += M_PI;
	while (azimuth >= PIx2)
		azimuth -= PIx2;

	//calculate zanith angle
	dX = (cos(localLatitudeRadians) * cos(declination) * cos(hourAngle))
		+ (sin(localLatitudeRadians) * sin(declination));
	zenithAngle = acos(dX);
	zenithAngle += EARTH_MEAN_RADIUS / (ASTRONOMICAL_UNIT * sin(zenithAngle));

	std::tuple<double, double> localCoords(azimuth, zenithAngle);
	return localCoords;
}

//transform to cartesian coordinate fram: east, north, zenith
std::tuple<double, double, double> calculateCartesianCoords(double azimuth, double zenithAngle) {
	double east, north, zenith;
	
	east = sin(zenithAngle) * sin(azimuth);
	north = sin(zenithAngle) * cos(azimuth);
	zenith = cos(zenithAngle);

	std::tuple<double, double, double> cartesianCoords(east, north, zenith);
	return cartesianCoords;
}

