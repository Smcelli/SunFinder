#pragma once

#define _USE_MATH_DEFINES
#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

#include <cmath>
#include <ctime>
#include <tuple>

double const JD_UNIX_EPOCH = 2440587.5;
double const JD_0101200 = 2451545.0;
double const SECONDS_PER_DAY = 86400;
double const EARTH_MEAN_RADIUS = 6371.01;
double const ASTRONOMICAL_UNIT = 149597870.7;
double const PIx2 = 2 * M_PI;
double const RADIANS = M_PI / 180;
//psa algorithm constants for 2000 - 2050
double const P[15] = { 2.1429,				//p0 omega
						-0.0010394594,		//p1 omega
						4.8950630,			//p2 longitude
						0.017202791698,		//p3 longitude
						6.2400600,			//p4 anomaly
						0.0172019699,		//p5 anomaly
						0.03341607,			//p6 ecliptic longitude
						0.00034894,			//p7 ecliptic longitude
						-0.0001134,			//p8 ecliptic longitude
						-0.0000203,			//p9 ecliptic longitude
						0.4090928,			//p10 obliquity of the ecliptic
						-6.2140e-9,			//p11 obliquity of the ecliptic
						0.0000396,			//p12 obliquity of the ecliptic
						6.6974243242,		//greenwich mean sidereal time
						0.0657098283 };		//greenwich mean sidereal time




std::tuple<double, double, double> sunFinderCartesian(double latitude, double longitude, bool degrees);

std::tuple<double, double, double> sunFinderCartesian(double latitudeRadians, double longitudeRadians, double elapsedJulianDays, double decimalHours);

double getJulianDateSystem(time_t timer);

double getDaysSince01012000(time_t timer);

double getDaysSince01012000(double);

double getDecimalHoursSystem(tm* timeinfo);

std::tuple<double, double> calcSunElipticCoords(double elapsedJulianDates);

std::tuple<double, double> calcCelestialCoords(double eclipticLongitude, double obliquityOfEcliptic);

std::tuple<double, double> calcLocalCoords(double decimalHours, double elapsedJulianDates, double localLatitudeRadians, double localLongitudeRadians, double rightAscension, double declination);

std::tuple<double, double, double> calculateCartesianCoords(double azimuth, double zenithAngle);
