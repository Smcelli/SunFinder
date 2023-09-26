#include <iostream>
#include <iomanip>
#include <string>
#include "SunFinder.h"

void print(std::string &location, double latitude, double longitude, bool degrees) {
	double east, north, zenith;
	std::cout << "you are at " << location << ", coordinates: " << latitude << ", " << longitude << std::endl;
	std::tie(east, north, zenith) = sunFinderCartesian(latitude, longitude, degrees);
	std::cout << "Sun's cartesian coordinates are: " << east << ", " << north << ", " << zenith << std::endl << std::endl;
}

int main() {
	std::string input = "san ramon";
	print(input, 37, 45, 1);
	return 0;
}

