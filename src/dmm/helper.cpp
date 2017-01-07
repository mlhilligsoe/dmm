
#include <iostream>
#include <ctime>
#include <chrono>
#include <string>
#include "dmm/helper.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using namespace std::chrono;

string pt() {
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%d-%m-%Y %I:%M:%S", timeinfo);
	std::string str(buffer);

	return str;
}


string ptm() {
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	cout << ms.count() << ' ';
	
	return "...";
}