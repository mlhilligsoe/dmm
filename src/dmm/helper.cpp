
#include <iostream>
#include <ctime>
#include <chrono>
#include "dmm/helper.h"

using std::cout;
using std::cerr;
using std::endl;
using namespace std::chrono;

void pt() {
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	cout << (now->tm_year + 1900) << '-'
		<< (now->tm_mon + 1) << '-'
		<< now->tm_mday << ' '
		<< now->tm_hour << ':'
		<< now->tm_min << ':'
		<< now->tm_sec << ' ';
}


void ptm() {
	milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	cout << ms.count() << ' ';
}