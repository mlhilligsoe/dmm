#ifndef GAUGE_H
#define GAUGE_H

#define WINVER 0x0500
#include <windows.h>

/* Setup keyboard control for guage measurements */
class Gauge {
public:
	Gauge();
	~Gauge();
	void measure();

private:
	INPUT ip;
};

#endif
