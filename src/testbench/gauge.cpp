#include "testbench\gauge.h"
#include <windows.h>

Gauge::Gauge() {
	ip.type = INPUT_KEYBOARD;
	ip.ki.wScan = 0; // hardware scan code for key
	ip.ki.time = 0;
	ip.ki.dwExtraInfo = 0;
	ip.ki.wVk = VK_F7; // virtual-key code for the "F7" key
}

Gauge::~Gauge() {

}

void Gauge::measure() {
	// Press the "F7" key
	ip.ki.dwFlags = 0; // 0 for key press
	SendInput(1, &ip, sizeof(INPUT));

	// Release the "F7" key
	ip.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
	SendInput(1, &ip, sizeof(INPUT));
}