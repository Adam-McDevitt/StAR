/*
 * Library for communicating between Arduino and Pi / app
 */
#ifndef PiConnection_h
#define PiConnection_h
#include "Arduino.h"

class PiConnection
{
	public:
		PiConnection();
		String readInstruction();
		void addNotification(String notification);
		void instructionDone();
		bool instructionAvailable();
		void send(String type, double x);
};

#endif