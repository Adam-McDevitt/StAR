/*
 * Library for communicating between Arduino and Pi / app
 */
#include "PiConnection.h"
#include "Arduino.h"
#include <SoftwareSerial.h>
PiConnection::PiConnection()
{

}
String PiConnection::readInstruction()
{
	if(Serial.available() > 0)
	{
		String s = Serial.readString();
		return s;
	}
	else
	{
		return "";
	}
}
void PiConnection::addNotification(String notification)
{
	Serial.println(notification);
}
void PiConnection::instructionDone()
{
	Serial.println("done");
}
bool PiConnection::instructionAvailable()
{
	return (Serial.available() > 0);
}

void PiConnection::send(String type,double x){

	Serial.print(type);
	Serial.print(" ");
	Serial.println(x);
}