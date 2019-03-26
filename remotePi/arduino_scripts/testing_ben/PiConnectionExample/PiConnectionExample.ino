#include "PiConnection.h"
int r = 1;
String s;
// initialises PiConnection instance, with baud rate 9600 for the serial
PiConnection pc;
void setup(){
  Serial.begin(9600);
}
void loop(){
    // Check if an instruction has been written to serial
    if (pc.instructionAvailable())
    {
      // Read instruction as a string
      // If this is true, it means that the pi should be listening for notifications until it is
      // told to stop
      s = pc.readInstruction();
      // This simple example expects instructions to be an integer in string form
      int n = s.toInt();
      // It will add count up to the integer received, adding a notification for each iteration
      for (int i = 1 ; i <= n ; i++)
      {
        delay(1000);
        // Adds a notification to the notification list
        pc.addNotification(String("Notification"+String(i)));
      }
      // Informs the pi to stop listening for notifications
      pc.instructionDone();
    }
}
