#include "PiConnection.h"
int r = 1;
String s;
PiConnection pc(9600);
void setup(){

}
void loop(){
    s = pc.readInstruction();
    int n = s.toInt();
    for (int i = 1 ; i <= n ; i++)
    {
      delay(1000);
      pc.addNotification(String("Notification"+String(i)));
    }
    pc.instructionDone();
}
