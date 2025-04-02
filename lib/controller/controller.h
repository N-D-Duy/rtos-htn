#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "service.h"


class Controller
{
public:
    Controller();
    void setup();
    boolean setDoorStatus(bool status);
    boolean setRingStatus(bool status);
    boolean getDoorStatus();
    void streamData();
private:
    Service service; 
};

#endif
