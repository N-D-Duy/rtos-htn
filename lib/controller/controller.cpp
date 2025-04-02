#include "controller.h"

Controller::Controller() : service()
{
}

void Controller::setup()
{
    service.initService();
}

boolean Controller::setDoorStatus(bool status)
{
    return service.setDoorStatus(status);
}

boolean Controller::setRingStatus(bool status)
{
    return service.setRingStatus(status);
}

boolean Controller::getDoorStatus()
{
    return service.getDoorStatus();
}

void Controller::streamData()
{
    service.syncFirebaseData();
}
