/*
 * ComputerInterface.h
 *
 * Created: 3/5/2015 8:45:11 AM
 *  Author: Nick McComb
 */ 


#ifndef COMPUTERINTERFACE_H_
#define COMPUTERINTERFACE_H_

#include "XMegaLib.h"

void initPCInterface(XMEGAID InputCurrentID);
void sendDriveResponse(DRIVE_RESPONSE input);

#endif /* COMPUTERINTERFACE_H_ */