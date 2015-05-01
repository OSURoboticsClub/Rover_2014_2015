/*
 * navigate.cpp
 *
 *  Created on: Apr 30, 2015
 *      Author: scott
 */

#include "HomeBase.hpp"

using namespace std;

vector<int> createCmd(int move, int val){
	vector<int> cmd(2);
	cmd[0] = move;
	cmd[1] = val;
	return cmd;
}
vector<vector<int> > generateCommands(float theta, float D, float offset){
	vector<vector<int> > cmds;
	if(abs(theta) < 10){
		if(D > FARAWAY){
			cmds.push_back(createCmd(FWD, D/2));
			return cmds;
		}else {
			if(D < CLOSE){
				cmds.push_back(createCmd(FWD, D));
			} else {
				cmds.push_back(createCmd(FWD, NEAR_DIST));
			}
			return cmds;
		}
	} else {
		if(D > FARAWAY){
			float d = D * sin(abs(theta));
			if(theta < 0){
				cmds.push_back(createCmd(TURN_LEFT, 90-abs(theta)));
				cmds.push_back(createCmd(FWD, d));
				cmds.push_back(createCmd(TURN_RIGHT, 90));
			} else {
				cmds.push_back(createCmd(TURN_RIGHT, 90-abs(theta)));
				cmds.push_back(createCmd(FWD, d));
				cmds.push_back(createCmd(TURN_LEFT, 90));
			}
		} else {
			float d = D * tan(abs(theta));
			if(theta < 0){
				cmds.push_back(createCmd(TURN_LEFT, 90));
				cmds.push_back(createCmd(FWD, d));
				cmds.push_back(createCmd(TURN_RIGHT, 90+abs(theta)));
			} else {
				cmds.push_back(createCmd(TURN_RIGHT, 90));
				cmds.push_back(createCmd(FWD, d));
				cmds.push_back(createCmd(TURN_LEFT, 90+abs(theta)));
			}
		}
	}
	return cmds;
}
void printCommands(vector<vector<int> > cmds){
	for(int i =0; i < cmds.size(); i++){
		int c = cmds[i][0];
		switch(c)
		{
		case FWD:
			cerr << "Forward: " << cmds[i][1] << "mm" << endl;
			break;
		case TURN_LEFT:
			cerr << "Turn Left: " << cmds[i][1] << " degrees" << endl;
			break;
		case TURN_RIGHT:
			cerr << "Turn Left: " << cmds[i][1] << " degrees" << endl;
			break;
		default:
			break;
		}
	}
}


