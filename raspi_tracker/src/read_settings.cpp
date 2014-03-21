#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
using namespace std;

int VIDEO_FRAME_RATE_NUM;
int RC_WIDTH;
int RC_HEIGHT;
int RC_SHUTTER_SPEED;
int RC_ISO;
int GRAYMODE;
int MINSIZE;
int THRESHOLD;
int NUMBER_OF_FRAMES;
int BACKGROUND_RATIO_NUMERATOR;
int BACKGROUND_RATIO_DENOMINATOR;

int ReadSettings (void)
{
  
  ifstream iFile("/home/pi/catkin_ws/src/raspi_tracker/src/raspicam_settings.config");
  string line;

  std::string delimiter = ",";

  while(getline(iFile, line)) {

	size_t pos = 0;
	std::string token;
	while ((pos = line.find(delimiter)) != std::string::npos) {
		token = line.substr(0, pos);
		line.erase(0, pos + delimiter.length());
	}
	int n = atoi(line.c_str());

	if 		(token 	== 	"VIDEO_FRAME_RATE_NUM") VIDEO_FRAME_RATE_NUM = n;
	else if (token 	==  "RC_WIDTH") 			RC_WIDTH = n;
	else if (token 	==  "RC_HEIGHT") 			RC_HEIGHT = n;
	else if (token 	==  "RC_SHUTTER_SPEED") 	RC_SHUTTER_SPEED = n;
	else if (token 	==  "RC_ISO") 				RC_ISO = n;
	else if (token 	==  "GRAYMODE") 			GRAYMODE = n;
	else if (token 	==  "MINSIZE") 				MINSIZE = n;
    	else if (token 	==  "THRESHOLD") 			THRESHOLD = n;
	else if (token 	==  "NUMBER_OF_FRAMES") 	NUMBER_OF_FRAMES = n;
	else if (token 	==  "BACKGROUND_RATIO_NUMERATOR") 	BACKGROUND_RATIO_NUMERATOR = n;
	else if (token 	==  "BACKGROUND_RATIO_DENOMINATOR") 	BACKGROUND_RATIO_DENOMINATOR = n;

  }

  return 0;
}
