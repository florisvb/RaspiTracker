# Introduction

This code is a simple program to use the Raspberry Pi and Raspberry Pi Camera to do simple object tracking.

The vast majority of this code was originally written by Pierre Raufast and Robidouille:

https://robidouille.wordpress.com/2013/10/19/raspberry-pi-camera-with-opencv/
https://thinkrpi.wordpress.com/2013/05/22/opencv-and-camera-board-csi/


# Installation

1. Install Raspbian. 
 
   Easiest method is using NOOBS  
   * http://www.raspberrypi.org/downloads   

  On first boot-up, in the configuration tool
   * enable camera   
   * advanced options, enable SSH   

   Once booted  
   * login: pi 
   * password: raspberry 
   * startx: will bring you to a window manager 
   
2. Update

   *sudo apt-get update*   

3. (optional) Install some helpful tools:

   *sudo apt-get install locate* 
   *sudo updatedb* 

4. Install cmake and opencv:

   *sudo apt-get install cmake libcv-dev libhighgui-dev libopencv-dev libcv2.3*

5. Download and install the userland raspberry pi camera code:

   (clone the userland git directory to the desktop, or elsewhere)   

   *git clone https://github.com/raspberrypi/userland.git*  
   *cd userland*  
   *cmake .*  
   *make*  
   
   (this takes about 15 minutes)  
   
   Move to /opt/vc:
   
   *cd ..*
   *sudo mv userland /opt/vc*

   Alternatively, you will need to change the CMakeLists.txt for RaspiTracker

6. Download and build the RaspiTracker code:

   *git clone https://github.com/florisvb/RaspiTracker.git*  
   *cd RaspiTracker*  
   *cmake .*  
   *make*

7. Run the code:

   ./raspicambg  


# Usage Notes

1. Changing the resolution (e.g. when switching between the S-Video screen and HDMI monitor:
   *sudo nano /boot/config.txt*
   *edit the hdmi resolution, and that's it*
   *more info at: http://elinux.org/R-Pi_ConfigurationFile*

2. Wiring Pi Permissions Fix:
   *after compiling pi_rotary_encoder run:*
   *sudo chown root:root catkin_ws/devel/lib/pi_rotary_encoder/encoder_talker*
   *sudo chmod 4755 catkin_ws/devel/lib/pi_rotary_encoder/encoder_talker*

3. USB stick:
   *Hard coded to be a patriot USB stick, without putting a patriot USB stick in it won't work*
