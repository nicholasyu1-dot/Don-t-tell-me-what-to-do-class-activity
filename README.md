# Don-t-tell-me-what-to-do-class-activity
This repository is for my Code base and, readme file for this assignment, using the code I made for the simple course navigation,included.
This Progam will be able to navigate a simeple course assumeing the walls are made of tape since it uses line  tracking to detect walls. also MAKE SURE THE CORENERS ARE 90DEG AS THIS IS JUST A SIMPLE COURSE NAVIGATION.

To run this you will first need the ELEGOO SMART ROBOT CAR KIT 4.0, not sure if the others will work. 
Next, you will need the arduino IDE and put the code into a .ino file or dowload the Robot.ino file
Then, you will need to plug in the  robot using the usb-A to the robot upload port, and MAKE SURE ITS ON UPLOAD MODE
Next, you need to select the Arduino uno board in the top left corner and select the port for the robot, mostlikely will only be one.
Then, press the upload button looks like this (->) so that the code will be put on the robot.

Now that the code is uploaded onto the robot simply unplug it, place it down in a maze, note its coded for WHITE tape so make sure its on a much darker surface so that it can actually detect the white.
and turn it on.
then, press the black long button that is on the same side ans the upload port.
then it should run and go through the maze

# THINGS OF NOTE
This code is written and calibarted for MY ROBOT
So, for example: the Servo motor is supposed to be centered when set to 90, but mine is centered at 95 so change the code to your robot
Also: i thihnk my left motors are slower so that it will drift a bit to that side
also, My gryo seems to drift more that others, but DRIFT IS NORMAL FOR THIS BOARD.
The line tracking is alsoyt per-robot so the 325 is white masking tape for my robot but may be differnt for your robot

# Other files
These files, the .h files and the base.ino file are for base peices of code for making your own programs.
