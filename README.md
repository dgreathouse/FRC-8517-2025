# FRC Team 8517 2025 Code Overview
## General
This code is a Java base for a FRC robot using a swerve drive. The code for the swerve drive is mostly custom code.
The code utilizes all CTRE motor controllers and Phoenix Pro. Nothing fancy in this code.
Changes to the base projects have been done to simplify the files for student understanding. This means files have been optimized and motor control has been minimized.

## Robot Overview
The base robot is our typical 3 wheel swerve drive system. Yes a swerve can easily be done with 3 wheels. 
Swerve drive power, speed and acceleration trade-offs are minimal with only 3 wheels when the robots weight is maintained.

## Current Code Structure
The code utilizes the Java WPILIb Command Based programming framework. 
### autoCommands
Contains all the commands and command groups associated with autonomous modes.
### commands
Contains all the subsystem default commands and other commands used in tele-Op.
### lib
Contains extra classes and enumerations used by various files.
<p>
One specific file called g.java "Globals" contains all the static values needed by different classes.
This globals class contains internal classes of data. The data in the classes are of two types.
</p>
<p>
<b>Globals Static Values</b> that can be changed. These values have a camel case naming convention with the first letter lower case.
</p>
<p>
<b>Global Static Constants</b> that can not be changed during program execution. These constants have a naming convention of all capitols with underscore seperating the names.</p>

<p><b>Units</b> are represented at the end of the name with an underscore and a lower case unit. The WPILib Units library is only used when we are forced to use it in the WPILib classes. The WPILib units library is a second iteration of adding units to the WPILib and I think they still need more work to get the units library easier for students to use and understand. Therefore we are going to ignore their units library until it has been around for a couple of years.</p>

### subsystems
The subsystems folder simply contains all the subsystem classes that make up the robot. 

### robot
The robot folder contains the initial Main class and Robot.java. Robot.java has all the functionality that RobotContainer had before. 

### vendordeps
This folder contains all the vendor dependencies the robot needs. Such dependencies like Phoneix Lib, RevLib, ...

### Where are all the other files we are use to seeing?
The file .vscode/settings.json has been modified to hide all the other files I felt were not needed to be seen in the file list.


