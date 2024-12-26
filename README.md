# FRC Team 8517 2025 Code Overview
## General
This code is a Java base for a FRC robot using a swerve drive. The code for the swerve drive is mostly custom code.  
The code utilizes all CTRE motor controllers and Phoenix Pro. Nothing fancy in this code.  
Changes to the base projects have been done to simplify the files for student understanding.  
This means files have been optimized and motor control has been minimized.

## Robot Overview
The base robot is our typical 3 wheel swerve drive system.  
Yes a swerve can easily be done with 3 wheels.   
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
<b>Global Static Constants</b> that can not be changed during program execution. These constants have a naming convention of all capitals with a underscore seperating the names.</p>

<b>Units</b> are represented at the end of the name with an underscore and a lower case unit.  
The WPILib Units library is only used when we are forced to use it in the WPILib classes.  
The WPILib units library is a second iteration of adding units to the WPILib and I think they still need more work to get the units library easier for students to use and understand.   
Therefore we are going to ignore their units library until it has been around for a couple of years.

### subsystems
The subsystems folder simply contains all the subsystem classes that make up the robot. 

### robot
The robot folder contains the initial Main class and Robot.java. Robot.java has all the functionality that RobotContainer had before. 

### vendordeps
This folder contains all the vendor dependencies the robot needs. Such dependencies like Phoneix Lib, RevLib, ...

### Where are all the other files we are use to seeing?
The file .vscode/settings.json has been modified to hide all the other files I felt were not needed to be seen in the file list.

## Code Execution Intent
The class g.java contains most of the robot constants and class data variables.  
This approach allows other classes easy access to all variables in a organized fashion.  
Either a command or subsystem periodic will get or set a variable from g.java. This may not be the best object oriented approach but it does simplifiy the code files. 

## Schedular
The schedular is a background task the runs every 20ms. This 20ms task does the following during normal operation.  
1. Find all subsystem objects
2. Locate the defaultCommand associated with the subsystems
3. Execute the ```periodic()``` method of each subsystem.
4. Execute the ```execute()``` method of each default command
5. Execute the ```isFinished()``` method of each default command
6. If ```isFinished()``` returns true the ```end()``` function is called and the command is removed from the list of commads that are called by the schedular
  - Since the defaultCommand always returns ```false``` from the ```isFinished()``` method the defaultCommand will always be executed if not interuppted.

## Schedular during a command that interrupts the default command
Every command must have a "requirement" of a subsystem. The schedular must know what subsystem the command is going to act on.  
Therefore we the ```addRequirement()``` method and tell the schedular what subsystem the command needs.  
This is very common when we want a button to make a subsystem do something that is not part of what the defaultCommand does.  
We have two options for performing this functionality where a command will interrupt the defaultCommand and make the subsystem do something.
1. Create a command that requires the subsystem and schedule the command.
- The new scheduled command will run and it's ```execute()``` method with be called until it's ```isFinished()``` method returns true.
  - The commands ```execute()``` method will call the subsystems methods that make the motors move. 

2. Create a ```instantCommand``` that requires the subsystem and schedule the command.
- An instant command will have it's ```execute()``` method called only once and it will be removed from the schedule of commands to be called.
  - In the ```execute()``` method a global variable, usualling in ```g.java``` will be changed.
- The defaultCommand will take back over and use the new variable that was changed in ```g.java```. 
  - This is the prefered approach for functionality that the defaultCommand can normally handle, such as:  
    - The default command may always be making a subsystem go to a particular angle, height or distance based on a controller input.  
    If the controller is not changing the input, a button can ```trigger``` the ```instantCommand``` to change a variable that the defaultCommand is using.



