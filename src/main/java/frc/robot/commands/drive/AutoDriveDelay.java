// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;


public class AutoDriveDelay extends Command {
  Timer m_Timer;
  double m_timeOut_sec;
  public AutoDriveDelay(double _timeOut_sec) {
    addRequirements(g.ROBOT.drive);
    m_timeOut_sec = _timeOut_sec;
    m_Timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Timer.hasElapsed(m_timeOut_sec)){ return true;}
    return false;
  }
}
