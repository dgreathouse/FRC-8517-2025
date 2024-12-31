package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class AutoRotateToAngle extends Command {
  double m_angleDesired_deg;
  double m_timeOut_sec;
  Timer m_timer;

  /** Rotate to an angle. If the timeOut_sec is reached then exit this command early.
   * The timeout should never be reached and it should be set to be slightly greater than how long it takes to get to the angle.
   * Timeout is a safe gaurd if the robot can not get to the angle. Such as it is up against a wall and can not rotate.
   * 
   * @param _angle_deg
   * @param _timeOut_sec
   */
  public AutoRotateToAngle(double _angle_deg, double _timeOut_sec) {
    addRequirements(g.ROBOT.drive);
    m_angleDesired_deg = _angle_deg;
    m_timeOut_sec = _timeOut_sec;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    g.ROBOT.angleTarget_deg = m_angleDesired_deg;
    m_timer = new Timer();
    m_timer.restart();
  }
  // TODO: Test this
  //   [ ] Time out not being activated
  //   [ ] Turn goes both directions
  @Override
  public void execute() {
    g.ROBOT.drive.driveAngleFieldCentric(0, 0, g.ROBOT.angleActual_deg, m_angleDesired_deg);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean timeOut = m_timer.hasElapsed(m_timeOut_sec);
    if(timeOut){ SmartDashboard.putBoolean("Robot/AutoRotateTimeOut", true);}
    if(g.ROBOT.drive.isRotateAtTarget() || timeOut) {
      return true;
    }
    return false;
  }
}
