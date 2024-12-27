package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class AutoRotateToAngle extends Command {
  double m_angleDesired_deg;
  double m_timeOut_sec;
  Timer m_timer;

  /** Creates a new AutoRotateRobot. */
  public AutoRotateToAngle(double _angle_deg, double _timeOut_sec) {
    addRequirements(g.ROBOT.drive);
    m_angleDesired_deg = _angle_deg;
    m_timeOut_sec = _timeOut_sec;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    g.ROBOT.angleRobotTarget_deg = m_angleDesired_deg;
    m_timer = new Timer();
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // TODO: Test this
  //   [ ] Time out not being activated
  //   [ ] Turn goes to both directions
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
    if(g.ROBOT.drive.isRotateAtTarget() || m_timer.hasElapsed(m_timeOut_sec)) {
      return true;
    }
    return false;
  }
}
