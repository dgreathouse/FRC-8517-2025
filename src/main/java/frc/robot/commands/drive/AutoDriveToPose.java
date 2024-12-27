package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;


public class AutoDriveToPose extends Command {
  Pose2d m_desiredPose;
  double m_speed;
  double m_timeOut_sec;
  double m_rampuUpTime_sec = 0.5;
  PIDController m_drivePID = new PIDController(1, 0, 0);
  Timer m_timer = new Timer();
  /** Drive to a pose on the field. Pose must be relative to starting pose or the starting pose must be set based on field pose.
   * 
   * @param _pose The Pose to drive to
   * @param _speed The max speed +/- 1.0 to drive at
   * @param _timeOut_sec The time to end if pose not reached
   */
  public AutoDriveToPose(Pose2d _pose, double _speed, double _timeOut_sec) {
    addRequirements(g.ROBOT.drive);
    m_desiredPose = _pose;
    m_speed = _speed;
    m_timeOut_sec = _timeOut_sec;
    m_drivePID.setTolerance(.1);
    m_drivePID.setIZone(0.5);
    m_drivePID.setIntegratorRange(-m_speed/2, m_speed/2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_timer.restart();

  }

  // TODO: Test this class. Possible issues.
  //  [ ] Starting Pose
  //  [ ]Tolerance
  //  [ ] PIDs

  @Override
  public void execute() {
    Pose2d trajectory = m_desiredPose.relativeTo(g.ROBOT.pose2d);
    double driveAngle_deg = trajectory.getTranslation().getAngle().getDegrees();
    double driveDistance_m = trajectory.getTranslation().getDistance(g.ROBOT.pose2d.getTranslation());
    double speed =  m_drivePID.calculate(0,driveDistance_m);
    speed = rampUpValue(speed, m_rampuUpTime_sec);
    speed = MathUtil.clamp(speed, -m_speed, m_speed);
    g.ROBOT.drive.drivePolarFieldCentric(speed, g.ROBOT.angleActual_deg, driveAngle_deg, m_desiredPose.getRotation().getDegrees());

  }
  private double rampUpValue(double _val, double rampTime_sec){   
    double currentTime_sec = m_timer.get();                                        
    if(currentTime_sec < rampTime_sec){
      _val = _val * currentTime_sec / rampTime_sec;
    }
    return _val;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(
      Math.abs((g.ROBOT.pose2d.getX()) - m_desiredPose.getX()) < g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m
      && Math.abs((g.ROBOT.pose2d.getY()) - m_desiredPose.getY()) < g.DRIVETRAIN.AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m
      && g.ROBOT.drive.isRotateAtTarget())
    {
      return true;
    }
    if(m_timer.hasElapsed(m_timeOut_sec)){
      return true;
    }
    return false;
  }
}
