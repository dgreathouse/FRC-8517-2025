// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.defaultCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;
import frc.robot.subsystems.Drivetrain;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivetrainDefaultCommand extends Command {
  private Drivetrain m_drive;
  SlewRateLimiter m_stickLimiterLX = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterLY = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterRX = new SlewRateLimiter(3);
  ChassisSpeeds m_speeds = new ChassisSpeeds();

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand() {
    addRequirements(g.ROBOT.drive);
    m_drive = g.ROBOT.drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // get thumbstick values
    double leftYRaw = -g.OI.driverController.getLeftX();
    double leftXRaw = -g.OI.driverController.getLeftY();
    double rightXRaw = -g.OI.driverController.getRightX();
    double rightYRaw = -g.OI.driverController.getRightY();

    // Limit the inputs for a deadband related to the joystick
    double leftYFiltered = MathUtil.applyDeadband(leftYRaw, 0.08, 1.0);
    double leftXFiltered = MathUtil.applyDeadband(leftXRaw, 0.08, 1.0);
    double rightXFiltered = MathUtil.applyDeadband(rightXRaw, 0.15, 1.0);
    double rightYFiltered = MathUtil.applyDeadband(rightYRaw, 0.15, 1.0);

    // Limit the speed of change to reduce the acceleration
    leftXFiltered = m_stickLimiterLX.calculate(leftXFiltered);
    leftYFiltered = m_stickLimiterLY.calculate(leftYFiltered);
    rightXFiltered = m_stickLimiterRX.calculate(rightXFiltered);
    rightYFiltered = m_stickLimiterRX.calculate(rightYFiltered);

    m_drive.setAngleTarget(rightXFiltered,rightYFiltered);
    switch (g.DRIVETRAIN.driveMode) {
      case FIELD_CENTRIC:
        m_drive.driveFieldCentric(leftXFiltered,leftYFiltered,rightXFiltered, g.ROBOT.angleActual_deg);
        break;
      case ANGLE_FIELD_CENTRIC:
        m_drive.driveAngleFieldCentric(leftXFiltered, leftYFiltered, g.ROBOT.angleActual_deg, g.ROBOT.angleTarget_deg);
        break;
      case POLAR_CENTRIC:
        // Do nothing in teleop since this is used in autonomous
        break;
      case ROBOT_CENTRIC:
        m_drive.driveRobotCentric(leftXFiltered,leftYFiltered,rightXFiltered);
        break;
      case FAST_STOP:
       // m_drive.fastStop();
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
