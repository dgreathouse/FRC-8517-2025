package frc.robot.defaultCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class DrivetrainDefaultCommand extends Command {
  SlewRateLimiter m_stickLimiterLX = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterLY = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterRX = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterRY = new SlewRateLimiter(3);

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand() {
    addRequirements(g.ROBOT.drive);
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
    rightYFiltered = m_stickLimiterRY.calculate(rightYFiltered);

    switch (g.DRIVETRAIN.driveMode) {
      case FIELD_CENTRIC:
        g.ROBOT.drive.driveFieldCentric( leftXFiltered, leftYFiltered, rightXFiltered, g.ROBOT.angleActual_deg);
        break;
      case ANGLE_FIELD_CENTRIC:
        g.ROBOT.drive.setAngleTarget(rightXFiltered, rightYFiltered);
        g.ROBOT.drive.driveAngleFieldCentric( leftXFiltered, leftYFiltered, g.ROBOT.angleActual_deg, g.ROBOT.angleTarget_deg);
        break;
      case POLAR_CENTRIC:
        // This mode is not used by the operator. It is intented for autonomous or teleOp commands. 
        g.ROBOT.drive.drivePolarFieldCentric(g.ROBOT.speedDriveTarget_mPsec, g.ROBOT.angleActual_deg, g.ROBOT.angleDriveTarget_deg, g.ROBOT.angleTarget_deg);
        break;
      case ROBOT_CENTRIC:
        g.ROBOT.drive.driveRobotCentric(leftXFiltered, leftYFiltered, rightXFiltered);
        break;
      case FAST_STOP:
        // g.ROBOT.drive.fastStop();
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
