package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class AutoDriveToAprilTag extends Command {
  // TODO Complete this once the final release of PhotonVision is done in 2025.
  /** Drive to an AprilTag at a distance away from the tag.
   * If the tag is not in view the command will end.
   * 
   * @param _aprilTagID The ID number of the desired AprilTag
   * @param _distance_m The distance to drive to
   */
  public AutoDriveToAprilTag(int _aprilTagID, double _distance_m) {
    addRequirements(g.ROBOT.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
