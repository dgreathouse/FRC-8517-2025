// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoDriveToPose;
import frc.robot.commands.drive.AutoRotateToAngle;


public class AutoDriveRotateTest extends SequentialCommandGroup {
  /** Creates a new AutoDriveRotateTest. */
  public AutoDriveRotateTest() {

    addCommands(
      new AutoDriveToPose(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), 0.5, 5),
      new AutoRotateToAngle(90, 5)
    );
  }
}
