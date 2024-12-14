// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autoCommands.AutoDoNothing;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.lib.DriveMode;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.g;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private Notifier m_telemetry = new Notifier(this::updateDashboard);

  private DrivetrainDefaultCommand m_drivetrainDefaultCommand = new DrivetrainDefaultCommand();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Set the default commands for the subsystems
    g.ROBOT.drive.setDefaultCommand(m_drivetrainDefaultCommand);

    // Configure/Map all the controller buttons to commands
    configureBindings();

    // Setup the autonomous play default and send to dashboard for selection
    m_autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    SmartDashboard.putData("Autonomouse Play", m_autoChooser);

    // Start telemetry in a slower rate than the main loop
    m_telemetry.startPeriodic(g.ROBOT.TELEMETRY_RATE_sec);
  }

  /**
   * This method is called by m_telemetry which is a Notifier. A Notifier will be executed at a
   * specified rate. The typical rate is 0.1 seconds which is slower that our base rate or 0.02
   * seconds.
   */
  private void updateDashboard() {
    for (IUpdateDashboard updates : g.DASHBOARD.updates) {
      updates.updateDashboard();
    }
  }

  /** */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_autoChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void configureBindings() {
    g.OI.DRIVER_RESET_YAW.onTrue(
        new InstantCommand(() -> g.ROBOT.drive.resetYaw(0.0), g.ROBOT.drive));
    g.OI.DRIVER_MODE_ANGLEFIELDCENTRIC.onTrue(
        new InstantCommand(
            () -> {
              g.DRIVETRAIN.driveMode = DriveMode.ANGLE_FIELD_CENTRIC;
            },
            g.ROBOT.drive));
    g.OI.DRIVER_MODE_FIELDCENTRIC.onTrue(
        new InstantCommand(
            () -> {
              g.DRIVETRAIN.driveMode = DriveMode.FIELD_CENTRIC;
            },
            g.ROBOT.drive));
    g.OI.DRIVER_MODE_ROBOTCENTRIC.onTrue(
        new InstantCommand(
            () -> {
              g.DRIVETRAIN.driveMode = DriveMode.ROBOT_CENTRIC;
            },
            g.ROBOT.drive));

    g.OI.DRIVER_MODE_SPEED_HI.onTrue(
        new InstantCommand(
            () -> {
              g.DRIVETRAIN.speedMultiplier = 1.0;
            },
            g.ROBOT.drive));
    g.OI.DRIVER_MODE_SPEED_LOW.onTrue(
        new InstantCommand(
            () -> {
              g.DRIVETRAIN.speedMultiplier = 0.5;
            },
            g.ROBOT.drive));

    g.OI.DRIVER_TOGGLE_DRIVETRAIN_ENABLE.onTrue(
        new InstantCommand(
            () -> {
              g.SWERVE.isEnabled = !g.SWERVE.isEnabled;
            },
            g.ROBOT.drive));
  }
}
