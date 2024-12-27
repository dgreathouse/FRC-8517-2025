package frc.robot.lib;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;
import java.util.ArrayList;
import java.util.List;

/**
 * g.java contains all of our "static" java classes.
 *
 * <p>
 * The "static" java classses are defined for all needed subsystems and other
 * relevant java
 * classes like: ROBOT, OI/"OperatorInterface", and CAN IDs The "static" keyword
 * means these classes
 * and there static data are available to all other classes in the project and
 * they never need to
 * have a new instance made to access the internal static data.
 *
 * <p>
 * These classes should contain 2 types of data:
 *
 * <ul>
 * <li><b>Data</b> are values of some type that can change during the program
 * execution. Example:
 * <i>"double static volatile robotAngle_deg;" </i>
 * <li><b>Constants</b> are values of some type that CANNOT change during the
 * program execution.
 * Example: <i>"double static final LENGTH_mm = 320.0;"</i>
 * </ul>
 *
 * <p>
 * <b>Naming conventions</b>
 *
 * <p>
 * <b>Data</b> are labeled with camel casing and if units are appropriate the
 * word is appended
 * with a underscore followed by the abbreviated units as showin in units area.
 * <b>Constancs</b> are
 * labeled with capitol letters with an underscore seperating each name.If units
 * are appropriate the
 * word is appended with a underscore followed by the abbreviated units as shown
 * in units area.
 *
 * <p>
 * <b>Units</b>
 *
 * <p>
 * deg,rad,in,mm,m,sec,ft,rot degPsec, ftPsec, mPsec
 */
public class g {
  /** ROBOT class contains the static data associated with Robot.java
   *  The ROBOT represents the robot as a whole and not one subsystem.
   *  Therefore the ROBOT class holds data like the subsystem instances, gyro, field,
   *  and robot Pose. 
   * 
   */
  public static class ROBOT {
    public static Drivetrain drive = new Drivetrain();
    public static Pigeon2 gyro = new Pigeon2(g.CAN_IDS_CANIVORE.PIGEON2, g.CAN_IDS_CANIVORE.NAME);
    public static volatile double angleActual_deg;
    public static volatile Rotation2d angleActual_Rot2d = new Rotation2d();
    public static volatile double angleRobotTarget_deg;
    public static volatile double angleDriveTarget_deg;
    public static volatile double speedDriveTarget_mPsec;
    public static volatile Pose2d pose2d = new Pose2d();
    public static volatile Pose3d pose3d = new Pose3d();
    public static volatile Field2d field2d = new Field2d();

    public static final double TELEMETRY_RATE_sec = 0.02;
    public static final long ODOMETRY_RATE_ms = 5;

    public static final double MAX_BATTERY_SUPPLY_volts = 12.8;
  }
  /** This is one place to store all the devices that are on the RoboRIO CAN bus.
   *  The RoboRIO CAN bus runs at a rate slower that the CANIvore bus.
   * 
   */
  public static class CAN_IDS_ROBORIO {
    public static final String NAME = "rio";
  }
  /** This is one place to store all the devices that are on the CANIvore CAN bus
   *  The CANIvore is a high speed CAN-FD adapter that was made by CTRE.
   *  Since this is made by CTRE it can only manage device represented by CTRE, like:
   *  Pigeon2 gyro, TalonFX motor controllers, CANCoder encoder.
   */
  public static class CAN_IDS_CANIVORE {
    // 10-25 taken for drivetrain

    public static final String NAME = "CANivore";
    public static final int PIGEON2 = 5;
    public static final double UPDATE_FREQ_hz = 250;
  }
  /** CV contains the data for Conversion Variables. 
   * These are used in cases that a quick conversion is needed instead of getting the conversion is some other way.  */
  public static class CV {
    public static final double DEGREES_TO_RADIANS = 0.017453292519943295;
    public static final double RADIANS_TO_DEGREES = 57.29577951308232;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double MPS_TO_FEETPERSEC = 3.28084;
  }
  /** The OI class contains data associated with Operator Interfaces, such as PS5 Controllers */
  public static class OI {
    // Driver controller
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static CommandPS5Controller driverController = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);

    public static final double ANGLE_TARGET_DEADBAND = 0.8;
    public static final Trigger DRIVER_MODE_FIELDCENTRIC = driverController.povLeft();
    public static final Trigger DRIVER_MODE_ROBOTCENTRIC = driverController.povRight();
    public static final Trigger DRIVER_MODE_ANGLEFIELDCENTRIC = driverController.povDown();
    public static final Trigger DRIVER_MODE_ROTATEFIELDCENTRIC = driverController.povUp();

    public static final Trigger DRIVER_MODE_SPEED_HI = driverController.R1();
    public static final Trigger DRIVER_MODE_SPEED_LOW = driverController.L1();

    public static final Trigger DRIVER_RESET_YAW = driverController.create();

    public static final Trigger DRIVER_TOGGLE_DRIVETRAIN_ENABLE = driverController.touchpad();

    // Operator controller
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static CommandPS5Controller operatorController = new CommandPS5Controller(OPERATOR_CONTROLLER_PORT);
  }
  /** DASHBOARD store data for the smartdashboard. The smartdashboard that we use for 2025 is Elastic   */
  public static class DASHBOARD {
    public static List<IUpdateDashboard> updates = new ArrayList<>();
  }
  /** Swerve stores data for the Swerve drives. This class contains other classes like DRIVE and STEER */
  public static class SWERVE {
    /* CONTANTS */
    public static final int COUNT = 3;
    /* data */
    public static volatile double totalSwerveCurrent_amps = 0;
    public static volatile boolean isEnabled = true;
    public static final SwerveModule[] modules = new SwerveModule[COUNT];
    public static final SwerveModulePosition[] positions = new SwerveModulePosition[COUNT];
    /** The data for the SWERVE Drive motors */
    public static class DRIVE {

      private static final double MOTOR_PINION_TEETH = 12.0;
      private static final double GEAR_1_TEETH = 32.0;
      private static final double GEAR_2_DRIVE_TEETH = 28.0;
      private static final double GEAR_2_DRIVEN_TEETH = 18.0;
      private static final double GEAR_BEVEL_DRIVE_TEETH = 15.0;
      private static final double GEAR_BEVEL_DRIVEN_TEETH = 45.0;
      public static final double GEAR_RATIO = (GEAR_1_TEETH / MOTOR_PINION_TEETH)
          * (GEAR_2_DRIVEN_TEETH / GEAR_2_DRIVE_TEETH)
          * (GEAR_BEVEL_DRIVEN_TEETH / GEAR_BEVEL_DRIVE_TEETH);
      public static final double WHEEL_DIAMETER_m = .1015; // .10287
      private static final double WHEEL_CIRCUMFERENCE_m = Math.PI * WHEEL_DIAMETER_m;
      public static final double MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm = GEAR_RATIO / WHEEL_CIRCUMFERENCE_m;
      private static final double MOTOR_MAX_VELOCITY_rotPmin = 5800.0;
      private static final double MOTOR_MAX_VELOCITY_rotPsec = MOTOR_MAX_VELOCITY_rotPmin / 60.0;
      private static final double WHEEL_MAX_VELOCITY_rotPsec = MOTOR_MAX_VELOCITY_rotPsec / GEAR_RATIO;
      private static final double MOTOR_PEAK_EFFICIENCY_percent = 80;
      public static final double MAX_VELOCITY_mPsec = WHEEL_CIRCUMFERENCE_m
          * WHEEL_MAX_VELOCITY_rotPsec
          * MOTOR_PEAK_EFFICIENCY_percent
          / 100.0;
      public static final double MAX_ANGULAR_VELOCITY_radPsec = MAX_VELOCITY_mPsec * (1 / CHASSIS.WHEELBASE_mPrad);

      public static double PID_KP = 1.0;
      public static double PID_KI = 0.0;
      public static double PID_KV = g.ROBOT.MAX_BATTERY_SUPPLY_volts / MAX_VELOCITY_mPsec; // 2.8256;
      public static double PID_KS = 0.0;

      public static final double CURRENT_LIMIT_amps = 50;
    }
    /** The data for the SWERVE Steer motors */
    public static class STEER {
      public static double PID_KP = 0.1;
      public static double PID_KI = 0.0;

      private static final double MOTOR_PINION_TEETH = 8.0;
      private static final double MOTOR_DRIVE_GEAR_TEETH = 24.0;
      private static final double GEAR_1_DRIVE_TEETH = 14.0;
      private static final double GEAR_1_DRIVEN_TEETH = 72.0;
      private static final double CANCODER_GEAR_RATIO = 1.0;
      public static final double GEAR_RATIO = 1
          / ((MOTOR_PINION_TEETH / MOTOR_DRIVE_GEAR_TEETH)
              * (GEAR_1_DRIVE_TEETH / GEAR_1_DRIVEN_TEETH));
      public static final double GEAR_RATIO_TO_CANCODER = GEAR_RATIO * CANCODER_GEAR_RATIO;
      public static final double CURRENT_LIMIT_amps = 50;
    }
  }

  public static class CHASSIS {
    public static final double FRONT_SWERVE_X_POSITION_m = 352.44 / 1000.0;
    public static final double FRONT_SWERVE_Y_POSITION_m = 0.0;
    public static final double BACK_LEFT_SWERVE_X_POSITION_m = -255.18 / 1000.0;
    public static final double BACK_LEFT_SWERVE_Y_POSITION_m = 243.1 / 1000.0;
    public static final double BACK_RIGHT_SWERVE_X_POSITION_m = -255.18 / 1000.0;
    public static final double BACK_RIGHT_SWERVE_Y_POSITION_m = -243.1 / 1000.0;
    public static final double WHEELBASE_DIAMETER_m = 0.705;
    private static final double WHEELBASE_CIRCUMFERENCE_m = Math.PI * WHEELBASE_DIAMETER_m;
    private static final double WHEELBASE_mPrad = WHEELBASE_CIRCUMFERENCE_m / (2 * Math.PI);
  }

  public static class DRIVETRAIN {
    public static final double TURN_KP = 4.5;
    public static final double TURN_KI = 1.50;
    public static final double TURN_KD = 0.0;
    public static final double TURN_DEADBAND = 0.02;
    public static final double AUTO_DRIVE_POSE_DISTANCE_TOLERANCE_m = 0.125; // 5 inches
    public static final double AUTO_DRIVE_POSE_ANGLE_TOLERANCE_deg = 1.0; 
    public static volatile DriveMode driveMode = DriveMode.ANGLE_FIELD_CENTRIC;
    public static volatile double speedMultiplier = 1.0;
  }

  public static class SIM {
    public static boolean IS_GYRO_DISABLED = false;
  }
}
