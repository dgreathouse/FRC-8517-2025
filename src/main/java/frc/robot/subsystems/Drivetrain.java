package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.SwerveModuleConstants;
import frc.robot.lib.g;

public class Drivetrain extends SubsystemBase implements IUpdateDashboard {
  SwerveDriveKinematics m_kinematics;
  SwerveDriveOdometry m_odometry;

  OdometryThread m_odometryThread;
  StatusSignal<Angle> m_yaw;
  StatusSignal<AngularVelocity> m_angularVelocityZ;

  private PIDController m_turnPID = new PIDController(g.DRIVETRAIN.TURN_KP, g.DRIVETRAIN.TURN_KI, g.DRIVETRAIN.TURN_KD);

  private ChassisSpeeds m_speeds = new ChassisSpeeds();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    initialize();
  }

  @SuppressWarnings("unused")
  private void initialize() {
    m_yaw = g.ROBOT.gyro.getYaw();
    m_yaw.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
    m_angularVelocityZ = g.ROBOT.gyro.getAngularVelocityZDevice();
    m_angularVelocityZ.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);

    g.SWERVE.modules[0] = new SwerveModule(new SwerveModuleConstants(
      "BR",
      12, true, 
      22, true, 
      2,0.4304,
      g.CHASSIS.BACK_RIGHT_SWERVE_X, g.CHASSIS.BACK_RIGHT_SWERVE_Y));
      g.SWERVE.modules[1] = new SwerveModule(new SwerveModuleConstants(
        "BL",
        13, false,
        23, true,
        3, -0.1567,
        g.CHASSIS.BACK_LEFT_SWERVE_X, g.CHASSIS.BACK_LEFT_SWERVE_Y));
    g.SWERVE.modules[2] = new SwerveModule(new SwerveModuleConstants(
        "F",
        11, true,
        21, true,
        1, 0.04785,
        g.CHASSIS.FRONT_SWERVE_X, g.CHASSIS.FRONT_SWERVE_Y));
    if (g.SWERVE.COUNT == 4) {
      g.SWERVE.modules[3] = new SwerveModule(new SwerveModuleConstants(
          "BL",
          13, false,
          23, false,
          4, 0,
          0, 0));
    }

    for(int i = 0; i < g.SWERVE.COUNT; i++){
      g.SWERVE.positions[i] = new SwerveModulePosition();
      g.DASHBOARD.updates.add(g.SWERVE.modules[i]);
    }
    updatePositions();

    m_kinematics = new SwerveDriveKinematics(g.SWERVE.modules[0].m_location, g.SWERVE.modules[1].m_location, g.SWERVE.modules[2].m_location);
    m_odometry = new SwerveDriveOdometry(m_kinematics, g.ROBOT.angleActual_rot2d, g.SWERVE.positions);

    m_turnPID.enableContinuousInput(-Math.PI, Math.PI);
    m_turnPID.setTolerance(Math.toRadians(.1), 1);

    m_odometryThread = new OdometryThread();
    m_odometryThread.start();

    g.DASHBOARD.updates.add(this);

    m_odometryThread = new OdometryThread();
    m_odometryThread.start();
  }
  public void updatePositions() {
    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      g.SWERVE.positions[i] = g.SWERVE.modules[i].updatePosition();
    }
  }
  public void driveRobotCentric(double _xSpeed, double _ySpeed, double _rotate){
    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.omegaRadiansPerSecond = _rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec;

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_speeds);
    setSwerveModules(states);

  }
  
  public void driveFieldCentric(double _xSpeed, double _ySpeed, double _rotate, double _robotAngle_deg) {
    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.omegaRadiansPerSecond = _rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec;
    
    m_speeds.toFieldRelativeSpeeds(new Rotation2d(_robotAngle_deg));
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_speeds);
    setSwerveModules(states);
  }


  public void driveAngleFieldCentric(double _xSpeed, double _ySpeed, double _robotAngle_deg, double _targetAngle_deg){
    m_speeds.vxMetersPerSecond = _xSpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;
    m_speeds.vyMetersPerSecond = _ySpeed * g.SWERVE.DRIVE.MAX_VELOCITY_mPsec;

    double rotate = m_turnPID.calculate(Math.toRadians(_robotAngle_deg),Math.toRadians(_targetAngle_deg));
    rotate = MathUtil.applyDeadband(rotate, 0.01);

    m_speeds.omegaRadiansPerSecond = rotate * g.SWERVE.DRIVE.MAX_ANGULAR_VELOCITY_radPsec;
    m_speeds.toFieldRelativeSpeeds(new Rotation2d(_robotAngle_deg));
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_speeds);
    setSwerveModules(states);
  }

  public void drivePolarFieldCentric(double _speed_mps, double _driveAngle_deg, double _targetAngle_deg) {
    double y = Math.sin(Units.degreesToRadians(_driveAngle_deg)) * _speed_mps;
    double x = Math.cos(Units.degreesToRadians(_driveAngle_deg)) * _speed_mps;
    driveAngleFieldCentric(x, y, g.ROBOT.angleActual_deg, _targetAngle_deg);
  }

  public void setSwerveModules(SwerveModuleState[] _states) {
    for (int i = 0; i < g.SWERVE.COUNT; i++) {
      g.SWERVE.modules[i].setDesiredState(_states[i]);
    }
  }

  public void setAngleTarget(double _x, double _y) {
    double x = _x; //-g.OI.driverController.getRightX();
    double y = _y; //-g.OI.driverController.getRightY();
    double hyp = Math.hypot(x, y);
    double joystickAngle = Math.toDegrees(Math.atan2(x, y));
    
    if (Math.abs(hyp) > g.OI.ANGLE_TARGET_DEADBAND) {
      if (joystickAngle >= -22.5 && joystickAngle <= 22.5) { // North
        g.ROBOT.angleTarget_deg = 0.0;
      } else if (joystickAngle >= -67.5 && joystickAngle < -22.5) { // North East
        g.ROBOT.angleTarget_deg = -45.0;
      } else if (joystickAngle >= -112.5 && joystickAngle < -67.5) { // East
        g.ROBOT.angleTarget_deg = -90.0;
      } else if (joystickAngle >= -157.5 && joystickAngle < -112.5) { // South East
        g.ROBOT.angleTarget_deg = -135.0;
      } else if ((joystickAngle >= 157.5 && joystickAngle <= 180.0) || (joystickAngle <= -157.5 && joystickAngle > -179.99)) { // South
        g.ROBOT.angleTarget_deg = 180.0;
      } else if (joystickAngle <= 67.5 && joystickAngle > 22.5) { // North West
        g.ROBOT.angleTarget_deg = 45.0;
      } else if (joystickAngle <= 112.5 && joystickAngle > 67.5) { // West
        g.ROBOT.angleTarget_deg = 90.0;
      } else if (joystickAngle <= 157.5 && joystickAngle > 112.5) { // South West
        g.ROBOT.angleTarget_deg = 135.0;
      }
    }
  }
  
  /**
   * Directly set the g.ROBOT.angleTarget_deg to a angle. Generally this is used
   * in autonomous or by a button.
   * 
   * @param _angle_deg The target angle the robot should PID to in
   *                   AngleFieldCentric Mode.
   */
  public void setAngleTarget(double _angle_deg) {
    g.ROBOT.angleTarget_deg = _angle_deg;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateDashboard() {
    g.SWERVE.totalSwerveCurrent_amps  = 0;
    for (SwerveModule swerveModule : g.SWERVE.modules) {
      g.SWERVE.totalSwerveCurrent_amps += Math.abs(swerveModule.getDriveCurrent()) + Math.abs(swerveModule.getSteerCurrent());
    }
    SmartDashboard.putNumber("Swerve/totalSwerveCurrent_amps", g.SWERVE.totalSwerveCurrent_amps);
    SmartDashboard.putData("Robot/Field2d", g.ROBOT.field2d);
    SmartDashboard.putNumber("Robot/angleTarget_deg", g.ROBOT.angleTarget_deg);
    SmartDashboard.putNumber("Robot/angleActual_deg", g.ROBOT.angleActual_deg);
    
  }

  public void resetYaw(double _angle) {
    g.ROBOT.gyro.setYaw(_angle);
  }
  /**
   * Is the robot rotational on target for g.ROBOT.AngleTarget_deg
   * 
   * @return if the rotation of the robot is on target
   */
  public boolean isRotateAtTarget() {
    return m_turnPID.atSetpoint();
  }

  private class OdometryThread extends Thread {
    public OdometryThread() {
      super();
    }

    @Override
    public void run() {

      while (true) {
        /* Now update odometry */
        for (int i = 0; i < g.SWERVE.COUNT; i++) {
          g.SWERVE.positions[i] = g.SWERVE.modules[i].updatePosition();
        }
        m_yaw = g.ROBOT.gyro.getYaw();
        m_angularVelocityZ = g.ROBOT.gyro.getAngularVelocityZDevice();
        g.ROBOT.angleActual_deg = StatusSignal.getLatencyCompensatedValueAsDouble(m_yaw, m_angularVelocityZ);
        g.ROBOT.angleActual_rot2d = Rotation2d.fromDegrees(g.ROBOT.angleActual_deg);
        g.ROBOT.pose2d = m_odometry.update(g.ROBOT.angleActual_rot2d, g.SWERVE.positions);
        g.ROBOT.pose3d = new Pose3d(g.ROBOT.pose2d);
        g.ROBOT.field2d.setRobotPose(g.ROBOT.pose2d);
        try {
          Thread.sleep(g.ROBOT.ODOMETRY_RATE_ms);
        } catch (InterruptedException e) {
          System.out.println(e.getMessage());
        }
      }
    }
  }
}
