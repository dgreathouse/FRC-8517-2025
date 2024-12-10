// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.SwerveModuleConstants;
import frc.robot.lib.g;

/** Add your docs here. */
public class SwerveModule implements IUpdateDashboard{
    SwerveModuleConstants m_k;
    public Translation2d m_location;
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_canCoder;
    private SwerveModulePosition m_position = new SwerveModulePosition();
    private PIDController m_steerPID = new PIDController(g.SWERVE.STEER.PID_KP, g.SWERVE.STEER.PID_KI, 0);
    private PIDController m_drivePID = new PIDController(g.SWERVE.DRIVE.PID_KP, g.SWERVE.DRIVE.PID_KI, 0);
    private SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(g.SWERVE.DRIVE.PID_KS, g.SWERVE.DRIVE.PID_KV);
    private VoltageOut m_steerVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private VoltageOut m_driveVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private StatusSignal<Angle> m_drivePosition;
    private StatusSignal<AngularVelocity> m_driveVelocity;
    private StatusSignal<Angle> m_steerPosition;
    private StatusSignal<AngularVelocity> m_steerVelocity;
    public SwerveModule(SwerveModuleConstants _k) {
        StatusCode status;
        m_k = _k;
        m_location = new Translation2d(m_k.LOCATION_X_METER, m_k.LOCATION_Y_METER);
        m_driveMotor = new TalonFX(m_k.DRIVE_CAN_ID, g.CAN_IDS_CANIVORE.NAME);
        m_steerMotor = new TalonFX(m_k.STEER_CAN_ID, g.CAN_IDS_CANIVORE.NAME);
        m_canCoder = new CANcoder(m_k.CANCODER_ID, g.CAN_IDS_CANIVORE.NAME);

        // Configure Drive Motor
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.MotorOutput.Inverted = m_k.DRIVE_IS_REVERSED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
                
        // FIXME: driveConfigs.withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.1));
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);


        status = m_driveMotor.getConfigurator().apply(driveConfigs);
        System.out.println(m_k.NAME + " Drive Motor TalonFX Config Status =" + status.toString());

        CurrentLimitsConfigs driveCurrentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(g.SWERVE.DRIVE.CURRENT_LIMIT_amps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(g.SWERVE.DRIVE.CURRENT_LIMIT_amps)
                .withSupplyCurrentLimitEnable(true);

        status = m_driveMotor.getConfigurator().apply(driveCurrentConfig);
        System.out.println(m_k.NAME + " Drive Motor Current Config Status =" + status.toString());

        m_drivePosition = m_driveMotor.getPosition();
        m_drivePosition.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
        m_driveVelocity = m_driveMotor.getVelocity();
        m_driveVelocity.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
        // Configure Steer Motor
        m_steerPID.enableContinuousInput(-180.0, 180.0);
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();
        steerConfigs.MotorOutput.Inverted = m_k.STEER_IS_REVERSED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        //steerConfigs.withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(1));
        status = m_steerMotor.getConfigurator().apply(steerConfigs);
        System.out.println(m_k.NAME + " Steer Motor TalonFX Config Status =" + status.toString());
        m_steerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        CurrentLimitsConfigs steerCurrentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(g.SWERVE.STEER.CURRENT_LIMIT_amps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(g.SWERVE.STEER.CURRENT_LIMIT_amps)
                .withSupplyCurrentLimitEnable(true);

        status = m_steerMotor.getConfigurator().apply(steerCurrentConfig);
        System.out.println(m_k.NAME + " Steer Motor Current Config Status =" + status.toString());

        // Configure CANCoder
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = m_k.CANCODER_OFFSET_ROT;
        
        status = m_canCoder.getConfigurator().apply(cancoderConfigs);
        System.out.println(m_k.NAME + " m_canCoder Config Status =" + status.toString());
        // Set the offset position of the steer motor based on the CANCoder
        m_steerMotor.setPosition(m_canCoder.getPosition().getValueAsDouble() * g.SWERVE.STEER.GEAR_RATIO);

        m_steerPosition = m_steerMotor.getPosition();
        m_steerPosition.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
        m_steerVelocity = m_steerMotor.getVelocity();
        m_steerVelocity.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);

    }

    public SwerveModulePosition updatePosition() {
        double drive_rot = m_driveMotor.getPosition().getValueAsDouble();
        double angle_rot = m_steerMotor.getPosition().getValueAsDouble();
        // anagle_rot is the Motor rotations. Apply the gear ratio to get wheel rotations for steer
        angle_rot = angle_rot / g.SWERVE.STEER.GEAR_RATIO;
        /* And push them into a SwerveModuleState object to return *///WHEEL_MotRotPerMeter
        m_position.distanceMeters = drive_rot / g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm;
        /* Angle is already in terms of steer rotations */
        m_position.angle = Rotation2d.fromRotations(angle_rot);

        return m_position;
    }

    public double getSteerActualAngle() {
        return m_position.angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState _state) {
        _state.optimize(m_position.angle);//SwerveModuleState.optimize(_state, m_position.angle);
        /* ------------------------------------- Steer --------------------------------------------------- */
        if (g.SWERVE.isEnabled) {
            double steerVolts = m_steerPID.calculate(getSteerActualAngle(), _state.angle.getDegrees());
            m_steerMotor.setControl(m_steerVoltageOut.withOutput(steerVolts));
        } else {
            m_steerMotor.setControl(m_steerVoltageOut.withOutput(0));
        }
        /* ------------------------------------- Drive --------------------------------------------------- */
        if (g.SWERVE.isEnabled) {

            double driveSetVelocity_mps = _state.speedMetersPerSecond * g.DRIVETRAIN.speedMultiplier;//optimized.speedMetersPerSecond * g.DRIVETRAIN.driveSpeedMultiplier;
            double driveVolts = m_drivePID.calculate(m_driveMotor.getVelocity().getValueAsDouble() / g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm, driveSetVelocity_mps);
            driveVolts = MathUtil.clamp(driveVolts, -6, 6);
            driveVolts = driveVolts + m_driveFF.calculate(driveSetVelocity_mps);

            m_driveMotor.setControl(m_driveVoltageOut.withOutput(driveVolts));
        } else {
            m_driveMotor.setControl(m_driveVoltageOut.withOutput(0));
        }
    }
    public double getDriveCurrent(){
        return m_driveMotor.getTorqueCurrent().getValueAsDouble();
    }
    public double getSteerCurrent(){
        return m_steerMotor.getTorqueCurrent().getValueAsDouble();
    }
  /**
   * Called by separate thread to put stuff to the dashboard at a slower rate than the main periodic
   */
  public void updateDashboard() {
    SmartDashboard.putNumber("Swerve/"+this.m_k.NAME+"/Vel(ftPsec)", 
                            (m_driveMotor.getVelocity().getValueAsDouble() /   // Rot/sec /
                            g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm)        // Rot/m
                            * g.CV.MPS_TO_FEETPERSEC);                         // ft/s
    SmartDashboard.putNumber("Swerve/"+this.m_k.NAME+"/Drive Current", getDriveCurrent());
    SmartDashboard.putNumber("Swerve/"+this.m_k.NAME+"/Steer Current", getSteerCurrent());
    SmartDashboard.putData("Swerve/"+this.m_k.NAME+"/Steer PID", m_steerPID);
    SmartDashboard.putData("Swerve/"+this.m_k.NAME+"/Drive PID", m_drivePID);
  }
}