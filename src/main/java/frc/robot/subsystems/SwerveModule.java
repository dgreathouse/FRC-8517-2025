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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.SimpleMotorFeedforward_DG;
import frc.robot.lib.g;

/** Add your docs here. */
public class SwerveModule implements IUpdateDashboard {
    // SwerveModuleConstants m_k;
    public String m_name = "";
    public Translation2d m_location;
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_canCoder;
    private SwerveModulePosition m_position = new SwerveModulePosition();
    private PIDController m_steerPID = new PIDController(g.SWERVE.STEER.PID_KP, g.SWERVE.STEER.PID_KI, 0);
    private PIDController m_drivePID = new PIDController(g.SWERVE.DRIVE.PID_KP, g.SWERVE.DRIVE.PID_KI, 0);
    private SimpleMotorFeedforward_DG m_driveFF = new SimpleMotorFeedforward_DG(g.SWERVE.DRIVE.PID_KS, g.SWERVE.DRIVE.PID_KV,0.0);
    private VoltageOut m_steerVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private VoltageOut m_driveVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private StatusSignal<Angle> m_drivePosition;
    private StatusSignal<AngularVelocity> m_driveVelocity;
    private StatusSignal<Angle> m_steerPosition;
    private StatusSignal<AngularVelocity> m_steerVelocity;

    public SwerveModule(String _name,
            int _driveCanId, boolean _driveIsReversed,
            int _steerCanId, boolean _steerIsReversed,
            int _canCoderId, double _canCoderOffset_rot,
            double _locationX_m, double _locationY_m) {
        StatusCode status;
        m_name = _name;
        m_location = new Translation2d(_locationX_m, _locationY_m);
        m_driveMotor = new TalonFX(_driveCanId, g.CAN_IDS_CANIVORE.NAME);
        m_steerMotor = new TalonFX(_steerCanId, g.CAN_IDS_CANIVORE.NAME);
        m_canCoder = new CANcoder(_canCoderId, g.CAN_IDS_CANIVORE.NAME);

        // Configure Drive Motor
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.MotorOutput.Inverted = _driveIsReversed ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // FIXME: driveConfigs.withOpenLoopRamps(new
        // OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.1));
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);

        status = m_driveMotor.getConfigurator().apply(driveConfigs);
        System.out.println(m_name + " Drive Motor TalonFX Config Status =" + status.toString());

        CurrentLimitsConfigs driveCurrentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(g.SWERVE.DRIVE.CURRENT_LIMIT_amps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(g.SWERVE.DRIVE.CURRENT_LIMIT_amps)
                .withSupplyCurrentLimitEnable(true);

        status = m_driveMotor.getConfigurator().apply(driveCurrentConfig);
        System.out.println(m_name + " Drive Motor Current Config Status =" + status.toString());

        m_drivePosition = m_driveMotor.getPosition();
        m_drivePosition.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_hz);
        m_driveVelocity = m_driveMotor.getVelocity();
        m_driveVelocity.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_hz);
        // Configure Steer Motor
        m_steerPID.enableContinuousInput(-180.0, 180.0);
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();
        steerConfigs.MotorOutput.Inverted = _steerIsReversed ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        // steerConfigs.withOpenLoopRamps(new
        // OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(1));
        status = m_steerMotor.getConfigurator().apply(steerConfigs);
        System.out.println(m_name + " Steer Motor TalonFX Config Status =" + status.toString());
        m_steerMotor.setNeutralMode(NeutralModeValue.Brake);

        CurrentLimitsConfigs steerCurrentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(g.SWERVE.STEER.CURRENT_LIMIT_amps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(g.SWERVE.STEER.CURRENT_LIMIT_amps)
                .withSupplyCurrentLimitEnable(true);

        status = m_steerMotor.getConfigurator().apply(steerCurrentConfig);
        System.out.println(m_name + " Steer Motor Current Config Status =" + status.toString());

        // Configure CANCoder
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = _canCoderOffset_rot;

        status = m_canCoder.getConfigurator().apply(cancoderConfigs);
        System.out.println(m_name + " m_canCoder Config Status =" + status.toString());
        // Set the offset position of the steer motor based on the CANCoder
        m_steerMotor.setPosition(m_canCoder.getPosition().getValueAsDouble() * g.SWERVE.STEER.GEAR_RATIO);

        m_steerPosition = m_steerMotor.getPosition();
        m_steerPosition.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_hz);
        m_steerVelocity = m_steerMotor.getVelocity();
        m_steerVelocity.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_hz);
    }

    /**
     * Called by separate thread to put stuff to the dashboard at a slower rate than the main periodic
     */
    public void updateDashboard() {
        SmartDashboard.putNumber("Swerve/" + this.m_name + "/Vel(ftPsec)",
                (m_driveMotor.getVelocity().getValueAsDouble() / // Rot/sec /
                        g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm) // Rot/m
                        * g.CV.MPS_TO_FEETPERSEC); // ft/s
        SmartDashboard.putNumber("Swerve/" + this.m_name + "/Drive Current", getDriveCurrent());
        SmartDashboard.putNumber("Swerve/" + this.m_name + "/Steer Current", getSteerCurrent());
        SmartDashboard.putData("Swerve/" + this.m_name + "/Steer PID", m_steerPID);
        SmartDashboard.putData("Swerve/" + this.m_name + "/Drive PID", m_drivePID);
    }

    public SwerveModulePosition updatePosition() {
        double drive_rot = m_driveMotor.getPosition().getValueAsDouble();
        double angle_rot = m_steerMotor.getPosition().getValueAsDouble();
        // anagle_rot is the Motor rotations. Apply the gear ratio to get wheel
        // rotations for steer
        angle_rot = angle_rot / g.SWERVE.STEER.GEAR_RATIO;
        /* And push them into a SwerveModuleState object to return */// WHEEL_MotRotPerMeter
        m_position.distanceMeters = drive_rot / g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm;
        /* Angle is already in terms of steer rotations */
        m_position.angle = Rotation2d.fromRotations(angle_rot);

        return m_position;
    }

    public double getSteerActualAngle() {
        return m_position.angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState _state) {
        _state.optimize(m_position.angle);
        /*-------------------- Steer---------------------*/
        if (g.SWERVE.isEnabled) {
            double steerVolts = m_steerPID.calculate(getSteerActualAngle(), _state.angle.getDegrees());
            m_steerMotor.setControl(m_steerVoltageOut.withOutput(steerVolts));
        } else {
            m_steerMotor.setControl(m_steerVoltageOut.withOutput(0));
        }
        /*-------------------- Drive---------------------*/
        if (g.SWERVE.isEnabled) {

            double driveSetVelocity_mps = _state.speedMetersPerSecond * g.DRIVETRAIN.speedMultiplier;
            double driveVolts = m_drivePID.calculate(m_driveMotor.getVelocity().getValueAsDouble() / g.SWERVE.DRIVE.MOTOR_ROTATIONS_TO_WHEEL_DISTANCE_rotPm, driveSetVelocity_mps);
            driveVolts = MathUtil.clamp(driveVolts, -6, 6);
            
            driveVolts = driveVolts + m_driveFF.calculate(driveSetVelocity_mps,0.0);
            m_driveMotor.setControl(m_driveVoltageOut.withOutput(driveVolts));
        } else {
            m_driveMotor.setControl(m_driveVoltageOut.withOutput(0));
        }
    }

    public double getDriveCurrent() {
        return m_driveMotor.getTorqueCurrent().getValueAsDouble();
    }

    public double getSteerCurrent() {
        return m_steerMotor.getTorqueCurrent().getValueAsDouble();
    }

}
