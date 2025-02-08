// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivebase;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveBaseConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final SparkMax driveMotor;
  private final SparkMax turningMotor;
  private final CANcoder turningEncoder;
  private final RelativeEncoder driveEncoder;
  public final PIDController rotController;
  private final String name;
  private final Boolean turningInverted;
  private final Boolean driveInverted;
  private double turningMotorVoltage;
  private double driveMotorVoltage;

  public SwerveModule(int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel, boolean driveInverted, boolean turningInverted, double canCoderMagOffset,
      String name) {
    this.name = name;
    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    turningEncoder = new CANcoder(turningEncoderChannel);
    CANcoderConfiguration turningEncoderConfiguration = new CANcoderConfiguration();
    turningEncoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);
    turningEncoderConfiguration.MagnetSensor.MagnetOffset = canCoderMagOffset;
    turningEncoder.getConfigurator().apply(turningEncoderConfiguration);
    driveEncoder = driveMotor.getEncoder();
    rotController = new PIDController(ModuleConstants.kPRotationController, ModuleConstants.kIRotationController,
        ModuleConstants.kDRotationController);
    rotController.enableContinuousInput(-180.0, 180.0);
    this.driveInverted = driveInverted;
    this.turningInverted = turningInverted;
    driveMotorVoltage = 0;
    turningMotorVoltage = 0;
    SmartDashboard.putNumber(name + "_ModuleDriveMotorVoltage", driveMotorVoltage);
    SmartDashboard.putNumber(name + "_ModuleTurningMotorVoltage", turningMotorVoltage);
    init();
  }

  public void init() {
    configDriveMotor();
    configTurningMotor();
    resetAllEncoder();
  }

  public void configDriveMotor() {
    SparkMaxConfig configDriveMotor = new SparkMaxConfig();
    configDriveMotor.idleMode(IdleMode.kBrake);
    configDriveMotor.smartCurrentLimit(10, 80);
    configDriveMotor.closedLoopRampRate(ModuleConstants.kDriveClosedLoopRampRate);
    configDriveMotor.voltageCompensation(ModuleConstants.kMaxModuleDriveVoltage);
    configDriveMotor.signals.primaryEncoderPositionPeriodMs(10);
    configDriveMotor.signals.primaryEncoderVelocityPeriodMs(20);
    configDriveMotor.inverted(driveInverted);
    driveMotor.configure(configDriveMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void configTurningMotor() {
    SparkBaseConfig configTurningMotor = new SparkMaxConfig();
    configTurningMotor.smartCurrentLimit(20);
    configTurningMotor.closedLoopRampRate(ModuleConstants.kTurningClosedLoopRampRate);
    configTurningMotor.idleMode(IdleMode.kBrake);
    configTurningMotor.voltageCompensation(ModuleConstants.kMaxModuleTurningVoltage);
    configTurningMotor.inverted(turningInverted);
    turningMotor.configure(configTurningMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void resetAllEncoder() {
    driveEncoder.setPosition(0);
  }

  public void stopModule() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // to get the single swerveModule speed and the turning rate
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveRate(), new Rotation2d(Math.toRadians(getRotation())));
  }

  // to get the drive distance
  public double getDriveDistance() {
    return (driveEncoder.getPosition() / 6.75) * (2.0 * Math.PI * ModuleConstants.kWheelRadius);

  }

  // calculate the rate of the drive
  public double getDriveRate() {
    return driveEncoder.getVelocity() * 1 / 60.0 / 6.75 * 2.0 * Math.PI
        * ModuleConstants.kWheelRadius;
  }

  // to get rotation of turning motor
  public double getRotation() {
    return turningEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  // to the get the position by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(Math.toRadians(getRotation())));
  }

  public double[] optimizeOutputVoltage(SwerveModuleState goalState, double currentTurningDegree) {
    SwerveModuleState desiredState = new SwerveModuleState(
        goalState.speedMetersPerSecond, goalState.angle);
    desiredState.optimize(new Rotation2d(Math.toRadians(currentTurningDegree)));
    driveMotorVoltage = desiredState.speedMetersPerSecond * ModuleConstants.kDesireSpeedtoMotorVoltage;
    turningMotorVoltage = rotController.calculate(currentTurningDegree, desiredState.angle.getDegrees());
    return new double[] { driveMotorVoltage, turningMotorVoltage };
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < DriveBaseConstants.kMinSpeed) {
      stopModule();
    } else {
      var moduleState = optimizeOutputVoltage(desiredState, getRotation());
      driveMotor.setVoltage(moduleState[0]);
      turningMotor.setVoltage(moduleState[1]);
    }
  }

  public void resetTurningDegree() {
    turningMotorVoltage = rotController.calculate(getRotation(), 0);
    turningMotor.setVoltage(turningMotorVoltage);
  }

  public void setTurningDegree90() {
    turningMotorVoltage = rotController.calculate(getRotation(), 90);
    turningMotor.setVoltage(turningMotorVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(name + "_ModuleDistance", getDriveDistance());
    SmartDashboard.putNumber(name + "_ModuleVelocity", getDriveRate());
    SmartDashboard.putNumber(name + "_ModuleRotation", getRotation());
    SmartDashboard.putNumber(name + "_ModuleDriveMotorVoltage", driveMotorVoltage);
    SmartDashboard.putNumber(name + "_ModuleTurningMotorVoltage", turningMotorVoltage);
    SmartDashboard.putData(name + "_rotController", rotController);
  }
}