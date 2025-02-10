// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivebase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
  private double turningMotorVoltage;
  private double driveMotorVoltage;

  public SwerveModule(int driveMotorChannel,
      int turningMotorChannel, int turningEncoderChannel,
      boolean driveInverted, boolean turningInverted,
      double canCoderMagOffset, String name) {
    this.name = name;
    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    turningEncoder = new CANcoder(turningEncoderChannel);

    CANcoderConfiguration turningEncoderConfiguration = new CANcoderConfiguration();
    turningEncoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);
    turningEncoderConfiguration.MagnetSensor.MagnetOffset = canCoderMagOffset;
    turningEncoder.getConfigurator().apply(turningEncoderConfiguration);

    driveEncoder = driveMotor.getEncoder();

    rotController = new PIDController(
        ModuleConstants.kPRotationController,
        ModuleConstants.kIRotationController,
        ModuleConstants.kDRotationController);
    rotController.enableContinuousInput(-180.0, 180.0);

    driveMotorVoltage = 0;
    turningMotorVoltage = 0;
    SmartDashboard.putNumber(name + "_ModuleDriveMotorVoltage", driveMotorVoltage);
    SmartDashboard.putNumber(name + "_ModuleTurningMotorVoltage", turningMotorVoltage);

    SparkMaxConfig configDriveMotor = new SparkMaxConfig();
    configDriveMotor.idleMode(IdleMode.kBrake);
    configDriveMotor.smartCurrentLimit(10, 80);
    configDriveMotor.closedLoopRampRate(ModuleConstants.kDriveClosedLoopRampRate);
    configDriveMotor.voltageCompensation(ModuleConstants.kMaxModuleDriveVoltage);
    configDriveMotor.signals.primaryEncoderPositionPeriodMs(10);
    configDriveMotor.signals.primaryEncoderVelocityPeriodMs(20);
    configDriveMotor.inverted(driveInverted);
    driveMotor.configure(
        configDriveMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkBaseConfig configTurningMotor = new SparkMaxConfig();
    configTurningMotor.smartCurrentLimit(20);
    configTurningMotor.closedLoopRampRate(ModuleConstants.kTurningClosedLoopRampRate);
    configTurningMotor.idleMode(IdleMode.kBrake);
    configTurningMotor.voltageCompensation(ModuleConstants.kMaxModuleTurningVoltage);
    configTurningMotor.inverted(turningInverted);
    turningMotor.configure(
        configTurningMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    resetAllEncoder();
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
        getDriveRate(), getRotation2d());
  }

  // to get the drive distance
  public double getDriveDistance() {
    return (driveEncoder.getPosition() / 6.75)
        * (2.0 * Math.PI * ModuleConstants.kWheelRadius.in(Meters));

  }

  // calculate the rate of the drive
  public double getDriveRate() {
    return driveEncoder.getVelocity() * 1 / 60.0 / 6.75 * 2.0 * Math.PI
        * ModuleConstants.kWheelRadius.in(Meters);
  }

  // to get rotation of turning motor
  public Rotation2d getRotation2d() {
    return new Rotation2d(
        Math.toRadians(
            turningEncoder.getAbsolutePosition().getValueAsDouble() * 360.0));
  }

  // to the get the position by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), getRotation2d());
  }

  public double[] optimizeOutputVoltage(SwerveModuleState goalState, Rotation2d currentRotation2d) {
    SwerveModuleState desiredState = new SwerveModuleState(
        goalState.speedMetersPerSecond, goalState.angle);
    desiredState.optimize(currentRotation2d);
    driveMotorVoltage = desiredState.speedMetersPerSecond
        * ModuleConstants.kDesireSpeedToMotorVoltage;
    turningMotorVoltage = rotController.calculate(
        currentRotation2d.getDegrees(), desiredState.angle.getDegrees());
    return new double[] { driveMotorVoltage, turningMotorVoltage };
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    //CHECKSTYLE.SUPPRESS\: LineLength
    if (Math.abs(desiredState.speedMetersPerSecond) < DriveBaseConstants.kMinSpeed.in(MetersPerSecond)) {
      stopModule();
    } else {
      var moduleState = optimizeOutputVoltage(desiredState, getRotation2d());
      driveMotor.setVoltage(moduleState[0]);
      turningMotor.setVoltage(moduleState[1]);
    }
  }

  public void setTurningDegree(double degree) {
    turningMotorVoltage = rotController.calculate(getRotation2d().getDegrees(), degree);
    turningMotor.setVoltage(turningMotorVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(name + "_ModuleDistance", getDriveDistance());
    SmartDashboard.putNumber(name + "_ModuleVelocity", getDriveRate());
    SmartDashboard.putNumber(name + "_ModuleRotation", getRotation2d().getDegrees());
    SmartDashboard.putNumber(name + "_ModuleDriveMotorVoltage", driveMotorVoltage);
    SmartDashboard.putNumber(name + "_ModuleTurningMotorVoltage", turningMotorVoltage);
    SmartDashboard.putData(name + "_rotController", rotController);
  }
}