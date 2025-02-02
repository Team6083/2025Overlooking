// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivebase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
  private final SparkBaseConfig driveMotorConfig;
  private final SparkBaseConfig turningmotorConfig;
  private final RelativeEncoder driveEncoder;
  private final CANcoder turningEncoder;
  public double AbsoluteSensorDiscontinuityPoint;
  private String Modulename;
  private boolean driveInverted;
  private boolean turningInverted;
  private final PIDController rotController;

  public SwerveModule(
      int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, int CANcoderID, String Modulename) {
    this.Modulename = Modulename;
    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPosition(0);
    turningEncoder = new CANcoder(CANcoderID);
    driveMotorConfig = new SparkMaxConfig();
    turningmotorConfig = new SparkMaxConfig();
    rotController = new PIDController(1, 0, 0);
    rotController.enableContinuousInput(-180.0, 180.0);

    driveMotorConfig.inverted(driveInverted);
    turningmotorConfig.inverted(turningInverted);

    setName(Modulename);
    AbsoluteSensorDiscontinuityPoint = 0.5;
    CANcoderConfiguration turningEncoderConfiguration = new CANcoderConfiguration();
    turningEncoderConfiguration.MagnetSensor.MagnetOffset = 0;
    turningEncoderConfiguration.MagnetSensor.MagnetOffset = AbsoluteSensorDiscontinuityPoint;

    turningEncoder.getConfigurator().apply(turningEncoderConfiguration);
  }





public void configDriveMotor() {
    SparkMaxConfig configdriveMotor = new SparkMaxConfig();
    configdriveMotor.smartCurrentLimit(40, 20);
    configdriveMotor.closedLoopRampRate(ModuleConstants.kDdriveClosedLoopRampRate);
    configdriveMotor.idleMode(IdleMode.kBrake);
    configdriveMotor.voltageCompensation(ModuleConstants.kMaxModuleDriveVoltage);
    driveMotor.configure(configdriveMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configTurningMotor() {
    SparkMaxConfig configturningMotor = new SparkMaxConfig();
    configturningMotor.smartCurrentLimit(20);
    configturningMotor.closedLoopRampRate(ModuleConstants.kTurningClosedLoopRampRate);
    configturningMotor.idleMode(IdleMode.kBrake);
    configturningMotor.voltageCompensation(ModuleConstants.kMaxModuleTurningVoltage);
    turningMotor.configure(configturningMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void resetAllEncoder() {
    driveEncoder.setPosition(0);
  }

  public void stopModule() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveRate(), new Rotation2d(Math.toRadians(getRotation())));
  }

  public double getDriveDistance() {
    return driveEncoder.getPosition() / ModuleConstants.kWheelGearRate * 2.0 * Math.PI * ModuleConstants.kWheelRadius;
  }

  public double getDriveRate() {
    return driveEncoder.getVelocity() / 60.0 / ModuleConstants.kWheelGearRate * 2.0 * Math.PI
        * ModuleConstants.kWheelRadius;
  }
   public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(Math.toRadians(getRotation())));
  }

  public double getRotation() {
    return turningEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }


  public double[] optimizeOutputVoltage(SwerveModuleState goalState, double currentTurningDegree) {
    goalState.optimize(Rotation2d.fromDegrees(currentTurningDegree));
    double driveMotorVoltage = goalState.speedMetersPerSecond * ModuleConstants.kDesireSpeedtoMotorVoltage;
    double turningMotorVoltage = rotController.calculate(currentTurningDegree, goalState.angle.getDegrees());
    double[] moduleState = { driveMotorVoltage, turningMotorVoltage };
    return moduleState;
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
    double turningDegreeVoltage = rotController.calculate(getRotation(), 0);
    turningMotor.setVoltage(turningDegreeVoltage);
  }

  public void setTurningDegree90() {
    double turningDegreeTo90Voltage = rotController.calculate(getRotation(), 90);
    turningMotor.setVoltage(turningDegreeTo90Voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(Modulename + "_ModuleDistance", getDriveDistance());
    SmartDashboard.putNumber(Modulename + "_ModuleVelocity", getDriveRate());
    SmartDashboard.putNumber(Modulename + "_ModuleRotation", getRotation());
    SmartDashboard.putData(Modulename + "_rotController", rotController);
  }
}
