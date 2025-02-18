// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstant;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final WPI_VictorSPX rightElevatorMotor;
  private final WPI_VictorSPX leftElevatorMotor;
  private final Encoder leftEncoder;
  private final Encoder rightEncoder;
  private final PIDController leftElevatorPID;
  private final PIDController rightElevatorPID;
  private Distance leftTargetHeight;
  private Distance rightTargetHeight;
  private boolean manualControl;

  public ElevatorSubsystem() {
    rightElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kRightElevatorMotorChannel);
    leftElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kLeftElevatorMotorChannel);
    leftElevatorMotor.setInverted(false);
    rightElevatorMotor.setInverted(true);
    leftEncoder = new Encoder(2, 3);
    rightEncoder = new Encoder(4, 5);
    leftEncoder.setDistancePerPulse(ElevatorConstant.kEncoderDistancePerPulse);
    leftEncoder.setDistancePerPulse(ElevatorConstant.kEncoderDistancePerPulse);
    leftElevatorPID = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
    rightElevatorPID = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
    manualControl = false;
    leftEncoder.reset();
    rightEncoder.reset();
    leftTargetHeight = ElevatorConstant.kInitialHeight;
    rightTargetHeight = ElevatorConstant.kInitialHeight;

  }

  public void resetEncoder() {
    leftEncoder.reset();
    rightEncoder.reset();
    leftTargetHeight = getLeftCurrentHeight();
    rightTargetHeight = getRightCurrentHeight();

  }

  public void moveToHeight(Distance newTargetHeight) {
    if (newTargetHeight.gt(ElevatorConstant.kMaxHeight)) {
      newTargetHeight = ElevatorConstant.kMaxHeight;
    } else if (newTargetHeight.lt(ElevatorConstant.kLowestHeight)) {
      newTargetHeight = ElevatorConstant.kLowestHeight;
    }
    leftTargetHeight = newTargetHeight;
    rightTargetHeight = newTargetHeight;
  }

  public void setManualControl(boolean manualControlOn) {
    this.manualControl = manualControlOn;
  }

  public boolean isManualControl() {
    return manualControl;
  }

  public Distance getLeftCurrentHeight() {
    return Millimeters.of(leftEncoder.getDistance()).plus(ElevatorConstant.kHeightOffset);
  }

  public Distance getRightCurrentHeight() {
    return Millimeters.of(rightEncoder.getDistance()).plus(ElevatorConstant.kHeightOffset);
  }

  public void moveUp() {
    leftTargetHeight = leftTargetHeight.plus(ElevatorConstant.kStepHeight);
    rightTargetHeight = rightTargetHeight.plus(ElevatorConstant.kStepHeight);
  }

  public void moveDown() {
    leftTargetHeight = leftTargetHeight.minus(ElevatorConstant.kStepHeight);
    rightTargetHeight = rightTargetHeight.minus(ElevatorConstant.kStepHeight);
  }

  public void toGetCarolHeight() {
    moveToHeight(ElevatorConstant.kGetCarolHeight);
  }

  public void toSecFloor() {
    moveToHeight(ElevatorConstant.kSecFloor);
  }

  public void toTrdFloor() {
    moveToHeight(ElevatorConstant.kTrdFloor);
  }

  public void toTopFloor() {
    moveToHeight(ElevatorConstant.kTopFloor);
  }

  public void toDefaultPosition() {
    moveToHeight(ElevatorConstant.kInitialHeight);
  }

  public void stopMove() {
    rightElevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public Command toGetCarolHeightCmd() {
    Command cmd = runOnce(this::toGetCarolHeight);
    return cmd;
  }

  public Command toSecFloorCmd() {
    Command cmd = runOnce(this::toSecFloor);
    return cmd;
  }

  public Command toTrdFloorCmd() {
    Command cmd = runOnce(this::toTrdFloor);
    return cmd;
  }

  public Command toTopFloorCmd() {
    Command cmd = runOnce(this::toTopFloor);
    return cmd;
  }

  public Command toDefaultPositionCmd() {
    Command cmd = runOnce(this::toDefaultPosition);
    return cmd;
  }

  public Command manualMoveCmd(double power) {
    return runEnd(() -> {
      rightElevatorMotor.set(ControlMode.PercentOutput, power);
      manualControl = true;

    }, () -> {
      rightElevatorMotor.set(ControlMode.PercentOutput, 0);
      manualControl = false;
    });
  }

  public Command moveUpCmd() {
    return this.run(this::moveUp);
  }

  public Command moveDownCmd() {
    return this.run(this::moveDown);
  }

  public Command switchManualControlCmd(boolean manualControl) {
    Command cmd = runOnce(() -> {
      setManualControl(manualControl);
    });
    cmd.setName("switchManualControl");
    return cmd;
  }

  public Command elevatorReset() {
    Command cmd = run(this::resetEncoder);
    cmd.setName("elevatorReset");
    return cmd;
  }

  @Override
  public void periodic() {
    Distance leftCurrentHeight = getLeftCurrentHeight();
    Distance rightCurrentHeight = getRightCurrentHeight();

    if (!manualControl) {
      leftElevatorPID.setSetpoint(leftTargetHeight.in(Millimeters));
      double leftOutput = leftElevatorPID.calculate(leftCurrentHeight.in(Millimeters));
      leftOutput = MathUtil.clamp(leftOutput, ElevatorConstant.kMinOutput, ElevatorConstant.kMaxOutput);
      leftElevatorMotor.set(ControlMode.PercentOutput, leftOutput);
      SmartDashboard.putNumber("LeftOutput", leftOutput);
      rightElevatorPID.setSetpoint(rightTargetHeight.in(Millimeters));
      double rightOutput = rightElevatorPID.calculate(rightCurrentHeight.in(Millimeters));
      rightOutput = MathUtil.clamp(rightOutput, ElevatorConstant.kMinOutput, ElevatorConstant.kMaxOutput);
      rightElevatorMotor.set(ControlMode.PercentOutput, rightOutput);
      SmartDashboard.putNumber("RightOutput", rightOutput);

    } else {
      leftTargetHeight = leftCurrentHeight;
      rightTargetHeight = rightCurrentHeight;
    }

    SmartDashboard.putNumber("ElevatorLeftEncoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("ElevatorRightEncoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("ElevatorLeftCurrentHeight", leftCurrentHeight.in(Millimeters));
    SmartDashboard.putNumber("ElevatorRightCurrentHeight", rightCurrentHeight.in(Millimeters));
    SmartDashboard.putBoolean("ElevatorIsManualControl", isManualControl());
    SmartDashboard.putData("ElevatorPID", leftElevatorPID);
  }
}