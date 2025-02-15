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
import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final WPI_VictorSPX elevatorMotor;
  private final Encoder encoder;
  private final PIDController elevatorPID;
  private Distance targetHeight;
  private boolean manualControl;

  public ElevatorSubsystem() {
    elevatorMotor = new WPI_VictorSPX(ElevatorConstant.kElevatorMotorChannel);
    elevatorMotor.setInverted(true);
    encoder = new Encoder(2, 3);
    encoder.setDistancePerPulse(ElevatorConstant.kEncoderDistancePerPulse);
    elevatorPID = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
    manualControl = false;

    encoder.reset();
    targetHeight = ElevatorConstant.kInitialHeight;

  }

  public void resetEncoder() {
    encoder.reset();
    targetHeight = getCurrentHeight();
  }

  public void moveToHeight(Distance newTargetHeight) {
    if (newTargetHeight.gt(ElevatorConstant.kMaxHeight)) {
      newTargetHeight = ElevatorConstant.kMaxHeight;
    } else if (newTargetHeight.lt(ElevatorConstant.kLowestHeight)) {
      newTargetHeight = ElevatorConstant.kLowestHeight;
    }
    targetHeight = newTargetHeight;
  }

  public void setManualControl(boolean manualControlOn) {
    this.manualControl = manualControlOn;
  }

  public boolean isManualControl() {
    return manualControl;
  }

  public Distance getCurrentHeight() {
    return Millimeters.of(encoder.getDistance()).plus(ElevatorConstant.kHeightOffset);
  }

  public void moveUp() {
    moveToHeight(targetHeight.plus(ElevatorConstant.kStepHeight));
  }

  public void moveDown() {
    moveToHeight(targetHeight.minus(ElevatorConstant.kStepHeight));
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
    elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public Command toGetCarolHeightCmd() {
    Command cmd = run(this::toGetCarolHeight);
    return cmd;
  }

  public Command toSecFloorCmd() {
    Command cmd = run(this::toSecFloor);
    return cmd;
  }

  public Command toTrdFloorCmd() {
    Command cmd = run(this::toTrdFloor);
    return cmd;
  }

  public Command toTopFloorCmd() {
    Command cmd = run(this::toTopFloor);
    return cmd;
  }

  public Command toDefaultPositionCmd() {
    Command cmd = run(this::toDefaultPosition);
    return cmd;
  }

  public Command manualMoveCmd(double power) {
    return run(() -> {
      if (manualControl) {
        elevatorMotor.set(ControlMode.PercentOutput, power);
      }
    }).finallyDo(() -> elevatorMotor.set(ControlMode.PercentOutput, 0));
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
    Distance currentHeight = getCurrentHeight();

    if (!manualControl) {
      elevatorPID.setSetpoint(targetHeight.in(Millimeters));
      double output = elevatorPID.calculate(currentHeight.in(Millimeters));
      output = MathUtil.clamp(output, ElevatorConstant.kMinOutput, ElevatorConstant.kMaxOutput);
      elevatorMotor.set(ControlMode.PercentOutput, output);
      SmartDashboard.putNumber("Output", output);
    } else {
      targetHeight = currentHeight;
    }

    SmartDashboard.putNumber("ElevatorEncoder", encoder.getDistance());
    SmartDashboard.putNumber("ElevatorCurrentHeight", currentHeight.in(Millimeters));
    SmartDashboard.putBoolean("ElevatorIsManualControl", isManualControl());
    SmartDashboard.putData("ElevatorPID", elevatorPID);
  }
}