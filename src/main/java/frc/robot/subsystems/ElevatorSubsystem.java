// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstant;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final WPI_VictorSPX leftElevatorMotor;
  private final WPI_VictorSPX rightElevatorMotor;

  private final DigitalInput upLimitSwitch;
  private final DigitalInput downLimitSwitch;

  private final Encoder encoder;
  private final PIDController elevatorPID;

  private Distance targetHeight;

  private final Supplier<Boolean> shouldUsePID, bypassLimitSW;

  public ElevatorSubsystem(Supplier<Boolean> shouldUsePID, Supplier<Boolean> bypassLimitSW) {
    this.shouldUsePID = shouldUsePID;
    this.bypassLimitSW = bypassLimitSW;

    leftElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kLeftElevatorMotorChannel);
    rightElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kRightElevatorMotorChannel);

    leftElevatorMotor.setInverted(ElevatorConstant.kMotorInverted);

    leftElevatorMotor.setSafetyEnabled(true);
    rightElevatorMotor.setSafetyEnabled(true);

    leftElevatorMotor.setExpiration(ElevatorConstant.kMotorSafetyExpirationTime);
    rightElevatorMotor.setExpiration(ElevatorConstant.kMotorSafetyExpirationTime);


    upLimitSwitch = new DigitalInput(5);
    downLimitSwitch = new DigitalInput(7);

    encoder = new Encoder(ElevatorConstant.kEncoderChannelA, ElevatorConstant.kEncoderChannelB);
    encoder.setDistancePerPulse(ElevatorConstant.kEncoderDistancePerPulse);
    encoder.setReverseDirection(false);

    elevatorPID = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
    elevatorPID.setTolerance(8);

    targetHeight = ElevatorConstant.kInitialHeight;
    encoder.reset();
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

  public Distance getCurrentHeight() {
    return Millimeters.of(encoder.getDistance()).plus(ElevatorConstant.kHeightOffset);
  }

  public void moveUp() {
    moveToHeight(targetHeight.plus(ElevatorConstant.kStepHeight));
  }

  public void moveDown() {
    moveToHeight(targetHeight.minus(ElevatorConstant.kStepHeight));
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

  public void toGetSecAlgae() {
    moveToHeight(ElevatorConstant.kToGetSecAlgaeHeight);
  }

  public void toGetTrdAlgae() {
    moveToHeight(ElevatorConstant.kToGetTrdAlgaeHeight);
  }

  public void stopMove() {
    leftElevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    Distance currentHeight = getCurrentHeight();
    boolean usePID = this.shouldUsePID.get();
    boolean bypassLimitSW = this.bypassLimitSW.get();

    var shouldSlowHeight = Constants.ElevatorConstant.kMaxHeight
        .minus(Constants.ElevatorConstant.kHeightOffset).div(3).times(2);

    if (usePID) {
      elevatorPID.setSetpoint(targetHeight.in(Millimeters));
      double output = elevatorPID.calculate(currentHeight.in(Millimeters));

      var maxOutput = currentHeight.gt(shouldSlowHeight) ? ElevatorConstant.kMaxOutputLower : ElevatorConstant.kMaxOutputHigher;
      output = MathUtil.clamp(output, ElevatorConstant.kMinOutput, maxOutput);

      if (!bypassLimitSW) {
        if (!upLimitSwitch.get() && output > 0) {
          output = 0;
        }
        // if (!downLimitSwitch.get() && output < 0) {
        // output = 0;
        // }
      }

      leftElevatorMotor.set(ControlMode.PercentOutput, output);
      rightElevatorMotor.set(ControlMode.PercentOutput, output);
      SmartDashboard.putNumber("ElevatorOutput", output);
    } else {
      targetHeight = currentHeight;
    }

    SmartDashboard.putBoolean("ElevatorUsePID", usePID);
    SmartDashboard.putBoolean("ElevatorBypassLimitSW", bypassLimitSW);

    SmartDashboard.putNumber("ElevatorEncoder", encoder.getDistance());
    SmartDashboard.putBoolean("ElevatorUpLimitSwitch", upLimitSwitch.get());
    SmartDashboard.putBoolean("ElevatorDownLimitswitch", downLimitSwitch.get());

    SmartDashboard.putNumber("ElevatorCurrentHeight", currentHeight.in(Millimeters));

    SmartDashboard.putData("ElevatorPID", elevatorPID);
  }

  public Command toSecFloorCmd() {
    Command cmd = runOnce(this::toSecFloor);
    cmd.setName("toSecFloor");
    return cmd;
  }

  public Command toTrdFloorCmd() {
    Command cmd = runOnce(this::toTrdFloor);
    cmd.setName("toTrdFloor");
    return cmd;
  }

  public Command toTopFloorCmd() {
    Command cmd = runOnce(this::toTopFloor);
    cmd.setName("toTopFloor");
    return cmd;
  }

  public Command toDefaultPositionCmd() {
    Command cmd = runOnce(this::toDefaultPosition);
    cmd.setName("toDefaultPosition");
    return cmd;
  }

  public Command moveUpCmd() {
    Command cmd = runOnce(() -> targetHeight = getCurrentHeight())
        .andThen(run(this::moveUp));
    cmd.setName("moveUp");
    return cmd;
  }

  public Command moveDownCmd() {
    Command cmd = runOnce(() -> targetHeight = getCurrentHeight())
        .andThen(run(this::moveDown));
    cmd.setName("moveDown");
    return cmd;
  }

  public Command manualMoveCmd(double power) {
    Command cmd = runEnd(() -> {
      double adjustedPower = power;
      if (!upLimitSwitch.get() && power > 0) {
        adjustedPower = 0;
      }

      leftElevatorMotor.set(ControlMode.PercentOutput, adjustedPower);
      rightElevatorMotor.set(ControlMode.PercentOutput, adjustedPower);
      SmartDashboard.putNumber("manualMovePower", adjustedPower);

      targetHeight = getCurrentHeight();
    }, () -> {
      leftElevatorMotor.set(ControlMode.PercentOutput, 0);
      rightElevatorMotor.set(ControlMode.PercentOutput, 0);
      SmartDashboard.putNumber("manualMovePower", power);
    });
    cmd.setName("manualMove");
    return cmd;
  }

  public Command manualMoveUpCmd() {
    Command cmd = manualMoveCmd(ElevatorConstant.kManualUpPower);
    cmd.setName("manualMoveUp");
    return cmd;
  }

  public Command manualMoveDownCmd() {
    Command cmd = manualMoveCmd(ElevatorConstant.kManualDownPower);
    cmd.setName("manualMoveDown");
    return cmd;
  }

  public Command elevatorReset() {
    Command cmd = run(this::resetEncoder);
    cmd.setName("elevatorReset");
    return cmd;
  }

  public Command toGetSecAlgaeCmd() {
    Command cmd = runOnce(this::toGetSecAlgae);
    cmd.setName("toGetSecAlgae");
    return cmd;
  }

  public Command toGetTrdAlgaeCmd() {
    Command cmd = runOnce(this::toGetTrdAlgae);
    cmd.setName("toGetTrdAlgae");
    return cmd;
  }

  public double getAbsoluteError() {
    return Math.abs(elevatorPID.getError());
  }

  public boolean isAtTargetHeight() {
    return elevatorPID.atSetpoint();
  }
}