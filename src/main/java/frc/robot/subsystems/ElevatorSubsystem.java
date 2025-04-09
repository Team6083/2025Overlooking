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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConfigChooser;
import frc.robot.Constants.ElevatorConstant;
import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final WPI_VictorSPX leftElevatorMotor;
  private final WPI_VictorSPX rightElevatorMotor;

  private final DigitalInput upLimitSwitch;

  private final Encoder encoder;
  private final PIDController elevatorPID;

  private Distance targetHeight;

  private final Supplier<Boolean> shouldUsePID;
  private final Supplier<Boolean> bypassLimitSwitch;

  public ElevatorSubsystem(Supplier<Boolean> shouldUsePID, Supplier<Boolean> bypassLimitSwitch) {
    this.shouldUsePID = shouldUsePID;
    this.bypassLimitSwitch = bypassLimitSwitch;

    leftElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kLeftElevatorMotorChannel);
    rightElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kRightElevatorMotorChannel);

    leftElevatorMotor.setInverted(ElevatorConstant.kMotorInverted);

    leftElevatorMotor.setSafetyEnabled(true);
    rightElevatorMotor.setSafetyEnabled(true);

    leftElevatorMotor.setExpiration(ElevatorConstant.kMotorSafetyExpirationTime);
    rightElevatorMotor.setExpiration(ElevatorConstant.kMotorSafetyExpirationTime);

    upLimitSwitch = new DigitalInput(5);

    encoder = new Encoder(ElevatorConstant.kEncoderChannelA,
        ElevatorConstant.kEncoderChannelB);
    encoder.setDistancePerPulse(ElevatorConstant.kEncoderDistancePerPulse);
    encoder.setReverseDirection(false);

    elevatorPID = new PIDController(ConfigChooser.Elevator.getDouble("kP"), ElevatorConstant.kI, ElevatorConstant.kD);
    elevatorPID.setTolerance(8);

    targetHeight = ConfigChooser.Elevator.getDistance("kInitialHeight");
    encoder.reset();
  }

  public void resetEncoder() {
    encoder.reset();
    targetHeight = getCurrentHeight();
  }

  public void moveToHeight(Distance newTargetHeight) {
    if (newTargetHeight.gt(ConfigChooser.Elevator.getDistance("kMaxHeight"))) {
      newTargetHeight = ConfigChooser.Elevator.getDistance("kMaxHeight");
    } else if (newTargetHeight.lt(ConfigChooser.Elevator.getDistance("kLowestHeight"))) {
      newTargetHeight = ConfigChooser.Elevator.getDistance("kLowestHeight");
    }
    targetHeight = newTargetHeight;
  }

  public Distance getCurrentHeight() {
    return Millimeters.of(encoder.getDistance()).plus(ConfigChooser.Elevator.getDistance("kHeightOffset"));
  }

  public void moveUp() {
    moveToHeight(targetHeight.plus(ElevatorConstant.kStepHeight));
  }

  public void moveDown() {
    moveToHeight(targetHeight.minus(ElevatorConstant.kStepHeight));
  }

  public void toSecFloor() {
    moveToHeight(ConfigChooser.Elevator.getDistance("kSecFloor"));
  }

  public void toTrdFloor() {
    moveToHeight(ConfigChooser.Elevator.getDistance("kTrdFloor"));
  }

  public void toTopFloor() {
    moveToHeight(ConfigChooser.Elevator.getDistance("kTopFloor"));
  }

  public void toDefaultPosition() {
    moveToHeight(ConfigChooser.Elevator.getDistance("kInitialHeight"));
  }

  public void toGetSecAlgae() {
    moveToHeight(ConfigChooser.Elevator.getDistance("kToGetSecAlgaeHeight"));
  }

  public void toGetTrdAlgae() {
    moveToHeight(ConfigChooser.Elevator.getDistance("kToGetTrdAlgaeHeight"));
  }

  public void stopMove() {
    leftElevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean shouldMotorStop() {
    if (bypassLimitSwitch.get() && !shouldUsePID.get()) {
      return false;
    }

    return upLimitSwitch.get();
  }

  @Override
  public void periodic() {
    Distance currentHeight = getCurrentHeight();
    boolean usePID = this.shouldUsePID.get();

    var shouldSlowHeight = ConfigChooser.Elevator.getDistance("kTrdFloor")
        .plus((ConfigChooser.Elevator.getDistance("kTopFloor")
            .minus(ConfigChooser.Elevator.getDistance("kTrdFloor")))
            .div(3).times(1));

    if (usePID) {
      elevatorPID.setSetpoint(targetHeight.in(Millimeters));
      double output = elevatorPID.calculate(currentHeight.in(Millimeters));

      var maxOutput = currentHeight.gt(shouldSlowHeight) ? ConfigChooser.Elevator.getDouble("kMaxOutputLower")
          : ConfigChooser.Elevator.getDouble("kMaxOutputHigher");
      output = MathUtil.clamp(output, ElevatorConstant.kMinOutput, maxOutput);

      if (shouldMotorStop()) {
        output = 0;
      }

      leftElevatorMotor.set(ControlMode.PercentOutput, output);
      rightElevatorMotor.set(ControlMode.PercentOutput, output);
    } else {
      targetHeight = currentHeight;

      if (shouldMotorStop()) {
        leftElevatorMotor.set(ControlMode.PercentOutput, 0);
        rightElevatorMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    SmartDashboard.putNumber("ElevatorOutput",
        leftElevatorMotor.getMotorOutputVoltage());

    SmartDashboard.putBoolean("ElevatorUsePID", usePID);
    SmartDashboard.putBoolean("ElevatorBypassLimitSwitch", bypassLimitSwitch.get());

    SmartDashboard.putNumber("ElevatorEncoder", encoder.getDistance());
    SmartDashboard.putBoolean("ElevatorUpLimitSwitch", upLimitSwitch.get());

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
      if (shouldMotorStop()) {
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