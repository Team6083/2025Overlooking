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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstant;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final WPI_VictorSPX leftElevatorMotor;
  private final WPI_VictorSPX rightElevatorMotor;
  private final Encoder encoder;
  private final PIDController elevatorPID;
  private Distance targetHeight;
  private boolean manualControl;
  private DigitalInput upLimitSwitch;
  private DigitalInput downLimitSwitch;

  public ElevatorSubsystem() {
    leftElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kLeftElevatorMotorChannel);
    rightElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kRightElevatorMotorChannel);
    leftElevatorMotor.setInverted(ElevatorConstant.kMotorInverted);
    encoder = new Encoder(ElevatorConstant.kEncoderChannelA, ElevatorConstant.kEncoderChannelB);
    encoder.setDistancePerPulse(ElevatorConstant.kEncoderDistancePerPulse);
    elevatorPID = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
    rightElevatorMotor.follow(leftElevatorMotor);
    manualControl = false;
    encoder.setReverseDirection(false);
    encoder.reset();
    targetHeight = ElevatorConstant.kInitialHeight;
    upLimitSwitch = new DigitalInput(5);
    downLimitSwitch = new DigitalInput(8);

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

    // if (encoder.getStopped()
    // || Math.abs(encoder.getRate()) < 0.5
    // || (output > 0 && encoder.getRate() < 0)
    // || (output < 0 && encoder.getRate() > 0)) {

    // SmartDashboard.putNumber("Output", 0);

    if (encoder.getStopped()
        || Math.abs(encoder.getRate()) < 0.5) {
      stopMove();
    } else if (!manualControl && upLimitSwitch.get() && downLimitSwitch.get()) {
      if (!upLimitSwitch.get()) {
        targetHeight = currentHeight.minus(ElevatorConstant.kStepHeight);
      } else if (!downLimitSwitch.get()) {
        targetHeight = currentHeight.plus(ElevatorConstant.kStepHeight);
      }
      elevatorPID.setSetpoint(targetHeight.in(Millimeters));
      double output = elevatorPID.calculate(currentHeight.in(Millimeters));
      output = MathUtil.clamp(output, ElevatorConstant.kMinOutput, ElevatorConstant.kMaxOutput);
      leftElevatorMotor.set(ControlMode.PercentOutput, output);
      rightElevatorMotor.set(ControlMode.PercentOutput, output);
      SmartDashboard.putNumber("Output", output);
    } else {
      targetHeight = currentHeight;
    }

    SmartDashboard.putNumber("ElevatorEncoder", encoder.getDistance());
    SmartDashboard.putNumber("ElevatorCurrentHeight", currentHeight.in(Millimeters));
    SmartDashboard.putBoolean("ElevatorIsManualControl", isManualControl());
    SmartDashboard.putData("ElevatorPID", elevatorPID);
    SmartDashboard.putBoolean("ElevatorUpLimitSwitch", upLimitSwitch.get());
    SmartDashboard.putBoolean("ElevatorDownLimitswitch", downLimitSwitch.get());
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
    Command cmd = run(this::moveUp);
    cmd.setName("moveUp");
    return cmd;
  }

  public Command moveDownCmd() {
    Command cmd = run(this::moveDown);
    cmd.setName("moveDown");
    return cmd;
  }

  public Command manualMoveCmd(double power) {
    Command cmd = runEnd(() -> {
      leftElevatorMotor.set(ControlMode.PercentOutput, power);
      rightElevatorMotor.set(ControlMode.PercentOutput, power);
      manualControl = true;

    }, () -> {
      leftElevatorMotor.set(ControlMode.PercentOutput, 0);
      rightElevatorMotor.set(ControlMode.PercentOutput, 0);
      manualControl = false;
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
    cmd.setName("toGetTriAlgae");
    return cmd;
  }

  public Command autoStopCmd(Command command) {
    Command cmd = new SequentialCommandGroup(
        command.repeatedly()
            .until(() -> Math.abs(elevatorPID.getError()) < 15),
        runOnce(this::stopMove));
    cmd.setName("autoStop");
    return cmd;
  }
}