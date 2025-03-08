// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstant;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final WPI_VictorSPX leftElevatorMotor;
  private final WPI_VictorSPX rightElevatorMotor;
  private final Encoder elevatorEncoder;
  private final PIDController elevatorPID;
  private Distance targetHeight;
  private boolean isManualControl;
  private final DigitalInput touchSensorDown;
  private final DigitalInput touchSensorUp1;
  private final DigitalInput touchSensorUp2;


  public ElevatorSubsystem() {
    // motor
    leftElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kLeftElevatorMotorChannel);
    rightElevatorMotor = new WPI_VictorSPX(ElevatorConstant.kRightElevatorMotorChannel);
    leftElevatorMotor.setInverted(ElevatorConstant.kLeftMotorInverted);
    rightElevatorMotor.setInverted(ElevatorConstant.kRightMotorInverted);

    // encoder
    elevatorEncoder = new Encoder(ElevatorConstant.kEncoderChannelA, ElevatorConstant.kEncoderChannelB);
    elevatorEncoder.setDistancePerPulse(ElevatorConstant.kEncoderDistancePerPulse);
    elevatorEncoder.setReverseDirection(ElevatorConstant.kEncoderInverted);
    elevatorEncoder.reset();

    // PID
    elevatorPID = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);

    // touch sensor
    touchSensorDown = new DigitalInput(ElevatorConstant.kTouchSensorChannel);
    touchSensorUp1 = new DigitalInput(ElevatorConstant.kTouchSensorUp1Channel);
    touchSensorUp2 = new DigitalInput(ElevatorConstant.kTouchSensorUp2Channel);

    targetHeight = ElevatorConstant.kInitialHeight;

    // manual control
    isManualControl = false;
  }

  public void resetEncoder() {
    elevatorEncoder.reset();
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
    return Millimeters.of(elevatorEncoder.getDistance()).plus(ElevatorConstant.kHeightOffset);
  }

  public void moveUp() {
    if(targetHeight.plus(ElevatorConstant.kStepHeight).gt(ElevatorConstant.kMaxHeight))  {
      moveToHeight(ElevatorConstant.kMaxHeight);
    }
    moveToHeight(targetHeight.plus(ElevatorConstant.kStepHeight));
  }

  public void moveDown() {
    if(targetHeight.minus(ElevatorConstant.kStepHeight).lt(ElevatorConstant.kInitialHeight)){
      moveToHeight(ElevatorConstant.kInitialHeight);
    }
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
    if (!touchSensorDown.get()) {
      targetHeight.plus(ElevatorConstant.kStepHeight);
    }
     
    if (!touchSensorUp1.get() || !touchSensorUp2.get()) {
      targetHeight.minus(ElevatorConstant.kStepHeight);
    }


    if (!isManualControl) {

      elevatorPID.setSetpoint(targetHeight.in(Millimeters));
      double output = elevatorPID.calculate(currentHeight.in(Millimeters));
      output = MathUtil.clamp(output, ElevatorConstant.kMinOutput, ElevatorConstant.kMaxOutput);

      if (getCurrentHeight().in(Millimeters) > 1300 && output > ElevatorConstant.kUpperMaxOutput) {
        output = MathUtil.clamp(output, ElevatorConstant.kMinOutput, ElevatorConstant.kUpperMaxOutput);
      }

      if (output < 0 && getCurrentHeight().in(Millimeters) < 900) {
        output = MathUtil.clamp(output, ElevatorConstant.kMinLowerOutput, ElevatorConstant.kUpperMaxOutput);
      }
      SmartDashboard.putNumber("ElevatorOutput", output);
      leftElevatorMotor.follow(rightElevatorMotor, FollowerType.PercentOutput);
      rightElevatorMotor.set(ControlMode.PercentOutput, output);
    } else {
      targetHeight = currentHeight;
    }
    

    SmartDashboard.putBoolean("DownElevatorTouchsensor", touchSensorDown.get());
    SmartDashboard.putBoolean("UpElevatorTouchsensor1", touchSensorUp1.get());
    SmartDashboard.putBoolean("UpElevatorTouchsensor2", touchSensorUp2.get());
    SmartDashboard.putNumber("ElevatorEncoder", elevatorEncoder.getDistance());
    SmartDashboard.putNumber("ElevatorCurrentHeight", currentHeight.in(Millimeters));
    SmartDashboard.putNumber("ElevatorTargetHeight", targetHeight.in(Millimeters));
    SmartDashboard.putBoolean("ElevatorIsManualControl", isManualControl);
    SmartDashboard.putData("ElevatorPID", elevatorPID);
  }

  public Command toGetCarolHeightCmd() {
    Command cmd = runOnce(this::toGetCarolHeight);
    cmd.setName("toGetCarolHeight");
    return cmd;
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
      isManualControl = true;

    }, () -> {
      leftElevatorMotor.set(ControlMode.PercentOutput, 0);
      rightElevatorMotor.set(ControlMode.PercentOutput, 0);
      isManualControl = false;
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

  public double getAbsoluteError(){
    return Math.abs(elevatorPID.getError());
  }
}