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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstant;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final WPI_VictorSPX elevatorMotor;
  private final Encoder encoder;
  private final PIDController elevatorPID;
  private final DigitalInput limitSwitchUp;
  private final DigitalInput limitSwitchDown;
  private boolean isButtonControl = false;
  private Distance targetHeight;
  private Distance truthHeight;
  public ElevatorSubsystem() {
    elevatorMotor = new WPI_VictorSPX(0);
    elevatorMotor.setInverted(true);
    encoder = new Encoder(0, 1);
    encoder.setDistancePerPulse(ElevatorConstant.kEncoderDistancePerPulse);
    elevatorPID = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
    limitSwitchUp = new DigitalInput(0);
    limitSwitchDown = new DigitalInput(1);

    encoder.reset();
    targetHeight = ElevatorConstant.kInitialHeight;
    truthHeight = targetHeight.plus(ElevatorConstant.kStartedOffest);
  }

  public void moveToHeight(Distance newTargetHeight) { 
    if (newTargetHeight.gt(ElevatorConstant.kMaxHeight)) { 
      newTargetHeight = ElevatorConstant.kMaxHeight;
    } else if (newTargetHeight.lt(ElevatorConstant.kLowestHeight)) { 
      newTargetHeight = ElevatorConstant.kLowestHeight;
    }
    targetHeight = newTargetHeight;
    truthHeight = newTargetHeight.plus(ElevatorConstant.kStartedOffest);

  }

  public void moveUp() {
    moveToHeight(targetHeight.plus(ElevatorConstant.kStepHeight
    ));
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

  public void stopMove() {
    elevatorMotor.set(ControlMode.PercentOutput, 0);
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

  public Command moveUpCmd() {
    Command cmd = run(this::moveUp);
    return cmd;
  }

  public Command moveDownCmd() {
    Command cmd = run(this::moveDown);
    return cmd;
  }

  public void setButtonControl(boolean controlMode) { // 切換搖桿控制或按鈕控制
    this.isButtonControl = controlMode;
  }

  public boolean isButtonControl() {
    return this.isButtonControl;
  }

  @Override
  public void periodic() {
    Distance currentHeight = Millimeters.of(encoder.getDistance()).plus(ElevatorConstant.kStartedOffest);
    elevatorPID.setSetpoint(targetHeight.in(Millimeters));
    double output = elevatorPID.calculate(currentHeight.in(Millimeters));
    output = MathUtil.clamp(output, -1.0, 1.0);

    if ((output>=0 && !limitSwitchUp.get())
        || ((output<=0 && !limitSwitchDown.get()))) {
      elevatorMotor.set(ControlMode.PercentOutput, output );
    } else {
      stopMove();
    }
    //V1沒有limitSwitch 但不影響
  }
}

