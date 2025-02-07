// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstant;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final SparkMax elevatorMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pidController;
  private final DigitalInput limitSwitchUp;
  private final DigitalInput limitSwitchDown;
  private boolean isButtonControl = false;
  private Distance targetHeight;

  public ElevatorSubsystem() {
    elevatorMotor = new SparkMax(0, MotorType.kBrushless);
    pidController = elevatorMotor.getClosedLoopController();
    encoder = elevatorMotor.getEncoder();
    limitSwitchUp = new DigitalInput(0);
    limitSwitchDown = new DigitalInput(1);
    boolean isInverted = elevatorMotor.configAccessor.getInverted();
    double positionFactor = elevatorMotor.configAccessor.encoder.getPositionConversionFactor();
    double velocityFactor = elevatorMotor.configAccessor.encoder.getVelocityConversionFactor();
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(isInverted)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(positionFactor)
        .velocityConversionFactor(velocityFactor);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);
    //不確定是否用SparkMax 所以我覺得還是分開寫 而且分開寫比較能做細部調整
    //如果確認不是的話我再改就好:)
    elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder.setPosition(ElevatorConstant.kInitialHeight.in(Meters));
    targetHeight = ElevatorConstant.kInitialHeight;

  }

  public void moveToHeight(Distance newTargetHeight) { 
    if (newTargetHeight.gt(ElevatorConstant.kMaxHeight)) { 
      newTargetHeight = ElevatorConstant.kMaxHeight;
    } else if (newTargetHeight.lt(ElevatorConstant.kInitialHeight)) { 
      newTargetHeight = ElevatorConstant.kInitialHeight;
    }
    targetHeight = newTargetHeight;
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

  public void todefaultPosition() {
    moveToHeight(ElevatorConstant.kInitialHeight);
  }

  public void stopMove() {
    pidController.setReference(encoder.getPosition(), SparkMax.ControlType.kPosition);
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
    Command cmd = run(this::todefaultPosition);
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
    Distance currentHeight = Meters.of(encoder.getPosition());

    if ((targetHeight.gt(currentHeight) && !limitSwitchUp.get()) ||
        (targetHeight.lt(currentHeight) && !limitSwitchDown.get())) {
      pidController.setReference(targetHeight.in(Meters), SparkMax.ControlType.kPosition);
    } else {
      stopMove();
    }
  }
}

