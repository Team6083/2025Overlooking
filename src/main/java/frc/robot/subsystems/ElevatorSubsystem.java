// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
  private double targetHeight;

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
        .pid(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);//不確定是否用SparkMax 所以我覺得還是分開寫 而且分開寫必較能做細部調整
        //如果確認不是的話我再改就好:)
    elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder.setPosition(ElevatorConstant.kInitialHeight);
    targetHeight = ElevatorConstant.kInitialHeight;

  }

  public void moveToHeight(double newTargetHeight) { // 限制開關偵測
    targetHeight = newTargetHeight;
  }//此行程式需不停重複運行 應放在最下面的periodic 所以可能要改

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
    elevatorMotor.stopMotor();
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

  public void setButtonControl(boolean controlMode) { // 切換搖桿控制或按鈕控制
    this.isButtonControl = controlMode;
  }

  public boolean isButtonControl() {
    return this.isButtonControl;
  }

  @Override
  public void periodic() {
    if ((targetHeight > encoder.getPosition() && !limitSwitchUp.get())
        || (targetHeight < encoder.getPosition() && !limitSwitchDown.get())) {
      pidController.setReference(targetHeight, SparkMax.ControlType.kPosition);
    } else {
      stopMove();
    }
    
    }
  }

