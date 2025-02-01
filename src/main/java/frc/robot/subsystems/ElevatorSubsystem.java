// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstant;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final SparkMax elevatorMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pidController;
  private final Joystick joystick;
  private final DigitalInput limitSwitchUp; 
  private final DigitalInput limitSwitchDown;  
  private boolean isButtonControl = false;
  

  public ElevatorSubsystem(Joystick joystick) {
    this.joystick = joystick;
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
    elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder.setPosition(ElevatorConstant.kInitialHeight);

  }

  public void moveToHeight(double targetHeight) {//限制開關偵測
    if ((targetHeight > encoder.getPosition() && !limitSwitchUp.get()) || 
        (targetHeight < encoder.getPosition() && !limitSwitchDown.get())) {
      pidController.setReference(targetHeight, SparkMax.ControlType.kPosition);
    } else {
      stopMove();
    }
  }

  public void toSecFloor(){
    moveToHeight(ElevatorConstant.kSecFloor);
  }

  public void toTrdFloor(){
    moveToHeight(ElevatorConstant.kTrdFloor);
  }

  public void toTopFloor(){
    moveToHeight(ElevatorConstant.kTopFloor);
  }

  public void todefaultPosition() {
    moveToHeight(ElevatorConstant.kInitialHeight);
  }

  public void stopMove() {
    elevatorMotor.stopMotor();
  }

  public Command ToSecFloorCmd(){
    Command cmd = run(this::toSecFloor);
    return cmd;
  }

  public Command ToTrdFloorCmd(){
    Command cmd = run(this::toTrdFloor);
    return cmd;
  }

  public Command ToTopFloorCmd(){
    Command cmd = run(this::toTopFloor);
    return cmd;
  }

  public Command ToDefaultPositionCmd(){
    Command cmd = run(this::todefaultPosition);
    return cmd;
  }

  public void setButtonControl(boolean controlMode) {//切換搖桿控制或按鈕控制
    this.isButtonControl = controlMode;
  }

  public boolean isButtonControl() {
    return this.isButtonControl;
  }

  @Override
  public void periodic() {
    if (!isButtonControl) {
      double joystickInput = -joystick.getY();
      double currentHeight = encoder.getPosition();
      double deltaHeight = joystickInput * 10; 
      double targetHeight = currentHeight + deltaHeight;
        if (Math.abs(targetHeight - currentHeight) > 5) {//平滑過渡閾值
            targetHeight = currentHeight + Math.signum(deltaHeight) * 5;
        }
      moveToHeight(targetHeight);
    }
  }
}
