// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstant;
import frc.robot.lib.PowerDistribution;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSPX intakeMotor;
  private final VictorSPX rotateMotor;
  private final PIDController algaeRotatePID;
  private final PowerDistribution powerDistribution;
  private boolean isManualControl = false;
  private final DutyCycleEncoder rotateEncoder;
  private double output;

  public AlgaeIntakeSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    algaeRotatePID = new PIDController(
        AlgaeIntakeConstant.rotMotorPIDkD,
        AlgaeIntakeConstant.rotMotorPIDkI,
        AlgaeIntakeConstant.rotMotorPIDkD);

    intakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeRotateMotorChannel);

    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    rotateMotor.setInverted(AlgaeIntakeConstant.kRotateMotorInverted);
    rotateEncoder = new DutyCycleEncoder(AlgaeIntakeConstant.kAlgaeEncoderChannelA);
  }

  public void setIntakeMotorFastOn() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput,
        AlgaeIntakeConstant.kIntakeFastSpeed);
  }

  public void setIntakeMotorSlowOn() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput,
        AlgaeIntakeConstant.kIntakeSlowSpeed);
  }

  public void setReIntake() {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput,
        AlgaeIntakeConstant.kReIntakeSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void setRotate(double speed) {
    rotateMotor.set(VictorSPXControlMode.PercentOutput, speed);

  }

  public void stopRotate() {
    rotateMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void setUpRotateIntakeSetpoint() {
    algaeRotatePID.setSetpoint(AlgaeIntakeConstant.kUpRotateIntakeSetpoint);
  }

  public void setDownRotateIntakeSetpoint() {
    algaeRotatePID.setSetpoint(AlgaeIntakeConstant.kUpRotateIntakeSetpoint);
  }

  public void getCurrentAngle(){
    rotateEncoder.get();
  }

  public void moveRotateToAngle(){
    rotateMotor.set(VictorSPXControlMode.PercentOutput, output);

  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("algaeIntakeVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("algaeRotateVoltage", rotateMotor.getMotorOutputVoltage());
    SmartDashboard.putData("algaeRotatePID", algaeRotatePID);
    SmartDashboard.putBoolean("isManualControl", isManualControl);
    if (powerDistribution.isAlgaeRotateOverCurrent()) {
      rotateMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
    double output = algaeRotatePID.calculate(rotateEncoder.get() * 360);
    rotateMotor.set(ControlMode.PercentOutput, output);
  }

  public Command setIntakeMotorFastOnCmd() {
    Command cmd = runEnd(this::setIntakeMotorFastOn, this::stopIntakeMotor);
    cmd.setName("setIntakeCmd");
    return cmd;
  }

  public Command setIntakeMotorSlowOnCmd() {
    Command cmd = runEnd(this::setIntakeMotorSlowOn, this::stopIntakeMotor);
    cmd.setName("setIntakeCmd");
    return cmd;
  }

  public Command reIntakeCmd() { // 吐出 algae 的 cmd
    Command cmd = runEnd(this::setReIntake, this::stopIntakeMotor);
    cmd.setName("setReIntakeMotorCmd");
    return cmd;
  }

  public Command setRotateCmd(double speed) { // 吐出 algae 的 cmd
    Command cmd = runEnd(() -> setRotate(speed), this::stopRotate);
    cmd.setName("manualSetRotateCmd");
    return cmd;
  }

  public Command moveRotateToAngleCmd() {
    Command cmd = runEnd(this::moveRotateToAngle, this::stopRotate);
    return cmd;
  }

  public Command rotateUpCmd() {
    return setRotateCmd(AlgaeIntakeConstant.kUpIntakeRotateSpeed);
  }

  public Command rotateDownCmd() {
    return setRotateCmd(AlgaeIntakeConstant.kDownIntakeRotateSpeed);
  }

}
