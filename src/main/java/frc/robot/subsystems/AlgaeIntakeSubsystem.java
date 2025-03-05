// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstant;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSPX intakeMotor;
  private final VictorSPX rotateMotor;
  private final PIDController algaeRotatePID;
  private boolean isManualControl = false;
  private boolean isPIDEnabled = true;
  private final DutyCycleEncoder rotateEncoder;

  public AlgaeIntakeSubsystem() {
    algaeRotatePID = new PIDController(0, 0, 0);
    algaeRotatePID.enableContinuousInput(0, 360);
    intakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateMotor = new VictorSPX(AlgaeIntakeConstant.kRotateMotorChannel);
    rotateEncoder = new DutyCycleEncoder(
        AlgaeIntakeConstant.kAlgaeEncoderChannel,
        AlgaeIntakeConstant.fullRange,
        AlgaeIntakeConstant.expectedZero);
    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    rotateMotor.setInverted(AlgaeIntakeConstant.kRotateMotorInverted);
    rotateEncoder.setInverted(AlgaeIntakeConstant.kAlgaeEncoderInverted);
  }

  public void setIntakeMotorOn() {
    intakeMotor.set(ControlMode.PercentOutput,
        AlgaeIntakeConstant.kIntakeFastSpeed);
  }

  public void setReIntake() {
    intakeMotor.set(ControlMode.PercentOutput,
        AlgaeIntakeConstant.kReIntakeSpeed);
  }

  public void stopIntakeMotor() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void manualSetRotate(double speed) {
    rotateMotor.set(ControlMode.PercentOutput, speed);
    algaeRotatePID.setSetpoint(rotateEncoder.get());
  }

  public void setRotateSetpoint(double setpoint) {
    algaeRotatePID.setSetpoint(setpoint);
    setpoint = MathUtil.clamp(setpoint,AlgaeIntakeConstant.kMaxAngle,AlgaeIntakeConstant.kMinAngle);
    double output = -algaeRotatePID.calculate(getCurrentAngle());
    output = MathUtil.clamp(output, AlgaeIntakeConstant.kMinOutput, AlgaeIntakeConstant.kMaxOutput);
    rotateMotor.set(ControlMode.PercentOutput, -output);
  }

  public void stopRotate() {
    rotateMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getRotateSetpoint() {
    return algaeRotatePID.getSetpoint();
  }

  public double getCurrentAngle() {
    return rotateEncoder.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("algaeIntakeVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("algaeRotateVoltage", rotateMotor.getMotorOutputVoltage());
    SmartDashboard.putData("algaeRotatePID", algaeRotatePID);
    SmartDashboard.putBoolean("algaeRotateIsManualControl", isManualControl);
    SmartDashboard.putNumber("algaeEncoderAngle", rotateEncoder.get());
    SmartDashboard.putBoolean("isPIDEnabled",isPIDEnabled);
    
    if (!isManualControl && isPIDEnabled) {
      if (getCurrentAngle() < getRotateSetpoint()) {
        algaeRotatePID.setPID(
          AlgaeIntakeConstant.kPRotateUp,
          AlgaeIntakeConstant.kIRotateUp,
          AlgaeIntakeConstant.kDRotateUp);
      } else {
        algaeRotatePID.setPID(
          AlgaeIntakeConstant.kPRotateDown,
          AlgaeIntakeConstant.kIRotateDown,
          AlgaeIntakeConstant.kDRotateDown);
      }
      double output = algaeRotatePID.calculate(getCurrentAngle());
      output = MathUtil.clamp(output,AlgaeIntakeConstant.kMinOutput,AlgaeIntakeConstant.kMaxOutput);
      rotateMotor.set(ControlMode.PercentOutput, output);
      SmartDashboard.putNumber("algaeOutput", output);
    } else {
      algaeRotatePID.setSetpoint(getCurrentAngle());
    }
  }

  public Command setIntakeMotorOnCmd() {
    Command cmd = runEnd(this::setIntakeMotorOn, this::stopIntakeMotor);
    cmd.setName("setIntakeCmd");
    return cmd;
  }


  public Command reIntakeCmd() { 
    Command cmd = runEnd(this::setReIntake, this::stopIntakeMotor);
    cmd.setName("setReIntakeMotorCmd");
    return cmd;
  }

  public Command setRotateCmd(double speed) { 
    Command cmd = runEnd(
        () -> {
          isManualControl = true;
          manualSetRotate(speed);
        },
        () -> {
            isManualControl = false;
          stopRotate();
        });
    cmd.setName("manualSetRotateCmd");
    return cmd;
  }

  public Command manualRotateUpCmd() {
    return setRotateCmd(AlgaeIntakeConstant.kUpIntakeRotateSpeed);
  }

  public Command manualRotateDownCmd() {
    return setRotateCmd(AlgaeIntakeConstant.kDownIntakeRotateSpeed);
  }

  public Command DisablePID() {
    Command cmd = runOnce(
        () -> {
          isPIDEnabled = false;
        });
    cmd.setName("setManualControl");
    return cmd;
  }

  public Command EnablePID() {
    Command cmd = runOnce(
        () -> {
          isPIDEnabled = true;
        });
    cmd.setName("setPIDControl");
    return cmd;
  }

  public Command toDefaultDegreeCmd() {
    Command cmd = runOnce(
        () -> setRotateSetpoint(0));
    cmd.setName("toDefaultDegreeCmd");
    return cmd;
  }

  public Command toAlgaeIntakeDegreeCmd() {
    Command cmd = runOnce(
        () -> setRotateSetpoint(AlgaeIntakeConstant.kGetSecAlgaeAngle));
    cmd.setName("toAlgaeIntakeDegreeCmd");
    return cmd;
  }

  public double getAbsoluteError() {
    return Math.abs(algaeRotatePID.getError());
  }
}
