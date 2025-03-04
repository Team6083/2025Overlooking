// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  public AlgaeIntakeSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;

    // motor
    intakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateMotor = new VictorSPX(AlgaeIntakeConstant.kRotateMotorChannel);
    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    rotateMotor.setInverted(AlgaeIntakeConstant.kRotateMotorInverted);

    // encoder
    rotateEncoder = new DutyCycleEncoder(
        AlgaeIntakeConstant.kAlgaeEncoderChannelA,
        AlgaeIntakeConstant.fullRange,
        AlgaeIntakeConstant.expectedZero);
    rotateEncoder.setInverted(AlgaeIntakeConstant.kAlgaeEncoderInverted);

    // PID
    algaeRotatePID = new PIDController(0, 0, 0);
    algaeRotatePID.enableContinuousInput(0, 360);
  }

  private void setIntakeMotor(double speed) {
    if (powerDistribution.isAlgaeIntakeOverCurrent()) {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    } else {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }
  }

  public void setIntakeMotorFastOn() {
    setIntakeMotor(AlgaeIntakeConstant.kIntakeFastSpeed);

  }

  public void setIntakeMotorSlowOn() {
    setIntakeMotor(AlgaeIntakeConstant.kIntakeSlowSpeed);
  }

  public void setReIntake() {
    setIntakeMotor(AlgaeIntakeConstant.kReIntakeSpeed);
  }

  public void stopIntakeMotor() {
    setIntakeMotor(0);
  }

  public void manualSetRotate(double speed) {
    rotateMotor.set(VictorSPXControlMode.PercentOutput, speed);
    algaeRotatePID.setSetpoint(rotateEncoder.get());
  }

  public void stopRotate() {
    rotateMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void setRotateSetpoint(double setpoint) {
    isManualControl = false;
    algaeRotatePID.setSetpoint(setpoint);
    double output = -algaeRotatePID.calculate(getCurrentAngle());
    output = MathUtil.clamp(output, -0.5, 0.5);
    rotateMotor.set(VictorSPXControlMode.PercentOutput, -output);
  }

  public double getRotateSetpoint() {
    return algaeRotatePID.getSetpoint();
  }

  public double getCurrentAngle() {
    return rotateEncoder.get();
  }

  public void setManualControl() {
    isManualControl = true;
  }

  public void setRotateUpPID() {
    algaeRotatePID.setPID(
        AlgaeIntakeConstant.rotMotorUpPIDkP,
        AlgaeIntakeConstant.rotMotorUpPIDkI,
        AlgaeIntakeConstant.rotMotorUpPIDkD);
  }

  public void setRotateDownPID() {
    algaeRotatePID.setPID(
        AlgaeIntakeConstant.rotMotorDownPIDkP,
        AlgaeIntakeConstant.rotMotorDownPIDkI,
        AlgaeIntakeConstant.rotMotorDownPIDkD);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("algaeIntakeVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("algaeRotateVoltage", rotateMotor.getMotorOutputVoltage());
    SmartDashboard.putData("algaeRotatePID", algaeRotatePID);
    SmartDashboard.putBoolean("algaeRotateIsManualControl", isManualControl);
    SmartDashboard.putNumber("algaeEncoderAngle", rotateEncoder.get());

    if (!isManualControl) {
      double output = -algaeRotatePID.calculate(getCurrentAngle());
      output = MathUtil.clamp(output, -0.5, 0.5);
      rotateMotor.set(VictorSPXControlMode.PercentOutput, -output);
      SmartDashboard.putNumber("algaeOutput", output);
    } else {
      algaeRotatePID.setSetpoint(getCurrentAngle());

      if (getCurrentAngle() < getRotateSetpoint()) {
        algaeRotatePID.setP(AlgaeIntakeConstant.rotMotorUpPIDkP);
      } else {
        algaeRotatePID.setP(AlgaeIntakeConstant.rotMotorDownPIDkP);
      }
    }
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

  public Command setManualControlCmd() {
    Command cmd = runOnce(
        () -> setManualControl());
    cmd.setName("setManualControl");
    return cmd;
  }

  public Command toDefaultDegreeCmd() {
    setRotateUpPID();
    Command cmd = runOnce(
        () -> setRotateSetpoint(0));
    cmd.setName("toDefaultDegreeCmd");
    return cmd;
  }

  public Command toAlgaeIntakeDegreeCmd() {
    setRotateDownPID();
    Command cmd = runOnce(
        () -> setRotateSetpoint(AlgaeIntakeConstant.kGetSecAlgaeAngle));
    cmd.setName("toAlgaeIntakeDegreeCmd");
    return cmd;
  }

  public Command autoStopRotateCmd(Command command) {
    Command cmd = new SequentialCommandGroup(
        command.repeatedly()
            .until(() -> Math.abs(algaeRotatePID.getError()) < 5),
        runOnce(this::stopRotate));
    cmd.setName("autoStopRotateCmd");
    return cmd;
  }
}
