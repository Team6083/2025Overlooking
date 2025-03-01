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
  private boolean isManualControl = true;
  private final DutyCycleEncoder rotateEncoder;
  public AlgaeIntakeSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;
    algaeRotatePID = new PIDController(
        AlgaeIntakeConstant.rotMotorPIDkP,
        AlgaeIntakeConstant.rotMotorPIDkI,
        AlgaeIntakeConstant.rotMotorPIDkD);
    algaeRotatePID.enableContinuousInput(0, 360);
    intakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateMotor = new VictorSPX(AlgaeIntakeConstant.kRotateMotorChannel);
    rotateEncoder = new DutyCycleEncoder(
        AlgaeIntakeConstant.kAlgaeEncoderChannelA,
        AlgaeIntakeConstant.fullRange,
        AlgaeIntakeConstant.expectedZero);
    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    rotateMotor.setInverted(AlgaeIntakeConstant.kRotateMotorInverted);
    rotateEncoder.setInverted(AlgaeIntakeConstant.kAlgaeEncoderInverted);
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
    setpoint = MathUtil.clamp(setpoint, AlgaeIntakeConstant.kMaxAngle, AlgaeIntakeConstant.kMinAngle);
  }
  public double getCurrentAngle() {
    return rotateEncoder.get();
  }
  public void moveToAngle() {
    algaeRotatePID.setSetpoint(AlgaeIntakeConstant.kGetSecAlgaeAngle);
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
      rotateMotor.set(VictorSPXControlMode.PercentOutput, output);
      SmartDashboard.putNumber("algaeOutput", output);
    } else {
      algaeRotatePID.setSetpoint(getCurrentAngle());
    }
  }
  public Command moveToAngleCmd() {
    Command cmd = runOnce(this::moveToAngle);
    cmd.setName("moveToAngleCmd");
    return cmd;
  }
  public Command setIntakeMotorFastOnCmd() {
    Command cmd = runEnd(this::moveToAngle, this::stopRotate);
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
  public Command rotateUpPIDCmd() {
    Command cmd = runOnce(
        () -> setRotateSetpoint(AlgaeIntakeConstant.kStepAngle));
    cmd.setName("rotateUpPID");
    return cmd;
  }
  public Command rotateDownPIDCmd() {
    Command cmd = runOnce(
        () -> setRotateSetpoint(-AlgaeIntakeConstant.kStepAngle));
    cmd.setName("rotateDownPID");
    return cmd;
  }
  public Command rotateMaxPIDCmd() {
    Command cmd = runOnce(
        () -> setRotateSetpoint(AlgaeIntakeConstant.kMaxAngle));
    cmd.setName("rotateUpPID");
    return cmd;
  }
  public Command rotateMinPIDCmd() {
    Command cmd = runOnce(
        () -> setRotateSetpoint(AlgaeIntakeConstant.kMinAngle));
    cmd.setName("rotateDownPID");
    return cmd;
  }
  public Command autoStopRotateCmd(Command command) {
    Command cmd = new SequentialCommandGroup(
        command.repeatedly()
            .until(() -> algaeRotatePID.getError() < 5),
        runOnce(this::stopRotate));
    cmd.setName("autoStopRotateCmd");
    return cmd;
  }
  // public boolean isRotationFinished() {
  // return algaeRotatePID.getError() < 5;
  // }
}




















