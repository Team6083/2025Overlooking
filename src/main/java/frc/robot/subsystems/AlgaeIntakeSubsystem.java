// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
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
  private final Encoder rotateEncoder;
  private final PowerDistribution powerDistribution;
  private boolean isManualControl = false;

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

    rotateEncoder = new Encoder(AlgaeIntakeConstant.kAlgaeEncoderChannelA,
        AlgaeIntakeConstant.kAlgaeEncoderChannelB);
    rotateEncoder.setDistancePerPulse(AlgaeIntakeConstant.kDistancePerPulse);
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

  public Boolean isOverLimit() {
    return powerDistribution.algaeIntakeCurrent() <= AlgaeIntakeConstant.kIntakeCurrentLimit;
  }

  public void manualSetRotate(double speed) {
    rotateMotor.set(VictorSPXControlMode.PercentOutput, speed);

  }

  public void stopRotate() {
    rotateMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void setManualControl(boolean maunnalControlOn) {
    this.isManualControl = maunnalControlOn;

  }

  public boolean getIsMaunnalControl() {
    return isManualControl;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("algaeRotateDistance", rotateEncoder.getDistance());
    SmartDashboard.putNumber("algaeIntakeVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("algaeRotateVoltage", rotateMotor.getMotorOutputVoltage());
    SmartDashboard.putBoolean("isOverLimit", isOverLimit());
    SmartDashboard.putData("algaeRotatePID", algaeRotatePID);
    SmartDashboard.putBoolean("isManualControl", isManualControl);
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

  public Command autoIntakeCmd() { // 吸入 algae 的 cmd
    Command cmd = run(this::setIntakeMotorFastOn)
        .until(this::isOverLimit)
        .andThen(this.runEnd(this::setIntakeMotorSlowOn, this::stopIntakeMotor));
    cmd.setName("setAutoIntakeCmd");
    return cmd;
  }

  public Command reIntakeCmd() { // 吐出 algae 的 cmd
    Command cmd = runEnd(this::setReIntake, this::stopIntakeMotor);
    cmd.setName("setReIntakeMotorCmd");
    return cmd;
  }

  public Command manualSetRotateCmd(double speed) { // 吐出 algae 的 cmd
    Command cmd = runEnd(() -> manualSetRotate(speed), this::stopRotate);
    cmd.setName("manualSetRotateCmd");
    return cmd;
  }

  public Command manualUpRotateCmd() {
    return manualSetRotateCmd(AlgaeIntakeConstant.kUpIntakeRotateSpeed);
  }

  public Command manualDownRotateCmd() {
    return manualSetRotateCmd(AlgaeIntakeConstant.kDownIntakeRotateSpeed);
  }

}
