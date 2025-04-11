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
import frc.robot.ConfigChooser;
import frc.robot.Constants.AlgaeIntakeConstant;
import java.util.function.Supplier;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new ALGAEIntakeSubsystem. */
  private final VictorSPX intakeMotor;
  private final VictorSPX rotateMotor;

  private final DutyCycleEncoder rotateEncoder;

  private final PIDController algaeRotatePID;

  private final Supplier<Boolean> shouldUsePIDSupplier;

  private boolean isManualControl = false;

  public AlgaeIntakeSubsystem(Supplier<Boolean> shouldUsePIDSupplier) {
    this.shouldUsePIDSupplier = shouldUsePIDSupplier;

    intakeMotor = new VictorSPX(AlgaeIntakeConstant.kIntakeMotorChannel);
    rotateMotor = new VictorSPX(AlgaeIntakeConstant.kRotateMotorChannel);

    intakeMotor.setInverted(AlgaeIntakeConstant.kIntakeMotorInverted);
    rotateMotor.setInverted(ConfigChooser.AlgaeIntake.getBoolean("kRotateMotorInverted"));

    rotateEncoder = new DutyCycleEncoder(
        AlgaeIntakeConstant.kAlgaeEncoderChannel,
        AlgaeIntakeConstant.fullRange,
        ConfigChooser.AlgaeIntake.getDouble("expectedZero"));
    rotateEncoder.setInverted(AlgaeIntakeConstant.kAlgaeEncoderInverted);

    algaeRotatePID = new PIDController(0, 0, 0);
    algaeRotatePID.enableContinuousInput(0, 360);
  }

  public void intake() {
    intakeMotor.set(ControlMode.PercentOutput,
        ConfigChooser.AlgaeIntake.getDouble("kIntakeFastSpeed"));
  }

  public void reverseIntake() {
    intakeMotor.set(ControlMode.PercentOutput,
        ConfigChooser.AlgaeIntake.getDouble("kReverseIntakeSpeed"));
  }

  public void stopIntakeMotor() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void manualSetRotate(double speed) {
    rotateMotor.set(ControlMode.PercentOutput, speed);
    algaeRotatePID.setSetpoint(rotateEncoder.get());
  }

  public void toDefaultDegree() {
    algaeRotatePID.setSetpoint(0);
  }

  public void toAlgaeIntakeDegree() {
    algaeRotatePID.setSetpoint(ConfigChooser.AlgaeIntake.getDouble("kGetAlgaeAngle"));
  }

  public void toTakeAlgaeFromReefDegree() {
    algaeRotatePID.setSetpoint(ConfigChooser.AlgaeIntake.getDouble("kTakeAlgaeFromReefAngle"));
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
    var usePID = shouldUsePIDSupplier.get();

    if (usePID && !isManualControl) {
      if (getCurrentAngle() > getRotateSetpoint()) {
        algaeRotatePID.setPID(
            ConfigChooser.AlgaeIntake.getDouble("rotMotorUpPIDkP"),
            ConfigChooser.AlgaeIntake.getDouble("rotMotorUpPIDkI"),
            ConfigChooser.AlgaeIntake.getDouble("rotMotorUpPIDkD"));
      } else {
        algaeRotatePID.setPID(
            ConfigChooser.AlgaeIntake.getDouble("rotMotorDownPIDkP"),
            ConfigChooser.AlgaeIntake.getDouble("rotMotorDownPIDkI"),
            ConfigChooser.AlgaeIntake.getDouble("rotMotorDownPIDkD"));
      }
      double output = algaeRotatePID.calculate(getCurrentAngle());
      output = MathUtil.clamp(output, AlgaeIntakeConstant.kMinOutput, AlgaeIntakeConstant.kMaxOutput);

      rotateMotor.set(ControlMode.PercentOutput, output);

      SmartDashboard.putNumber("AlgaeRotateOutput", output);
    } else {
      algaeRotatePID.setSetpoint(getCurrentAngle());
    }

    SmartDashboard.putNumber("AlgaeIntakeVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("AlgaeRotateVoltage", rotateMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("AlgaeRotateEncoder", rotateEncoder.get());

    SmartDashboard.putBoolean("AlgaeRotateUsePID", usePID);

    SmartDashboard.putData("AlgaeRotatePID", algaeRotatePID);
  }

  public Command intakeCmd() {
    Command cmd = runEnd(this::intake, this::stopIntakeMotor);
    cmd.setName("setIntakeCmd");
    return cmd;
  }

  public Command reverseIntakeCmd() {
    Command cmd = runEnd(this::reverseIntake, this::stopIntakeMotor);
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
    return setRotateCmd(ConfigChooser.AlgaeIntake.getDouble("kUpIntakeRotateSpeed"));
  }

  public Command manualRotateDownCmd() {
    return setRotateCmd(ConfigChooser.AlgaeIntake.getDouble("kDownIntakeRotateSpeed"));
  }

  public Command toDefaultDegreeCmd() {
    Command cmd = runOnce(this::toDefaultDegree);
    cmd.setName("toDefaultDegreeCmd");
    return cmd;
  }

  public Command toAlgaeIntakeDegreeCmd() {
    Command cmd = runOnce(this::toAlgaeIntakeDegree);
    cmd.setName("toAlgaeIntakeDegreeCmd");
    return cmd;
  }

  public Command toTakeAlgaeFromReefDegreeCmd() {
    Command cmd = runOnce(this::toTakeAlgaeFromReefDegree);
    cmd.setName("toTakeAlgaeFromReefDegreeCmd");
    return cmd;
  }

  public double getAbsoluteError() {
    return Math.abs(algaeRotatePID.getError());
  }
}
