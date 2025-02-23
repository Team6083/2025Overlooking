// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralShooterHoldCmd extends Command {
  /** Creates a new CoralShooterHoldCmd. */
  private double encoderTargetDegree = 0;
  private PIDController pidController = new PIDController(0.003, 0.0, 0.0);
  private CoralShooterSubsystem coralShooterSubsystem;

  public CoralShooterHoldCmd(CoralShooterSubsystem coralShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralShooterSubsystem = coralShooterSubsystem;
    addRequirements(this.coralShooterSubsystem);
    pidController.enableContinuousInput(0, 360);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderTargetDegree = coralShooterSubsystem.getEncoder();
    // record which degree the motor should be kept in
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderCurrentDegree = coralShooterSubsystem.getEncoder();
    double speed = MathUtil.clamp(pidController.calculate(encoderCurrentDegree, encoderTargetDegree), -0.2, 0.2);
    coralShooterSubsystem.setMotorSpeed(speed);
    SmartDashboard.putNumber("TargetDegree", encoderTargetDegree);
    SmartDashboard.putNumber("HoldSpeed", speed);
    SmartDashboard.putNumber("CoralShooterEncoder", encoderCurrentDegree);
    SmartDashboard.putData("coralHoldPID", pidController);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralShooterSubsystem.coralShooterStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
