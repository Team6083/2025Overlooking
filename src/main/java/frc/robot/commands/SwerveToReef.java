// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.subsystems.TagTrackingSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveToReef extends Command {
  /** Creates a new SwerveTagoTrackingCmd. */
  SwerveDrive swerveDrive;
  TagTrackingSubsystem tagTracking;
  PIDController txController = new PIDController(0.1, 0, 0);
  PIDController tyController = new PIDController(0.1, 0, 0);
  String leftOrRight;

  public SwerveToReef(SwerveDrive swerveDrive, TagTrackingSubsystem tagTracking, String leftOrRight) {
    this.swerveDrive = swerveDrive;
    this.tagTracking = tagTracking;
    this.leftOrRight = leftOrRight;
    addRequirements(swerveDrive);
    addRequirements(tagTracking);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // CHECKSTYLE.OFF: LocalVariableName
    double xSpeed;
    double ySpeed;
    xSpeed = tyController.calculate(tagTracking.getTy(), 3.25);
    if (leftOrRight == "left") {
      txController.setSetpoint(20.9);
    } else {
      txController.setSetpoint(-20.9);
    }
    ySpeed = txController.calculate(tagTracking.getTx());
    xSpeed = MathUtil.clamp(xSpeed, -2, 2);
    ySpeed = MathUtil.clamp(ySpeed, -2, 2);
    swerveDrive.drive(xSpeed, ySpeed, 0, false);
    // CHECKSTYLE.ON: LocalVariableName

    SmartDashboard.putNumber("tx", tagTracking.getTx());
    SmartDashboard.putNumber("ty", tagTracking.getTy());
    SmartDashboard.putNumber("tv", tagTracking.getTv());
    SmartDashboard.putNumber("TagTrackingXSpeed", xSpeed);
    SmartDashboard.putNumber("TagTrackingYSpeed", ySpeed);
    SmartDashboard.putData("tyController", tyController);
    SmartDashboard.putData("txController", txController);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tagTracking.getTv() == 0 ||
        (txController.getError() < 0.5 && tyController.getError() < 0.5);
  }
}