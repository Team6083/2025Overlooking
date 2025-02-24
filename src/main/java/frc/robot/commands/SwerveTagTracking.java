// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.subsystems.TagTrackingSubsystem;

  public class SwerveTagTracking extends Command {
  SwerveDrive swerveDrive;
  TagTrackingSubsystem tagTracking = new TagTrackingSubsystem();
  PIDController txController = new PIDController(0.1, 0, 0);
  PIDController tyController = new PIDController(0.1, 0, 0);
  double xSpeed;
  double ySpeed;
  public SwerveTagTracking(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
    // Use addRequirement
  }

  // Called when the command is initially scheduled.
  @Override
  
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (tagTracking.getTv() == 1) {
      xSpeed = tyController.calculate(tagTracking.getTy(), 0);
      ySpeed = txController.calculate(tagTracking.getTx(), 0);
      // swerveDrive.drive(xSpeed, ySpeed, 0, false);
    } else {
      xSpeed = 0;
      ySpeed = 0;
    }
    // swerveDrive.drive(xSpeed, ySpeed, 0, false);
    SmartDashboard.putNumber("tx", tagTracking.getTx());
    SmartDashboard.putNumber("ty", tagTracking.getTy());
    SmartDashboard.putNumber("tv", tagTracking.getTv());
    SmartDashboard.putNumber("TagTrackingXSpeed", xSpeed);
    SmartDashboard.putNumber("TagTrackingYSpeed", ySpeed);
    SmartDashboard.putData("tyController",tyController);
    SmartDashboard.putData("txController",txController);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return txController.getError() < 0.01 && tyController.getError() < 0.01;
  }
}