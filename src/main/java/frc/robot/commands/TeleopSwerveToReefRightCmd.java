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
public class TeleopSwerveToReefRightCmd extends Command {
  /** Creates a new SwerveTagoTrackingCmd. */
  SwerveDrive swerveDrive;
  TagTrackingSubsystem tagTracking;
  PIDController txController = new PIDController(0.01, 0, 0);

  public TeleopSwerveToReefRightCmd(SwerveDrive swerveDrive, TagTrackingSubsystem tagTracking) {
    this.swerveDrive = swerveDrive;
    this.tagTracking = tagTracking;
    addRequirements(swerveDrive);
    addRequirements(tagTracking);
    // txController.setSetpoint(-0.16);
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
    double ySpeed;
    ySpeed = txController.calculate(tagTracking.getTr()[0],-0.16);
    ySpeed = MathUtil.clamp(ySpeed, -2, 2);
    swerveDrive.drive(0, ySpeed, 0, false);
    // CHECKSTYLE.ON: LocalVariableName

    SmartDashboard.putNumber("TagTrackingYSpeed", ySpeed);
    SmartDashboard.putData("txController", txController);
    SmartDashboard.putNumber("testSetpoint", txController.getSetpoint());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tagTracking.getTv() == 0;
  }
}