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
public class SwerveToTagRightCmd extends Command {
  /** Creates a new SwerveTrackingCmd. */
  SwerveDrive swerveDrive;
  TagTrackingSubsystem tagTracking;
  PIDController txController = new PIDController(2, 0, 0.5);
  PIDController tzController = new PIDController(2, 0, 0);
  PIDController yawController = new PIDController(0.05, 0, 0);

  public SwerveToTagRightCmd(SwerveDrive swerveDrive, TagTrackingSubsystem tagTracking) {
    this.swerveDrive = swerveDrive;
    this.tagTracking = tagTracking;
    txController.setSetpoint(0.14);
    tzController.setSetpoint(0.43);
    yawController.setSetpoint(0);
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
    double rotSpeed;
    xSpeed = -tzController.calculate(tagTracking.getTr()[2]);
    ySpeed = txController.calculate(tagTracking.getTr()[0]);
    rotSpeed = yawController.calculate(tagTracking.getTr()[4]);
    xSpeed = MathUtil.clamp(xSpeed, -2, 2);
    ySpeed = MathUtil.clamp(ySpeed, -2, 2);
    swerveDrive.drive(xSpeed, ySpeed, rotSpeed, false);
    // CHECKSTYLE.ON: LocalVariableName

    SmartDashboard.putNumber("TagTrackingXSpeed", xSpeed);
    SmartDashboard.putNumber("TagTrackingYSpeed", ySpeed);
    SmartDashboard.putNumber("TagTrackingRotSpeed", rotSpeed);
    SmartDashboard.putData("tzController", tzController);
    SmartDashboard.putData("txController", txController);
    SmartDashboard.putData("yawController", yawController);
    SmartDashboard.putNumber("LeftSetpoint", txController.getSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tagTracking.getTv() == 0
        || (Math.abs((txController.getError())) < 0.1
            && Math.abs(tzController.getError()) < 0.1);
  }
}