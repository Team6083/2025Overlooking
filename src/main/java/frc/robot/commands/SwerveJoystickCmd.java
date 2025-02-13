// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveBaseConstants;
import frc.robot.drivebase.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveJoystickCmd. */
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter rotLimiter;
  private final double drivebaseMaxSpeed = DriveBaseConstants.kMaxSpeed;
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;
  private double magnification;

  public SwerveJoystickCmd(SwerveDrive swerveDrive, CommandXboxController mainController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrive = swerveDrive;
    this.mainController = mainController;
    xLimiter = new SlewRateLimiter(DriveBaseConstants.kXLimiterRateLimit);
    yLimiter = new SlewRateLimiter(DriveBaseConstants.kYLimiterRateLimit);
    rotLimiter = new SlewRateLimiter(DriveBaseConstants.kRotLimiterRateLimit);
    addRequirements(this.swerveDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    magnification = DriveBaseConstants.kMagnification;
    if (Math.abs(mainController.getLeftY()) > 0.1) {
      xSpeed = -xLimiter.calculate(mainController.getLeftY()) * drivebaseMaxSpeed * magnification;
    } else {
      xSpeed = 0;
    }
    if (Math.abs(mainController.getLeftX()) > 0.1) {
      ySpeed = -yLimiter.calculate(mainController.getLeftX()) * drivebaseMaxSpeed * magnification;
    } else {
      ySpeed = 0;
    }
    if (Math.abs(mainController.getRightX()) > 0.1) {
      rotSpeed = -rotLimiter.calculate(mainController.getRightX()) * drivebaseMaxSpeed * 1.2;
    } else {
      rotSpeed = 0;
    }
    swerveDrive.drive(xSpeed, ySpeed, rotSpeed, DriveBaseConstants.kFieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
