// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveControlConstant;
import frc.robot.drivebase.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveControlCmd extends Command {
  /** Creates a new SwerveJoystickCmd. */
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;

  // CHECKSTYLE.OFF: MemberName
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  // CHECKSTYLE.ON: MemberName
  private final SlewRateLimiter rotLimiter;

  // CHECKSTYLE.OFF: MemberName
  private double xSpeed;
  private double ySpeed;
  // CHECKSTYLE.ON: MemberName
  private double rotSpeed;

  // max magnification is 2.0
  private final double magnification = SwerveControlConstant.kMagnification;
  private final double drivebaseMaxSpeed = SwerveControlConstant.kDrivebaseMaxSpeed;
  private final double MinJoystickInput = SwerveControlConstant.kMinJoystickInput;

  public SwerveControlCmd(SwerveDrive swerveDrive, CommandXboxController mainController) {
    this.swerveDrive = swerveDrive;
    this.mainController = mainController;
    xLimiter = new SlewRateLimiter(SwerveControlConstant.kXLimiterRateLimit);
    yLimiter = new SlewRateLimiter(SwerveControlConstant.kYLimiterRateLimit);
    rotLimiter = new SlewRateLimiter(SwerveControlConstant.kRotLimiterRateLimit);
    addRequirements(this.swerveDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(mainController.getLeftY()) > MinJoystickInput) {
      xSpeed = -xLimiter.calculate(mainController.getLeftY())
          * drivebaseMaxSpeed * magnification;

    } else {
      xSpeed = 0;
    }
    if (Math.abs(mainController.getLeftX()) > MinJoystickInput) {
      ySpeed = -yLimiter.calculate(mainController.getLeftX())
          * drivebaseMaxSpeed * magnification;

    } else {
      ySpeed = 0;
    }
    if (Math.abs(mainController.getRightX()) > MinJoystickInput) {
      rotSpeed = rotLimiter.calculate(mainController.getRightX())
          * drivebaseMaxSpeed * 1.2;

    } else {
      rotSpeed = 0;
    }

    swerveDrive.drive(
        xSpeed, ySpeed, rotSpeed, SwerveControlConstant.kFieldRelative);
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
