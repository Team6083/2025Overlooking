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
  private final CommandXboxController main;
  private final SlewRateLimiter xlimiter;
  private final SlewRateLimiter ylimiter;
  private final SlewRateLimiter rotLimiter;
  private final double drivebaseMaxSpeed = DriveBaseConstants.kMaxSpeed;
  private double xspeed;
  private double yspeed;
  private double rotSpeed;
  private double magnification;

  public SwerveJoystickCmd(SwerveDrive swerveDrive, CommandXboxController main) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrive = swerveDrive;
    this.main = main;
    xlimiter = new SlewRateLimiter(DriveBaseConstants.kXLimiterRateLimit);
    ylimiter = new SlewRateLimiter(DriveBaseConstants.kYLimiterRateLimit);
    rotLimiter = new SlewRateLimiter(DriveBaseConstants.kRotLimiterRateLimit);
    addRequirements(this.swerveDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    magnification = DriveBaseConstants.kmagnification;
    if (Math.abs(main.getLeftY()) > 0.1) {
      xspeed = -xlimiter.calculate(main.getLeftY()) * drivebaseMaxSpeed * magnification;
    } else {
      xspeed = 0;
    }
    if (Math.abs(main.getLeftX()) > 0.1) {
      yspeed = -ylimiter.calculate(main.getLeftX()) * drivebaseMaxSpeed * magnification;
    } else {
      yspeed = 0;
    }
    if (Math.abs(main.getRightX()) > 0.1) {
      rotSpeed = -rotLimiter.calculate(main.getRightX()) * drivebaseMaxSpeed * 1.2;
    } else {
      rotSpeed = 0;
    }
    swerveDrive.drive(xspeed, yspeed, rotSpeed, true);
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
