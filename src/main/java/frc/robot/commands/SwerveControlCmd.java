// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ConfigChooser;
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

  private double magnification;
  private double rotMagnification;
  private final double driveBaseMaxSpeed = SwerveControlConstant.kDrivebaseMaxSpeed;
  private final double minJoystickInput = SwerveControlConstant.kMinJoystickInput;

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
    if (mainController.leftBumper().getAsBoolean()) {
      magnification = ConfigChooser.SwerveControl.getDouble("kFastMagnification");
      rotMagnification = ConfigChooser.SwerveControl.getDouble("kRotFastMagnification");
    } else {
      magnification = ConfigChooser.SwerveControl.getDouble("kDefaultMagnification");
      rotMagnification = ConfigChooser.SwerveControl.getDouble("kRotDefaultMagnification");
    }
    // CHECKSTYLE.OFF: LocalVariableName
    double xSpeed;
    double ySpeed;
    double rotSpeed;
    // CHECKSTYLE.ON: LocalVariableName

    if (Math.abs(mainController.getLeftY()) > minJoystickInput) {
      xSpeed = xLimiter.calculate(mainController.getLeftY())
          * driveBaseMaxSpeed * magnification;

    } else {
      xSpeed = 0;
    }
    if (Math.abs(mainController.getLeftX()) > minJoystickInput) {
      ySpeed = yLimiter.calculate(mainController.getLeftX())
          * driveBaseMaxSpeed * magnification;

    } else {
      ySpeed = 0;
    }
    if (Math.abs(mainController.getRightX()) > minJoystickInput) {
      rotSpeed = rotLimiter.calculate(mainController.getRightX())
          * driveBaseMaxSpeed * rotMagnification;

    } else {
      rotSpeed = 0;
    }
    swerveDrive.drive(
        xSpeed, ySpeed, rotSpeed, SwerveControlConstant.kFieldRelative);

    SmartDashboard.putNumber("XSpeed", xSpeed);
    SmartDashboard.putNumber("YSpeed", ySpeed);
    SmartDashboard.putNumber("RotSpeed", rotSpeed);
    SmartDashboard.putNumber("DrivebaseMagnification", magnification);

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
