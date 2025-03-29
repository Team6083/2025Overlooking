// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveControlConstant;
import frc.robot.PreferencesClass.Magnification;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;

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

  private final ElevatorSubsystem elevatorSubsystem;

  private double magnification;
  private double rotMagnification;
  private final double drivebaseMaxSpeed = SwerveControlConstant.kDrivebaseMaxSpeed;
  private final double minJoystickInput = SwerveControlConstant.kMinJoystickInput;
  private final Supplier<Boolean> elevatorBypassSafety;

  public SwerveControlCmd(SwerveDrive swerveDrive, CommandXboxController mainController,
      ElevatorSubsystem elevatorSubsystem, Supplier<Boolean> elevatorBypassSafety) {
    this.swerveDrive = swerveDrive;
    this.mainController = mainController;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorBypassSafety = elevatorBypassSafety;
    xLimiter = new SlewRateLimiter(SwerveControlConstant.kXLimiterRateLimit);
    yLimiter = new SlewRateLimiter(SwerveControlConstant.kYLimiterRateLimit);
    rotLimiter = new SlewRateLimiter(SwerveControlConstant.kRotLimiterRateLimit);
    addRequirements(this.swerveDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSubsystem.getCurrentHeight().gt(Millimeters.of(545)) && !elevatorBypassSafety.get()) {
      magnification = Magnification.get("kSafeMagnification");
      rotMagnification = Magnification.get("kRotSafeMagnification");
    } else if (mainController.leftBumper().getAsBoolean()) {
      magnification = Magnification.get("kFastMagnification");
      rotMagnification = Magnification.get("kRotFastMagnification");
    } else {
      magnification = Magnification.get("kSafeMagnification");
      rotMagnification = Magnification.get("kRotSafeMagnification");
    }
    // CHECKSTYLE.OFF: LocalVariableName
    double xSpeed;
    double ySpeed;
    double rotSpeed;
    // CHECKSTYLE.ON: LocalVariableName

    if (Math.abs(mainController.getLeftY()) > minJoystickInput) {
      xSpeed = -xLimiter.calculate(mainController.getLeftY())
          * drivebaseMaxSpeed * magnification;

    } else {
      xSpeed = 0;
    }
    if (Math.abs(mainController.getLeftX()) > minJoystickInput) {
      ySpeed = -yLimiter.calculate(mainController.getLeftX())
          * drivebaseMaxSpeed * magnification;

    } else {
      ySpeed = 0;
    }
    if (Math.abs(mainController.getRightX()) > minJoystickInput) {
      rotSpeed = -rotLimiter.calculate(mainController.getRightX())
          * drivebaseMaxSpeed * rotMagnification;

    } else {
      rotSpeed = 0;
    }
    swerveDrive.drive(
        xSpeed, ySpeed, rotSpeed, SwerveControlConstant.kFieldRelative);

    SmartDashboard.putNumber("LeftX()", mainController.getLeftX());
    SmartDashboard.putNumber("LeftY", mainController.getLeftY());
    SmartDashboard.putNumber("RightX", mainController.getRightX());
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
