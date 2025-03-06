// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.TagTracking;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveToTagCmd extends Command {
  /** Creates a new SwerveTrackingCmd. */
  SwerveDrive swerveDrive;
  TagTracking tagTracking = new TagTracking();
  PIDController txController = new PIDController(2, 0, 0.5);
  PIDController tzController = new PIDController(2, 0, 0);
  PIDController yawController = new PIDController(0.05, 0, 0);
  Boolean isLeft;

  public SwerveToTagCmd(SwerveDrive swerveDrive, Boolean isLeft) {
    this.swerveDrive = swerveDrive;
    this.isLeft = isLeft;
    if (isLeft) {
      txController.setSetpoint(0.17);
    } else {
      txController.setSetpoint(-0.17);
    }
    tzController.setSetpoint(0.43);
    yawController.setSetpoint(0);
    addRequirements(swerveDrive);
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
    if (isLeft) {
      txController.setSetpoint(0.17);
    } else {
      txController.setSetpoint(-0.17);
    }
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
    SmartDashboard.putData("TzController", tzController);
    SmartDashboard.putData("TxController", txController);
    SmartDashboard.putData("YawController", yawController);
    SmartDashboard.putNumber("LeftSetpoint", txController.getSetpoint());
    SmartDashboard.putBoolean("IsLeft", isLeft);
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
    || (Math.abs(txController.getError()) < 0.05 && Math.abs(tzController.getError()) < 0.05);
  }
}