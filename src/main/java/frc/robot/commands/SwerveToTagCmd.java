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
  PIDController txPID = new PIDController(0.1, 0, 0.5);
  PIDController tzPID = new PIDController(0.1, 0, 0);
  PIDController yawPID = new PIDController(0.3, 0, 0);
  Boolean isLeft;

  public SwerveToTagCmd(SwerveDrive swerveDrive, Boolean isLeft) {
    this.swerveDrive = swerveDrive;
    this.isLeft = isLeft;
    tzPID.setSetpoint(0.44);
    yawPID.setSetpoint(0);
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
      txPID.setSetpoint(0.17);
    } else {
      txPID.setSetpoint(-0.16);
    }
    double xSpeed;
    double ySpeed;
    double rotSpeed;
    xSpeed = -tzPID.calculate(tagTracking.get3DTz());
    ySpeed = txPID.calculate(tagTracking.get3DTx());
    rotSpeed = yawPID.calculate(tagTracking.get3DYaw());
    xSpeed = MathUtil.clamp(xSpeed, -2, 2);
    ySpeed = MathUtil.clamp(ySpeed, -2, 2);
    swerveDrive.drive(xSpeed, ySpeed, rotSpeed, false);
    // CHECKSTYLE.ON: LocalVariableName

    SmartDashboard.putNumber("TagTrackingXSpeed", xSpeed);
    SmartDashboard.putNumber("TagTrackingYSpeed", ySpeed);
    SmartDashboard.putNumber("TagTrackingRotSpeed", rotSpeed);
    SmartDashboard.putData("TzController", tzPID);
    SmartDashboard.putData("TxController", txPID);
    SmartDashboard.putData("YawController", yawPID);
    SmartDashboard.putNumber("LeftSetpoint", txPID.getSetpoint());
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
    return tagTracking.isGetTarget() == false
        || (Math.abs(txPID.getError()) < 0.04 && Math.abs(tzPID.getError()) < 0.04);
  }
}