// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.TagTracking;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveToReefCmd extends Command {
  /** Creates a new SwerveTrackingCmd. */
  SwerveDrive swerveDrive;

  TagTracking tagTracking = new TagTracking();

  PIDController txPID = new PIDController(1.2, 0, 0);
  PIDController tzPID = new PIDController(1.2, 0, 0);
  PIDController yawPID = new PIDController(0.03, 0, 0);

  Debouncer tagDebouncer = new Debouncer(1, Debouncer.DebounceType.kFalling);

  public SwerveToReefCmd(SwerveDrive swerveDrive, boolean isLeft) {
    this.swerveDrive = swerveDrive;

    yawPID.enableContinuousInput(-180, 180);

    tzPID.setSetpoint(0.55);

    addRequirements(swerveDrive);

    if (isLeft) {
      txPID.setSetpoint(0.1837);
    } else {
      txPID.setSetpoint(-0.16);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double tagID = tagTracking.getBestTargetID();

    if (tagID == 6 || tagID == 19) {
      yawPID.setSetpoint(-60);
    } else if (tagID == 7 || tagID == 18) {
      yawPID.setSetpoint(0);
    } else if (tagID == 8 || tagID == 17) {
      yawPID.setSetpoint(60);
    } else if (tagID == 9 || tagID == 22) {
      yawPID.setSetpoint(120);
    } else if (tagID == 10 || tagID == 21) {
      yawPID.setSetpoint(180);
    } else if (tagID == 11 || tagID == 20) {
      yawPID.setSetpoint(-120);
    } else {
      yawPID.setSetpoint(0);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed;
    double ySpeed;
    double rotSpeed;

    if (tagTracking.isGetTarget()) {

      if (Math.abs(tagTracking.get3DTx()) < 0.5) {
        xSpeed = -tzPID.calculate(tagTracking.get3DTz());
        rotSpeed = yawPID.calculate(swerveDrive.getRotation2dDegrees().getDegrees());

      } else {
        xSpeed = 0;
        rotSpeed = 0;
      }

      ySpeed = txPID.calculate(tagTracking.get3DTx());

      xSpeed = MathUtil.clamp(xSpeed, -0.6, 0.6);
      ySpeed = MathUtil.clamp(ySpeed, -0.6, 0.6);

      swerveDrive.drive(xSpeed, ySpeed, rotSpeed, false);

      SmartDashboard.putNumber("TagTrackingXSpeed", xSpeed);
      SmartDashboard.putNumber("TagTrackingYSpeed", ySpeed);
      SmartDashboard.putNumber("TagTrackingRotSpeed", rotSpeed);
      SmartDashboard.putData("TzController", tzPID);
      SmartDashboard.putData("TxController", txPID);
      SmartDashboard.putData("YawController", yawPID);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !tagDebouncer.calculate(tagTracking.isGetTarget())
        || Math.abs(txPID.getError()) < 0.025 && Math.abs(tzPID.getError()) < 0.05;
  }
}