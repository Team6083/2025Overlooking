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

  PIDController txPID = new PIDController(1.5, 0, 0);
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

    var name = isLeft ? "Left" : "Right";
    SmartDashboard.putData(name + "TzController", tzPID);
    SmartDashboard.putData(name + "TxController", txPID);
    SmartDashboard.putData(name + "YawController", yawPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double tagId = tagTracking.getBestTargetId();

    if (tagId == 6 || tagId == 19) {
      yawPID.setSetpoint(-60);
    } else if (tagId == 7 || tagId == 18) {
      yawPID.setSetpoint(0);
    } else if (tagId == 8 || tagId == 17) {
      yawPID.setSetpoint(60);
    } else if (tagId == 9 || tagId == 22) {
      yawPID.setSetpoint(120);
    } else if (tagId == 10 || tagId == 21) {
      yawPID.setSetpoint(180);
    } else if (tagId == 11 || tagId == 20) {
      yawPID.setSetpoint(-120);
    } else {
      yawPID.setSetpoint(0);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // CHECKSTYLE.OFF: LocalVariableName
    double xSpeed;
    double ySpeed;
    double rotSpeed;

    if (tagTracking.hasTarget()) {

      if (Math.abs(tagTracking.get3dTx()) < 0.5) {
        xSpeed = -tzPID.calculate(tagTracking.get3dTz());
        rotSpeed = yawPID.calculate(swerveDrive.getRotation2dDegrees().getDegrees());

      } else {
        xSpeed = 0;
        rotSpeed = 0;
      }

      ySpeed = txPID.calculate(tagTracking.get3dTx());

      xSpeed = MathUtil.clamp(xSpeed, -0.6, 0.6);
      ySpeed = MathUtil.clamp(ySpeed, -0.6, 0.6);

      swerveDrive.drive(xSpeed, ySpeed, rotSpeed, false);

      SmartDashboard.putNumber("TagTrackingXSpeed", xSpeed);
      SmartDashboard.putNumber("TagTrackingYSpeed", ySpeed);
      SmartDashboard.putNumber("TagTrackingRotSpeed", rotSpeed);
    }
    // CHECKSTYLE.ON: LocalVariableName
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !tagDebouncer.calculate(tagTracking.hasTarget())
        || Math.abs(txPID.getError()) < 0.03 && Math.abs(tzPID.getError()) < 0.05;
  }
}