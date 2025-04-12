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
public class TagTrackingCmd extends Command {
  /** Creates a new SwerveTrackingCmd. */
  SwerveDrive swerveDrive;

  TagTracking tagTracking = new TagTracking();

  PIDController txPID = new PIDController(3.2, 0, 0.001);
  PIDController tzPID = new PIDController(3, 0, 0);
  PIDController yawPID = new PIDController(0.06, 0, 0);

  Debouncer tagDebouncer = new Debouncer(1, Debouncer.DebounceType.kFalling);

  public enum AimTarget {
    LEFT("Left", 0.1837, 0.55),
    CENTER("Center", 0.0, 0.7),
    RIGHT("Right", -0.16, 0.55);

    double txSetpoint;
    double tzSetpoint;
    String name;

    AimTarget(String name, double txSetpoint, double tzSetpoint) {
      this.name = name;
      this.txSetpoint = txSetpoint;
      this.tzSetpoint = tzSetpoint;
    }

    public String getName() {
      return name;
    }

    public double getTxSetpoint() {
      return txSetpoint;
    }

    public double getTzSetpoint() {
      return tzSetpoint;
    }
  }

  AimTarget aimTarget;

  public TagTrackingCmd(SwerveDrive swerveDrive, AimTarget aimTarget) {
    this.swerveDrive = swerveDrive;
    this.aimTarget = aimTarget;

    yawPID.enableContinuousInput(-180, 180);

    tzPID.setSetpoint(aimTarget.getTzSetpoint());
    txPID.setSetpoint(aimTarget.getTxSetpoint());

    addRequirements(swerveDrive);

    SmartDashboard.putData(aimTarget.name + "ToTagTzController", tzPID);
    SmartDashboard.putData(aimTarget.name + "ToTagTxController", txPID);
    SmartDashboard.putData(aimTarget.name + "ToTagYawController", yawPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int tagId = (int) tagTracking.getBestTargetId();

    switch (tagId) {
      case 6, 19 -> yawPID.setSetpoint(120);
      case 7, 18 -> yawPID.setSetpoint(180);
      case 8, 17 -> yawPID.setSetpoint(-120);
      case 9, 22 -> yawPID.setSetpoint(-60);
      case 10, 21 -> yawPID.setSetpoint(0);
      case 11, 20 -> yawPID.setSetpoint(60);
      default -> yawPID.setSetpoint(0);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // CHECKSTYLE.OFF: LocalVariableName
    int tagId = (int) tagTracking.getBestTargetId();

    switch (tagId) {
      case 6, 19 -> yawPID.setSetpoint(120);
      case 7, 18 -> yawPID.setSetpoint(180);
      case 8, 17 -> yawPID.setSetpoint(-120);
      case 9, 22 -> yawPID.setSetpoint(-60);
      case 10, 21 -> yawPID.setSetpoint(0);
      case 11, 20 -> yawPID.setSetpoint(60);
      default -> yawPID.setSetpoint(0);
    }

    double xSpeed;
    double ySpeed;
    double rotSpeed;

    if (tagTracking.hasTarget()) {

      if (Math.abs(tagTracking.get3dTx()) < 0.5) {
        xSpeed = -tzPID.calculate(tagTracking.get3dTz());

      } else {
        xSpeed = 0;
        rotSpeed = 0;
      }

      double normalizeToMinus180To180Range = swerveDrive.getRotation2dDegrees().getDegrees();

      normalizeToMinus180To180Range = normalizeToMinus180To180Range % 360;

      // 如果角度大於 180°，則將它轉換到 -180° 到 180° 範圍
      if (normalizeToMinus180To180Range > 180) {
        normalizeToMinus180To180Range -= 360;
      } else if (normalizeToMinus180To180Range <= -180) {
        normalizeToMinus180To180Range += 360;
      }

      rotSpeed = yawPID.calculate(normalizeToMinus180To180Range);
      ySpeed = txPID.calculate(tagTracking.get3dTx());

      xSpeed = MathUtil.clamp(xSpeed, -0.6, 0.6);
      ySpeed = MathUtil.clamp(ySpeed, -0.6, 0.6);

      swerveDrive.drive(xSpeed, ySpeed, rotSpeed, false);

      SmartDashboard.putNumber("TagTrackingXSpeed", xSpeed);
      SmartDashboard.putNumber("TagTrackingYSpeed", ySpeed);
      SmartDashboard.putNumber("TagTrackingRotSpeed", rotSpeed);
      SmartDashboard.putNumber("TagCurrentDegree", normalizeToMinus180To180Range);
      SmartDashboard.putData(aimTarget.name + "ToTagTzController", tzPID);
      SmartDashboard.putData(aimTarget.name + "ToTagTxController", txPID);
      SmartDashboard.putData(aimTarget.name + "ToTagYawController", yawPID);
      SmartDashboard.putNumber("TagID", tagId);
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
    var hasTag = tagDebouncer.calculate(tagTracking.hasTarget());
    var txOK = Math.abs(txPID.getError()) < 0.03;
    var tzOK = Math.abs(tzPID.getError()) < 0.1;

    SmartDashboard.putBoolean("TrackingHasTag", hasTag);
    SmartDashboard.putBoolean("TrackingTxOK", txOK);
    SmartDashboard.putBoolean("TrackingTzOK", tzOK);

    return !hasTag
        || (txOK && tzOK);
  }
}