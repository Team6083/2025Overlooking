// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveBaseConstant;

public class preferencesSubsystem extends SubsystemBase {
  /** Creates a new preferrences. */
  public double kfrontLeftCanCoderMagOffset;
  public double kfrontRightCanCoderMagOffset;
  public double kbackRightCanCoderMagOffset;
  public double kbackLeftCanCoderMagOffset;
  public boolean kFrontLeftDriveMotorInverted;
  public boolean kFrontRightDriveMotorInverted;
  public boolean kBackLeftDriveMotorInverted;
  public boolean kBackRightDriveMotorInverted;
  private int au = 0;
  private int twn = 1;
  private int whereCanCoderMagOffset;
  private int trueCanCoderMagOffset;
  public preferencesSubsystem() {}

  @Override
  public void periodic() {
    Preferences.initInt("WhereCanCoderMagOffset", whereCanCoderMagOffset);
    trueCanCoderMagOffset = Preferences.getInt("WhereCanCoderMagOffset", whereCanCoderMagOffset);
    if (trueCanCoderMagOffset != Preferences.getInt("WhereCanCoderMagOffset", whereCanCoderMagOffset)) {
      trueCanCoderMagOffset = Preferences.getInt("WhereCanCoderMagOffset", whereCanCoderMagOffset);
    }
    if (trueCanCoderMagOffset == au) {
      kfrontLeftCanCoderMagOffset = DriveBaseConstant.AUkFrontLeftCanCoderMagOffset;
      kfrontRightCanCoderMagOffset = DriveBaseConstant.AUkFrontRightCanCoderMagOffset;
      kbackLeftCanCoderMagOffset = DriveBaseConstant.AUkBackLeftCanCoderMagOffset;
      kbackRightCanCoderMagOffset = DriveBaseConstant.AUkBackRightCanCoderMagOffset;
      kFrontLeftDriveMotorInverted = DriveBaseConstant.AUkFrontLeftDriveMotorInverted;
      kFrontRightDriveMotorInverted = DriveBaseConstant.AUkFrontRightDriveMotorInverted;
      kBackLeftDriveMotorInverted = DriveBaseConstant.AUkBackLeftDriveMotorInverted;
      kBackRightDriveMotorInverted = DriveBaseConstant.AUkBackRightDriveMotorInverted;
    } else if (trueCanCoderMagOffset == twn) {
      kfrontLeftCanCoderMagOffset = DriveBaseConstant.TWNkFrontLeftCanCoderMagOffset;
      kfrontRightCanCoderMagOffset = DriveBaseConstant.TWNkFrontRightCanCoderMagOffset;
      kbackLeftCanCoderMagOffset = DriveBaseConstant.TWNkBackLeftCanCoderMagOffset;
      kbackRightCanCoderMagOffset = DriveBaseConstant.TWNkBackRightCanCoderMagOffset;
      kFrontLeftDriveMotorInverted = DriveBaseConstant.TWNkFrontLeftDriveMotorInverted;
      kFrontRightDriveMotorInverted = DriveBaseConstant.TWNkFrontRightDriveMotorInverted;
      kBackLeftDriveMotorInverted = DriveBaseConstant.TWNkBackLeftDriveMotorInverted;
      kBackRightDriveMotorInverted = DriveBaseConstant.TWNkBackRightDriveMotorInverted;
    }
    SmartDashboard.putNumber("flCanCoderMagOffset", kfrontLeftCanCoderMagOffset);
    SmartDashboard.putNumber("frCanCoderMagOffset", kfrontRightCanCoderMagOffset);
    SmartDashboard.putNumber("blCanCoderMagOffset", kbackLeftCanCoderMagOffset);
    SmartDashboard.putNumber("brCanCoderMagOffset", kbackRightCanCoderMagOffset);
    SmartDashboard.putBoolean("flDriveMotorInverted", kFrontLeftDriveMotorInverted);
    SmartDashboard.putBoolean("frDriveMotorInverted", kFrontRightDriveMotorInverted);
    SmartDashboard.putBoolean("blDriveMotorInverted", kBackLeftDriveMotorInverted);
    SmartDashboard.putBoolean("brDriveMotorInverted", kBackRightDriveMotorInverted);
    SmartDashboard.putNumber("TrueCanCoderMagOffset", trueCanCoderMagOffset);
  }
}
