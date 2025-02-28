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
    } else if (trueCanCoderMagOffset == twn) {
      kfrontLeftCanCoderMagOffset = DriveBaseConstant.TWNkFrontLeftCanCoderMagOffset;
      kfrontRightCanCoderMagOffset = DriveBaseConstant.TWNkFrontRightCanCoderMagOffset;
      kbackLeftCanCoderMagOffset = DriveBaseConstant.TWNkBackLeftCanCoderMagOffset;
      kbackRightCanCoderMagOffset = DriveBaseConstant.TWNkBackRightCanCoderMagOffset;
    }
    SmartDashboard.putNumber("fl", kfrontLeftCanCoderMagOffset);
    SmartDashboard.putNumber("fr", kfrontRightCanCoderMagOffset);
    SmartDashboard.putNumber("bl", kbackLeftCanCoderMagOffset);
    SmartDashboard.putNumber("br", kbackRightCanCoderMagOffset);
    SmartDashboard.putNumber("TrueCanCoderMagOffset", trueCanCoderMagOffset);
  }
}
