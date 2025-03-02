// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

/** Add your docs here. */
public class PreferencesClass {
  public class DriveMotorInverted {
    public static Map<String, Boolean> AUDriveMotorInverted_MAP = Map.of(
        "kFrontLeftDriveMotorInverted", false,
        "kFrontRightDriveMotorInverted", true,
        "kBackLeftDriveMotorInverted", false,
        "kBackRightDriveMotorInverted", true);

    public static Map<String, Boolean> TWNDriveMotorInverted_MAP = Map.of(
        "kFrontLeftDriveMotorInverted", true,
        "kFrontRightDriveMotorInverted", false,
        "kBackLeftDriveMotorInverted", true,
        "kBackRightDriveMotorInverted", false);
    public static Map<String, Boolean> currentConfig = AUDriveMotorInverted_MAP;

    public static boolean get(String key) {

      return currentConfig.get(key);
    }

  }

  public class CanCoderMagOffset {
    public static Map<String, Double> AUCanCoderMagOffset_MAP = Map.of(
        "kFrontLeftCanCoderMagOffset", 0.145264 ,
        "kFrontRightCanCoderMagOffset", 0.245361 ,
        "kBackLeftCanCoderMagOffset", 0.019043 ,
        "kBackRightCanCoderMagOffset", 0.082764);
    public static Map<String, Double> TWNCanCoderMagOffset_MAP = Map.of(
        "kFrontLeftCanCoderMagOffset", 0.069092,
        "kFrontRightCanCoderMagOffset", 0.369141,
        "kBackLeftCanCoderMagOffset", 0.401855,
        "kBackRightCanCoderMagOffset", -0.010254

    );
    public static Map<String, Double> currentConfig = AUCanCoderMagOffset_MAP;

    public static double get(String key) {

      return currentConfig.get(key);
    }

  }

}