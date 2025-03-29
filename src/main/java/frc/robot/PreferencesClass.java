// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import frc.robot.Constants.ModuleConstant;

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
    public static Map<String, Boolean> currentConfig = TWNDriveMotorInverted_MAP;

    public static boolean get(String key) {
      return currentConfig.get(key);
    }
  }

  public class CanCoderMagOffset {
    public static Map<String, Double> AUCanCoderMagOffset_MAP = Map.of(
        "kFrontLeftCanCoderMagOffset", 0.145264,
        "kFrontRightCanCoderMagOffset", 0.245361,
        "kBackLeftCanCoderMagOffset", 0.019043,
        "kBackRightCanCoderMagOffset", 0.082764);
    public static Map<String, Double> TWNCanCoderMagOffset_MAP = Map.of(
        "kFrontLeftCanCoderMagOffset", 0.126709,
        "kFrontRightCanCoderMagOffset", 0.217285,
        "kBackLeftCanCoderMagOffset", -0.423828,
        "kBackRightCanCoderMagOffset", -0.087402);
    public static Map<String, Double> currentConfig = TWNCanCoderMagOffset_MAP;

    public static double get(String key) {
      return currentConfig.get(key);
    }
  }

  public class Magnification {
    public static Map<String, Double> AUMagnification_MAP = Map.of(
        "kDefaultMagnification", 0.4,
        "kFastMagnification", 0.75,
        "kSafeMagnification", 0.0625,
        "kRotDefaultMagnification", 1.2,
        "kRotFastMagnification",0.8,
        "kRotSafeMagnification", 0.25);
    public static Map<String, Double> TWNMagnification_MAP = Map.of(
        "kDefaultMagnification ", 0.15,
        "kFastMagnification ", 0.75,
        "kSafeMagnification", 0.0625,
        "kRotDefaultMagnification", 0.15,
        "kRotFastMagnification",0.75,
        "kRotSafeMagnification", 0.25);
    public static Map<String, Double> currentConfig = TWNMagnification_MAP;

    public static double get(String key) {
      return currentConfig.get(key);
    }
  }

  public class ModuleRotationController {
    public static Map<String, Double> AURotationController_MAP = Map.of(
        "kP", ModuleConstant.kMaxModuleTurningVoltage / 180,
        "kI", 0.0,
        "kD", 0.0);
    public static Map<String, Double> TWNRotationController_MAP = Map.of(
        "kP", ModuleConstant.kMaxModuleTurningVoltage / 180,
        "kI", 0.0,
        "kD", 0.0005);
    public static Map<String, Double> currentConfig = TWNRotationController_MAP;

    public static double get(String key) {
      return currentConfig.get(key);
    }
  }

  public class CoralShooter{
    public static Map<String, Double> AUCoralShooterConstant_MAP = Map.of(
        "kCoralInSpeed", 0.195,
        "kCoralOutSpeed", 0.195,
        "kReverseSpeed", -0.195,
        "kP", 0.0035,
        "kI", 0.0,
        "kD", 0.0);
    public static Map<String, Double> TWNCoralShooterConstant_MAP = Map.of(
        "kCoralInSpeed", 0.2,
        "kCoralOutSpeed", 0.8,
        "kReverseSpeed", -0.2,
        "kP", 0.0007,
        "kI", 0.0,
        "kD", 0.00015);
    public static Map<String, Double> currentConfig = TWNCoralShooterConstant_MAP;

    public static double get(String key) {
      return currentConfig.get(key);
    }
  }

}