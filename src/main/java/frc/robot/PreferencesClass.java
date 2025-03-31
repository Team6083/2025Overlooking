// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ModuleConstant;
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

  public class SwerveControl {
    public static Map<String, Double> AUSwerveControlDouble_MAP = Map.of(
        "kDefaultMagnification", 0.4,
        "kFastMagnification", 0.75,
        "kSafeMagnification", 0.0625,
        "kRotDefaultMagnification", 1.2,
        "kRotFastMagnification", 0.8,
        "kRotSafeMagnification", 0.25);
    public static Map<String, Distance> AUSwerveControlDistance_MAP = Map.of(
        "kElevatorSafetyHeight", Millimeters.of(545.0));

    public static Map<String, Double> TWNSwerveControlDouble_MAP = Map.of(
        "kDefaultMagnification", 0.17,
        "kFastMagnification", 0.75,
        "kSafeMagnification", 0.0625,
        "kRotDefaultMagnification", 0.35,
        "kRotFastMagnification", 0.75,
        "kRotSafeMagnification", 0.20);
    public static Map<String, Distance> TWNSwerveControlDistance_MAP = Map.of(
        "kElevatorSafetyHeight", Millimeters.of(719.0));

    public static Map<String, Double> currentConfigDouble = TWNSwerveControlDouble_MAP;
    public static Map<String, Distance> currentConfigDistance = TWNSwerveControlDistance_MAP;

    public static double getDouble(String key) {
      return currentConfigDouble.get(key);
    }

    public static Distance getDistance(String key) {
      return currentConfigDistance.get(key);
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

  public class CoralShooter {
    public static Map<String, Double> AUCoralShooterConstant_MAP = Map.of(
        "kCoralInSpeed", 0.195,
        "kCoralOutSpeed", 0.195,
        "kCoralReverseSpeed", -0.195,
        "kP", 0.0035,
        "kI", 0.0,
        "kD", 0.0);
    public static Map<String, Double> TWNCoralShooterConstant_MAP = Map.of(
        "kCoralInSpeed", 0.15,
        "kCoralOutSpeed", 0.25,
        "kCoralReverseSpeed", -0.2,
        "kP", 0.0007,
        "kI", 0.0,
        "kD", 0.00015);
    public static Map<String, Double> currentConfig = TWNCoralShooterConstant_MAP;

    public static double get(String key) {
      return currentConfig.get(key);
    }
  }

  public class Elevator {
    public static Map<String, Distance> AUElevator_MAP = Map.of(
      "kHeightOffset", Millimeters.of(600.0),

      "kLowestHeight", Millimeters.of(600.0),
      "kMaxHeight", Millimeters.of(1520),

      "kInitialHeight", Millimeters.of(600.0),
      "kSecFloor", Millimeters.of(839.0),
      "kTrdFloor", Millimeters.of(1186.666666),
      "kTopFloor", Millimeters.of(1740),
      "kToGetSecAlgaeHeight", Millimeters.of(818.0),
      "kToGetTrdAlgaeHeight", Millimeters.of(1355.0)
    )
    public static Map<String, Distance> TWNElevator_MAP = Map.of(
      "kHeightOffset", Millimeters.of(600.0),

      "kLowestHeight", Millimeters.of(600.0),
      "kMaxHeight", Millimeters.of(1520),

      "kInitialHeight", Millimeters.of(600.0),
      "kSecFloor", Millimeters.of(839.0),
      "kTrdFloor", Millimeters.of(1186.666666),
      "kTopFloor", Millimeters.of(1740),
      "kToGetSecAlgaeHeight", Millimeters.of(818.0),
      "kToGetTrdAlgaeHeight", Millimeters.of(1355.0)
    );
  }
}