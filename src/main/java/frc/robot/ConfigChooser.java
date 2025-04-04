package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.ModuleConstant;

import static edu.wpi.first.units.Units.Millimeters;

import java.util.Map;

/**
 * Manages robot preferences and configuration values for different robot
 * configurations.
 * Provides easy access to configuration values through static methods.
 */
public class ConfigChooser {

  // Current configuration selection
  private static boolean isAustraliaConfig = true;

  public static void initConfig() {
    Preferences.initInt("robotConfig", 0);
  }
  /**
   * Updates the current configuration based on the robot preference.
   */
  public static void updateConfig() {
    isAustraliaConfig = Preferences.getInt("robotConfig", 0) == 0;
  }

  public static boolean isAustraliaConfig() {
    return isAustraliaConfig;
  }

  /**
   * Gets a value from the current configuration map.
   * 
   * @param australiaMap The map for Australia configuration
   * @param taiwanMap    The map for Taiwan configuration
   * @param key          The key to look up
   * @param defaultValue The default value if the key is not found
   * @return The value from the map or the default value
   */
  private static <T> T getValue(Map<String, T> australiaMap, Map<String, T> taiwanMap, String key) {
    Map<String, T> currentMap = isAustraliaConfig ? australiaMap : taiwanMap;
    return currentMap.get(key);
  }

  /**
   * Configuration values for swerve control.
   */
  public static class SwerveControl {
    private static final Map<String, Double> australiaMap = Map.of(
        "kDefaultMagnification", 0.15,
        "kFastMagnification", 0.75,
        "kSafeMagnification", 0.0625,
        "kRotDefaultMagnification", 0.35,
        "kRotFastMagnification", 0.8,
        "kRotSafeMagnification", 0.25);

    private static final Map<String, Double> taiwanMap = Map.of(
        "kDefaultMagnification", 0.17,
        "kFastMagnification", 0.75,
        "kSafeMagnification", 0.0625,
        "kRotDefaultMagnification", 0.35,
        "kRotFastMagnification", 0.75,
        "kRotSafeMagnification", 0.20);

    public static double getDouble(String key) {
      return getValue(australiaMap, taiwanMap, key);
    }
  }

  /**
   * Configuration values for swerve modules.
   */
  public static class SwerveModule {
    private static final Map<String, Double> australiaMAP = Map.of(
        "kP", ModuleConstant.kMaxModuleTurningVoltage / 180,
        "kI", 0.0,
        "kD", 0.0);

    private static final Map<String, Double> taiwanMAP = Map.of(
        "kP", ModuleConstant.kMaxModuleTurningVoltage / 180,
        "kI", 0.0,
        "kD", 0.0005);

    public static double getDouble(String key) {
      return getValue(australiaMAP, taiwanMAP, key);
    }
  }

  /**
   * Configuration values for drivebase.
   */
  public static class DriveBase {
    private static final Map<String, Boolean> australiaBooleanMap = Map.of(
        "kFrontLeftDriveMotorInverted", false,
        "kFrontRightDriveMotorInverted", true,
        "kBackLeftDriveMotorInverted", false,
        "kBackRightDriveMotorInverted", true);

    private static final Map<String, Boolean> taiwanBooleanMap = Map.of(
        "kFrontLeftDriveMotorInverted", true,
        "kFrontRightDriveMotorInverted", false,
        "kBackLeftDriveMotorInverted", true,
        "kBackRightDriveMotorInverted", false);

    private static final Map<String, Double> australiaDoubleMap = Map.of(
        "kFrontLeftCanCoderMagOffset", 0.145264,
        "kFrontRightCanCoderMagOffset", 0.245361,
        "kBackLeftCanCoderMagOffset", 0.479043,
        "kBackRightCanCoderMagOffset", 0.082764);

    private static final Map<String, Double> taiwanDoubleMap = Map.of(
        "kFrontLeftCanCoderMagOffset", 0.126709,
        "kFrontRightCanCoderMagOffset", 0.217285,
        "kBackLeftCanCoderMagOffset", -0.423828,
        "kBackRightCanCoderMagOffset", -0.087402);

    public static boolean getBoolean(String key) {
      return getValue(australiaBooleanMap, taiwanBooleanMap, key);
    }

    public static double getDouble(String key) {
      return getValue(australiaDoubleMap, taiwanDoubleMap, key);
    }
  }

  /**
   * Configuration values for algae intake.
   */
  public static class AlgaeIntake {
    private static final Map<String, Integer> australiaMap = Map.of(
        "expectedZero", -157);

    private static final Map<String, Integer> taiwanMap = Map.of(
        "expectedZero", -265);

    public static int getInt(String key) {
      return getValue(australiaMap, taiwanMap, key);
    }
  }

  /**
   * Configuration values for coral shooter.
   */
  public static class CoralShooter {
    private static final Map<String, Double> australiaMap = Map.of(
        "kCoralInSpeed", 0.195,
        "kCoralOutSpeed", 0.195,
        "kCoralReverseSpeed", -0.195,
        "kP", 0.0035,
        "kI", 0.0,
        "kD", 0.0);

    private static final Map<String, Double> taiwanMap = Map.of(
        "kCoralInSpeed", 0.15,
        "kCoralOutSpeed", 0.25,
        "kCoralReverseSpeed", -0.2,
        "kP", 0.0007,
        "kI", 0.0,
        "kD", 0.00015);

    public static double getDouble(String key) {
      return getValue(australiaMap, taiwanMap, key);
    }
  }

  /**
   * Configuration values for elevator.
   */
  public static class Elevator {
    private static final Map<String, Distance> australiaMap = Map.of(
        "kHeightOffset", Millimeters.of(540.0),
        "kLowestHeight", Millimeters.of(540.0),
        "kMaxHeight", Millimeters.of(1630.0),
        "kInitialHeight", Millimeters.of(540.0),
        "kSecFloor", Millimeters.of(879.0),
        "kTrdFloor", Millimeters.of(1300.0),
        "kTopFloor", Millimeters.of(1850),
        "kToGetSecAlgaeHeight", Millimeters.of(818.0),
        "kToGetTrdAlgaeHeight", Millimeters.of(1355.0));

    private static final Map<String, Distance> taiwanMap = Map.of(
        "kHeightOffset", Millimeters.of(600.0),
        "kLowestHeight", Millimeters.of(600.0),
        "kMaxHeight", Millimeters.of(1520.0),
        "kInitialHeight", Millimeters.of(600.0),
        "kSecFloor", Millimeters.of(839.0),
        "kTrdFloor", Millimeters.of(1186.666666),
        "kTopFloor", Millimeters.of(1740.0),
        "kToGetSecAlgaeHeight", Millimeters.of(818.0),
        "kToGetTrdAlgaeHeight", Millimeters.of(1355.0));

    public static Distance getDistance(String key) {
      return getValue(australiaMap, taiwanMap, key);
    }
  }
}
