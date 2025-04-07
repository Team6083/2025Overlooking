package frc.robot;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.ModuleConstant;
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
    Preferences.initBoolean("isAustraliaConfig", true);
  }

  /**
   * Updates the current configuration based on the robot preference.
   */
  public static void updateConfig() {
    isAustraliaConfig = Preferences.getBoolean("isAustraliaConfig", true);
  }

  public static boolean isAustraliaConfig() {
    return isAustraliaConfig;
  }

  /**
   * Gets a value from the current configuration map.
   *
   * @param <T>          The type of the value in the map
   * @param australiaMap The map for Australia configuration
   * @param taiwanMap    The map for Taiwan configuration
   * @param key          The key to look up
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
   * Configuration values for drive base.
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
        "kFrontLeftCanCoderMagOffset", 0.126221,
        "kFrontRightCanCoderMagOffset", 0.226562,
        "kBackLeftCanCoderMagOffset", -0.397461,
        "kBackRightCanCoderMagOffset", -0.091064);

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
    private static final Map<String, Double> australiaMap = Map.ofEntries(
        Map.entry("kIntakeFastSpeed", 0.3),
        Map.entry("kIntakeSlowSpeed", 0.1),
        Map.entry("kReverseIntakeSpeed", -0.3),

        Map.entry("kUpIntakeRotateSpeed", -0.6),
        Map.entry("kDownIntakeRotateSpeed", 0.1),

        Map.entry("rotMotorUpPIDkP", 0.07),
        Map.entry("rotMotorUpPIDkI", 0.0),
        Map.entry("rotMotorUpPIDkD", 0.0),

        Map.entry("rotMotorDownPIDkP", 0.01),
        Map.entry("rotMotorDownPIDkI", 0.0),
        Map.entry("rotMotorDownPIDkD", 0.0),

        Map.entry("expectedZero", -157.0),

        Map.entry("kGetAlgaeAngle", 95.0));

    private static final Map<String, Double> taiwanMap = Map.ofEntries(
        Map.entry("kIntakeFastSpeed", 0.3),
        Map.entry("kIntakeSlowSpeed", 0.1),
        Map.entry("kReverseIntakeSpeed", -0.3),

        Map.entry("kUpIntakeRotateSpeed", -0.6),
        Map.entry("kDownIntakeRotateSpeed", 0.1),

        Map.entry("rotMotorUpPIDkP", 0.07),
        Map.entry("rotMotorUpPIDkI", 0.0),
        Map.entry("rotMotorUpPIDkD", 0.0),

        Map.entry("rotMotorDownPIDkP", 0.01),
        Map.entry("rotMotorDownPIDkI", 0.0),
        Map.entry("rotMotorDownPIDkD", 0.0),

        Map.entry("expectedZero", -265.0),

        Map.entry("kGetAlgaeAngle", 95.0));

    public static double getDouble(String key) {
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
        "kP", 0.003,
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
        "kMaxHeight", Millimeters.of(1935.0),

        "kInitialHeight", Millimeters.of(540.0),
        "kSecFloor", Millimeters.of(879.0),
        "kTrdFloor", Millimeters.of(1300.0),
        "kTopFloor", Millimeters.of(1850),

        "kToGetSecAlgaeHeight", Millimeters.of(818.0),
        "kToGetTrdAlgaeHeight", Millimeters.of(1355.0));

    private static final Map<String, Distance> taiwanMap = Map.of(
        "kHeightOffset", Millimeters.of(600.0),

        "kLowestHeight", Millimeters.of(600.0),
        "kMaxHeight", Millimeters.of(1770.0),

        "kInitialHeight", Millimeters.of(600.0),
        "kSecFloor", Millimeters.of(839.0),
        "kTrdFloor", Millimeters.of(1186.666666),
        "kTopFloor", Millimeters.of(1730.0),

        "kToGetSecAlgaeHeight", Millimeters.of(925.0),
        "kToGetTrdAlgaeHeight", Millimeters.of(1275.0));

    private static final Map<String, Double> australiaDoubleMap = Map.of(
        "kMaxOutputHigher", 0.5,
        "kMaxOutputLower", 0.4,
        "kP", 0.02);

    private static final Map<String, Double> taiwanDoubleMap = Map.of(
        "kMaxOutputHigher", 0.66,
        "kMaxOutputLower", 0.4,
        "kP", 0.058);

    public static Distance getDistance(String key) {
      return getValue(australiaMap, taiwanMap, key);
    }

    public static double getDouble(String key) {
      return getValue(australiaDoubleMap, taiwanDoubleMap, key);
    }
  }
}
