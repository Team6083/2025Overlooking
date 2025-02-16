package frc.robot;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;


public class Constants {
  public static final class CoralShooterConstant {
    public static final int kOnboard = 0;
    public static final double kDistanceRange = 4;
    public static final int kShooterLeftMotorChannel = 1;
    public static final int kShooterRightMotorChannel = 2;
    public static final double kShooterMotorFastSpeed = 4;
    public static final double kShooterMotorSlowSpeed = 4;
    public static final Boolean kCoralShooterMotorInverted = false;
  }

  public static final class PowerDistributionConstant {
    // Motor channel
    public static final int kCoralShooterRightMotorCurrentChannel = 7;
    public static final int kCoralShooterLeftMotorCurrentChannel = 7;
    public static final int kAlgaeIntakeMotorCurrentChannel = 7;
    public static final int kAlgaeRotateMotorCurrentChannel = 7;
    public static final int kClimberMotorCurrentChannel = 7;
    public static final int kRampMotorCurrentChannel = 7;
    // Motor Max Current
    public static final double kCoralShooterMotorMaxCurrent = 40;
    public static final double kAlgaeIntakeMotorMaxCurrent = 40;
    public static final double kAlgaeRotateMotorMaxCurrent = 40;
    public static final double kClimberMotorMaxCurrent = 40;
    public static final double kRampMotorMaxCurrent = 40;
  }
  public static final class ModuleConstant {
    // 定義輪子的半徑，單位是公尺
    public static final Distance kWheelRadius = Meters.of(0.064);

    // 定義輪子的 driveMotor & turningMotor 最大輸出電壓
    public static final double kMaxModuleDriveVoltage = 12.0;
    public static final double kMaxModuleTurningVoltage = 12.0;

    // 設定 Motor 的 closedLoopRampRate 之時距
    public static final double kDriveClosedLoopRampRate = 0.1; // 1 second 1 unit
    public static final double kTurningClosedLoopRampRate = 0.1;

    // 目前使用方式為直接將輸入速度轉換成電壓，並沒有考慮輪子是否有達到目標轉速
    public static final double kDesireSpeedToMotorVoltage = kMaxModuleDriveVoltage
        / DriveBaseConstant.kMaxSpeed.in(MetersPerSecond);

    // 設定 rotPID 的參數（180 是最大誤差角度）
    public static final double kPRotationController = kMaxModuleTurningVoltage
        / 180;

    public static final double kIRotationController = 0.0;
    public static final double kDRotationController = 0.0;

    public static final boolean kTurningMotorInverted = true;
  }

  public static final class DriveBaseConstant {
    // driveMotor channel
    public static final int kFrontLeftDriveMotorChannel = 24;
    public static final int kFrontRightDriveMotorChannel = 21;
    public static final int kBackLeftDriveMotorChannel = 23;
    public static final int kBackRightDriveMotorChannel = 22;

    // turningMotor channel
    public static final int kFrontLeftTurningMotorChannel = 28;
    public static final int kFrontRightTurningMotorChannel = 25;
    public static final int kBackLeftTurningMotorChannel = 27;
    public static final int kBackRightTurningMotorChannel = 26;

    // driveMotor inverted
    public static final boolean kFrontLeftDriveMotorInverted = false;
    public static final boolean kFrontRightDriveMotorInverted = true;
    public static final boolean kBackLeftDriveMotorInverted = false;
    public static final boolean kBackRightDriveMotorInverted = true;

    // turningMotor inverted
    public static final boolean kFrontLeftTurningMotorInverted = true;
    public static final boolean kFrontRightTurningMotorInverted = true;
    public static final boolean kBackLeftTuringMotorInverted = true;
    public static final boolean kBackRightTurningMotorInverted = true;

    // turning CANcoder ID
    public static final int kFrontLeftCanCoder = 6;
    public static final int kFrontRightCanCoder = 3;
    public static final int kBackLeftCanCoder = 5;
    public static final int kBackRightCanCoder = 4;

    // turning encoder magnet offset value
    public static final double kFrontLeftCanCoderMagOffset = 0.069092;
    public static final double kFrontRightCanCoderMagOffset = 0.369141;
    public static final double kBackLeftCanCoderMagOffset = 0.401855;
    public static final double kBackRightCanCoderMagOffset = -0.010254;

    // whether gyro is under the robot
    public static final boolean kGyroInverted = false;
    public static final double kGyroOffSet = 0.0;

    // 機器人的大小規格
    public static final Distance kRobotWidth = Meters.of(0.6);
    public static final Distance kRobotLength = Meters.of(0.6);
    public static final Distance kRobotDiagonal = Meters.of(
        Math.sqrt(Math.pow(kRobotLength.in(Meters), 2.0) + Math.pow(kRobotWidth.in(Meters), 2.0)));

    // 最大轉速需要實際測試看看
    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.0);
  }
}
