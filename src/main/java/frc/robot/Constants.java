package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
  public static final class CoralShooterConstant {
    public static final double kDistanceRange = 4.0;
    public static final int kShooterMotorChannel = 32;
    public static final int kShooterEncoderChannel = 4;
    public static final double kShooterMotorFastSpeed = 0.25;
    public static final double kShooterMotorSlowSpeed = 0.195;
    public static final Boolean kCoralShooterMotorInverted = false;
    public static final Boolean kCoralShooterEncoderInverted = true;
    public static final double kEncoderFullRange = 360.0;
    public static final double kEncoderOffset = 0.0;
  }

  public static final class PowerDistributionConstant {
    public static final int kCoralShooterRightMotorCurrentChannel = 0;
    public static final int kCoralShooterLeftMotorCurrentChannel = 1;
    public static final int kAlgaeIntakeMotorCurrentChannel = 7;
    public static final int kAlgaeRotateMotorCurrentChannel = 7;
    public static final int kClimberMotorCurrentChannel = 7;
    public static final int kRampMotorCurrentChannel = 7;
    public static final int kElevatorMotorCurrentChannel = 2;

    public static final double kCoralShooterMotorMaxCurrent = 40;
    public static final double kAlgaeIntakeMotorMaxCurrent = 40;
    public static final double kAlgaeRotateMotorMaxCurrent = 40;
    public static final double kClimberMotorMaxCurrent = 40;
    public static final double kRampMotorMaxCurrent = 40;
    public static final double kElevatorMotorMaxCurrent = 40;
  }

  public static final class ElevatorConstant {
    public static final double kP = 0.06;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final int kElevatorMotorChannel = 33;
    public static final boolean kMotorInverted = true;

    public static final int kEncoderChannelA = 2;
    public static final int kEncoderChannelB = 3;

    public static final double kManualUpPower = 0.5;
    public static final double kManualDownPower = -0.5;

    public static final Distance kHeightOffset = Millimeters.of(430.0);

    public static final Distance kLowestHeight = Millimeters.of(430.0);
    public static final Distance kMaxHeight = Millimeters.of(1000);

    public static final double kMaxOutput = 0.8;
    public static final double kMinOutput = -0.5;

    public static final Distance kInitialHeight = Millimeters.of(430.0);
    public static final Distance kGetCarolHeight = Millimeters.of(485);
    public static final Distance kSecFloor = Millimeters.of(810);
    public static final Distance kTrdFloor = Millimeters.of(1210);
    public static final Distance kTopFloor = Millimeters.of(1830);
    public static final Distance kStepHeight = Millimeters.of(1);

    public static final double kEncoderDistancePerPulse = (1.0 / 2048.0)
        * Inches.of(1.214 * Math.PI).in(Millimeters);
  }

  public static final class ModuleConstant {
    // 定義輪子的半徑，單位是公尺
    public static final Distance kWheelRadius = Meters.of(0.0508);

    // 定義輪子的 driveMotor & turningMotor 最大輸出電壓
    public static final double kMaxModuleDriveVoltage = 12.0;
    public static final double kMaxModuleTurningVoltage = 12.0;

    // 設定 Motor 的 closedLoopRampRate 之時距
    public static final double kDriveClosedLoopRampRate = 0; // 1 second 1 unit
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

  public static final class AlgaeIntakeConstant {
    // AlgaeMotor Channel
    public static final int kIntakeMotorChannel = 35;
    public static final int kIntakeRotateMotorChannel = 34;

    // Algae Encoder Channel
    public static final int kAlgaeEncoderChannelA = 0;
    public static final int kAlgaeEncoderChannelB = 1;
    public static final int kAlgaeFrontEncoderChannelA = 1;
    public static final int kAlgaeFrontEncoderChannelB = 2;

    // Algae 吸入、吐出的電壓

    public static final double kIntakeFastSpeed = 0.5;
    public static final double kIntakeSlowSpeed = 0.1;
    public static final double kReIntakeSpeed = -0.3;

    // AlgaeRotate 的電壓
    public static final double kUpIntakeRotateSpeed = 0.5;
    public static final double kDownIntakeRotateSpeed = -0.4;

    // Algae Rotate & Front PID
    public static final double rotMotorPIDkP = 0.01;
    public static final double rotMotorPIDkI = 0;
    public static final double rotMotorPIDkD = 0;

    // Algae setpoint
    public static final int kUpRotateIntakeSetpoint = 90;
    public static final int kDownRotateIntakeSetpoint = 0;

    // Algae Inverted
    public static final Boolean kIntakeMotorInverted = true;
    public static final Boolean kRotateMotorInverted = false;
    public static final Boolean kAlgaeEncoderInverted = false;

    // Algae Current Limit
    public static final double kIntakeCurrentLimit = 10;

    // Algae Encoder
    public static final double kDistancePerPulse = 360.0 / 2048;
    public static final double kRotateEncoderOffset = 0.0;
    public static final double output = 0.2;

  }

  public static final class SwerveControlConstant {

    // 變速的放大倍率
    public static final double kDefaultMagnification = 1;
    public static final double kSlowMagnification = 0.75;

    public static final double kDrivebaseMaxSpeed = DriveBaseConstant.kMaxSpeed.in(MetersPerSecond);
    public static final double kMinJoystickInput = 0.1;

    public static final double kXLimiterRateLimit = 5.0;
    public static final double kYLimiterRateLimit = 5.0;
    public static final double kRotLimiterRateLimit = 5.0;

    // fieldRelative
    // Field - true / Robot - false
    public static final Boolean kFieldRelative = true;
  }

  public static final class AutoConstants {
    public static final double kPTranslation = 6;
    public static final double kITranslation = 0.15;
    public static final double kDTranslation = 1;
    public static final double kPRotation = 5;
    public static final double kIRotation = 0;
    public static final double kDRotation = 0.6;
  }
}