package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
  public static final class CoralShooterConstant {
    public static final double kDistanceRange = 4.0;
    public static final int kShooterLeftMotorChannel = 1;
    public static final int kShooterRightMotorChannel = 2;
    public static final double kShooterMotorFastSpeed = 0.3;
    public static final double kShooterMotorSlowSpeed = 0.8;
    public static final Boolean kCoralShooterRightMotorInverted = false;
    public static final Boolean kCoralShooterLeftMotorInverted = false;
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
    public static final int kFrontLeftDriveMotorChannel = 19;
    public static final int kFrontRightDriveMotorChannel = 11;
    public static final int kBackLeftDriveMotorChannel = 16;
    public static final int kBackRightDriveMotorChannel = 2;

    // turningMotor channel
    public static final int kFrontLeftTurningMotorChannel = 23;
    public static final int kFrontRightTurningMotorChannel = 10;
    public static final int kBackLeftTurningMotorChannel = 15;
    public static final int kBackRightTurningMotorChannel = 21;

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
    public static final int kFrontLeftCanCoder = 0;
    public static final int kFrontRightCanCoder = 1;
    public static final int kBackLeftCanCoder = 3;
    public static final int kBackRightCanCoder = 2;

    // turning encoder magnet offset value
    public static final double kFrontLeftCanCoderMagOffset = 0.020752;
    public static final double kFrontRightCanCoderMagOffset = 0.085205;
    public static final double kBackLeftCanCoderMagOffset = 0.165771;
    public static final double kBackRightCanCoderMagOffset = 0.127930;

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
    public static final int kIntakeMotorChannel = 2;
    public static final int kIntakeRotateMotorChannal = 1;

    // Algae Encoder Channel
    public static final int kalgaeEncoderChannelA = 3;
    public static final int kalgaeEncoderChannelB = 4;

    // Algae 吸入、吐出的電壓
    public static final double kIntakeVoltage = 6.0;
    public static final double kReIntakeVoltage = 3.0;

    // AlgaeRotate 的電壓
    public static final double kUpIntakeRotateVoltage = 12;
    public static final double kDownIntakeRotateVoltage = -12;

    // Algae Rotate&Front PID
    public static final double rotMotorPIDkP = 0.8;
    public static final double rotMotorPIDkI = 0;
    public static final double rotMotorPIDkD = 0;
    public static final double frontMotorPIDkP = 0.8;
    public static final double frontMotorPIDkI = 0;
    public static final double frontMotorPIDkD = 0;

    // Algae setpoint
    public static final int kUprotateIntakeSetpoint = 130;
    public static final int kDownrotateIntakeSetpoint = 30;
    public static final int kfrontIntakeSetpoint = 30;

    // Algae Inverted
    public static final Boolean kIntakeMotorInverted = false;
    public static final Boolean krotateIntakeMotorInverted = false;
  }

  public static final class SwerveControlConstant {

    // 變速的放大倍率
    public static final double kMagnification = 2;
    public static final double kHighMagnification = 2;

    public static final double kDrivebaseMaxSpeed = DriveBaseConstant.kMaxSpeed.in(MetersPerSecond);
    public static final double kMinJoystickInput = 0.1;

    public static final double kXLimiterRateLimit = 5.0;
    public static final double kYLimiterRateLimit = 5.0;
    public static final double kRotLimiterRateLimit = 5.0;

    // fieldRelative
    // Field - true / Robot - false
    public static final Boolean kFieldRelative = true;
  }

}
