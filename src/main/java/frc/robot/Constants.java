package frc.robot;

public class Constants {

  public static final class ClimberConstant {
    public static final int kClimberSetpoint = 40;
    public static final double kClimbDownSpeed = -0.35;
    public static final Boolean kclimberMotorInverted = false;
    public static final Boolean kclimberEncoderInverted = true;
  }
  public static final class AutoConstants {
    public static final double kPTranslation = 0.08;
    public static final double kITranslation = 0;
    public static final double kDTranslation = 0;
    public static final double kPRotation = 1;
    public static final double kIRotation = 0;
    public static final double kDRotation = 0;
    public static final double kDrivebaseRadius = 0.3;

  }

  public static final class CoralShooterConstant {
    public static final int kOnboard = 0;
    public static final double kDistanceRange = 4;
    public static final int kShooterMotorChannel = 1;
    public static final double kShooterMotorSpeed = 4;
    public static final Boolean kcoralShooterMotorInverted = false;
  }

  public static final class AlgaeIntakeConstant {
    public static final int kIntakeMotorChannel = 2;
    public static final int kRotateMotorChannel = 1;
    public static final double kIntakeVoltage = 6.0;
    public static final double kReIntakeVoltage = 3.0;
    public static final double kUpIntakeVoltage = 12.0;
    public static final double kDownIntakeVoltage = -12.0;
    public static final Boolean kIntakeMotorInverted = false;
    public static final Boolean krotateIntakeMotorInverted = false;
  }

  public static final class RampConstant {
    public static final Boolean rampmotorInverted = false;
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

  public static final class ModuleConstants {
    // 定義輪子的半徑，單位是公尺
    public static final double kWheelRadius = 0.046;
    public static final double kWheelGearRate = 1;

    // 定義輪子的 driveMotor & turningMotor 最大輸出電壓
    public static final double kMaxModuleDriveVoltage = 12.0;
    public static final double kMaxModuleTurningVoltage = 12.0;

    // 設定 Motor 的 closedLoopRampRate 之時距
    public static final double kDriveClosedLoopRampRate = 0.1; // 1 second 1 unit
    public static final double kTurningClosedLoopRampRate = 0.1;

    // 目前使用方式為直接將輸入速度轉換成電壓，並沒有考慮輪子是否有達到目標轉速
    public static final double kDesireSpeedToMotorVoltage = kMaxModuleDriveVoltage
        / DriveBaseConstants.kMaxSpeed;

    // 設定 turningMotor 轉動到目標角度的速度比例，當此值越大轉動速度越慢
    public static final double kMaxSpeedTurningDegree = 180.0;

    // 設定 rotPID 的參數
    public static final double kPRotationController = kMaxModuleTurningVoltage
        / kMaxSpeedTurningDegree;
    // public static final double kPRotationController = 0.01;
    public static final double kIRotationController = 0.0;
    public static final double kDRotationController = 0.0;

    public static final boolean kTurningMotorInverted = true;
  }

  public static final class DriveBaseConstants {
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
    public static final double kFrontLeftCanCoderMagOffset = -0.02539071875;
    public static final double kFrontRightCanCoderMagOffset =-0.410888984375;
    public static final double kBackLeftCanCoderMagOffset = 0.169433734375;
    public static final double kBackRightCanCoderMagOffset =0.1416015625;

    public static final int kGyroChannel = 30;

    // whether gyro is under the robot
    public static final boolean kGyroInverted = false;
    public static final double kGyroOffSet = 0.0;

    // 機器人的大小規格
    public static final double kRobotWidth = 0.6;
    public static final double kRobotLength = 0.6;
    public static final double kRobotDiagonal = Math.sqrt(
        Math.pow(kRobotLength, 2.0) + Math.pow(kRobotWidth, 2.0));

    // 機器人的重量 (還沒測)
    public static final double kRobotMass = 45;

    // 機器人兩排輪子間的寬度，以公尺為單位(還沒測)
    public static final double kTrackWidth = 2;

    // 機器人的輪子半徑，以公尺為單位
    public static final double kWheelRadiusMeters = 0.1;

    // 最大轉速需要實際測試看看
    public static final double kMaxSpeed = 4.0;

    // 設定最小轉速，以避免 Gamepad 胡亂輸送訊號以至於機器人不受控制
    public static final double kMinSpeed = 0.2;

    // 最大角速度
    public static final double kMaxAngularSpeed = kMaxSpeed / (kRobotDiagonal / 2.0); // rad/s

    // 機器人的轉動慣量，單位為 KG*M^2 (這個不知道)
    public static final double kRobotMomentOfInertia = 6;

    // 輪子和地毯之間的摩擦係數
    public static final double kWheelCOF = 1.0;

    // 驅動馬達的電流限制，以安培為單位
    public static final double kDriveCurrentLimit = 12.0;

    // 每個模組的馬達數量
    public static final int kNumMotors = 2;

    // make the input from Gamepad more smooth
    public static final double kXLimiterRateLimit = 5.0;
    public static final double kYLimiterRateLimit = 5.0;
    public static final double kRotLimiterRateLimit = 5.0;

    // Tracking PID
    public static final double kTrackingP = 0.08;
    public static final double kTrackingI = 0.0;
    public static final double kTrackingD = 0.0;

    // 變速的放大倍率
    public static final double kMagnification = 2;
    public static final double kHighMagnification = 2;
    public static final double kDefaultMagnification = 0.5;

    // fieldRelative
    // Field - true / Robot - false
    public static final boolean kFieldRelative = true;
  }

}
