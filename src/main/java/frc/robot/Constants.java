package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
  public static final class SwerveControlConstant {
    // magnet offset
    public static final double kFrontLeftCanCoderMagOffset = 0.126709;
    public static final double kFrontRightCanCoderMagOffset = 0.217285;
    public static final double kBackLeftCanCoderMagOffset = -0.423828;
    public static final double kBackRightCanCoderMagOffset = -0.087402;

    // motor inverted
    public static final boolean kFrontLeftDriveMotorInverted = false;
    public static final boolean kFrontRightDriveMotorInverted = true;
    public static final boolean kBackLeftDriveMotorInverted = false;
    public static final boolean kBackRightDriveMotorInverted = true;

    // speed magnification
    public static final double kDefaultMagnification = 0.25;
    public static final double kFastMagnification = 0.75;

    public static final double kMinJoystickInput = 0.1;

    public static final double kXLimiterRateLimit = 5.0;
    public static final double kYLimiterRateLimit = 5.0;
    public static final double kRotLimiterRateLimit = 5.0;

    // field relative
    public static final Boolean kFieldRelative = true;
  }

  public static final class DriveBaseConstant {
    // drive motor channel
    public static final int kFrontLeftDriveMotorChannel = 27;
    public static final int kFrontRightDriveMotorChannel = 25;
    public static final int kBackLeftDriveMotorChannel = 21;
    public static final int kBackRightDriveMotorChannel = 23;

    // turning motor channel
    public static final int kFrontLeftTurningMotorChannel = 28;
    public static final int kFrontRightTurningMotorChannel = 26;
    public static final int kBackLeftTurningMotorChannel = 22;
    public static final int kBackRightTurningMotorChannel = 24;

    // turning motor inverted
    public static final boolean kFrontLeftTurningMotorInverted = true;
    public static final boolean kFrontRightTurningMotorInverted = true;
    public static final boolean kBackLeftTuringMotorInverted = true;
    public static final boolean kBackRightTurningMotorInverted = true;

    // turning CANcoder ID
    public static final int kFrontLeftCanCoder = 4;
    public static final int kFrontRightCanCoder = 5;
    public static final int kBackLeftCanCoder = 3;
    public static final int kBackRightCanCoder = 6;

    // whether gyro is under the robot
    public static final boolean kGyroInverted = false;
    public static final double kGyroOffSet = 0.0;

    // robot specification
    public static final Distance kRobotWidth = Meters.of(0.6);
    public static final Distance kRobotLength = Meters.of(0.6);
    public static final Distance kRobotDiagonal = Meters.of(
        Math.sqrt(Math.pow(kRobotLength.in(Meters), 2.0) + Math.pow(kRobotWidth.in(Meters), 2.0)));

    // maximum drive speed
    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.0); // TODO: need further test
    // maximum rotate speed
    public static final LinearVelocity kRotateMaxSpeed = MetersPerSecond.of(kMaxSpeed.in(MetersPerSecond)*kRobotDiagonal.in(Meters));
  }

  public static final class ModuleConstant {
    // wheel radius, unit: meter
    public static final Distance kWheelRadius = Meters.of(0.0508);

    // maximum output voltage
    public static final double kMaxModuleDriveVoltage = 12.0;
    public static final double kMaxModuleTurningVoltage = 12.0;

    // set time interval of closedLoopRampRate
    public static final double kDriveClosedLoopRampRate = 0; // 1 second 1 unit
    public static final double kTurningClosedLoopRampRate = 0.1;

    // current method: speed to voltage
    public static final double kDesireSpeedToMotorVoltage = kMaxModuleDriveVoltage
        / DriveBaseConstant.kMaxSpeed.in(MetersPerSecond);

    // set rotation PID parameter (maximum error angle: 180 degrees)
    public static final double kPRotationController = kMaxModuleTurningVoltage / 180;

    public static final double kIRotationController = 0.0;
    public static final double kDRotationController = 0.0005;

    public static final boolean kTurningMotorInverted = true;
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

  public static final class AlgaeIntakeConstant {
    // Algae motor Channel
    public static final int kIntakeMotorChannel = 34;
    public static final int kRotateMotorChannel = 32;

    // Algae encoder Channel
    public static final int kAlgaeEncoderChannelA = 4;

    // Algae intake speed percentage
    public static final double kIntakeFastSpeed = 0.5;
    public static final double kIntakeSlowSpeed = 0.1;
    public static final double kReIntakeSpeed = -0.3;

    // Algae intake rotate speed percentage
    public static final double kUpIntakeRotateSpeed = -0.5;
    public static final double kDownIntakeRotateSpeed = 0.4;

    // Algae rotate & front PID
    public static final double rotMotorUpPIDkP = 0.05;
    public static final double rotMotorUpPIDkI = 0;
    public static final double rotMotorUpPIDkD = 0;
    public static final double rotMotorDownPIDkP = 0.02;
    public static final double rotMotorDownPIDkI = 0;
    public static final double rotMotorDownPIDkD = 0;

    // Algae setpoint
    public static final double kMaxAngle = 90;
    public static final double kMinAngle = 0;
    public static final double kStepAngle = 0.1;

    // Algae inverted
    public static final Boolean kIntakeMotorInverted = true;
    public static final Boolean kRotateMotorInverted = true;
    public static final Boolean kAlgaeEncoderInverted = true;

    // Algae encoder
    public static final double kDistancePerPulse = 360.0 / 2048;
    public static final double kRotateEncoderOffset = 0.0;
    public static final double expectedZero = -265;
    public static final double fullRange = 360;
    public static final double kGetSecAlgaeAngle = 73;
  }

  public static final class CoralShooterConstant {
    public static final double kDistanceRange = 4.0;
    public static final int kMotorChannel = 33;
    public static final int kEncoderChannel = 3;
    // public static final double kShooterMotorFastSpeed = 0.25;
    public static final double kMotorSpeed = 0.195;
    public static final Boolean kMotorInverted = true;
    public static final Boolean kEncoderInverted = true;
    public static final double kEncoderFullRange = 360.0;
    public static final double kEncoderOffset = 0.0;

    public static final int kAddressableLEDChannel = 0;
  }

  public static final class ElevatorConstant {
    public static final double kP = 0.023;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final int kLeftElevatorMotorChannel = 35;
    public static final int kRightElevatorMotorChannel = 36;
    public static final boolean kLeftMotorInverted = true;
    public static final boolean kRightMotorInverted = false;

    public static final int kEncoderChannelA = 1;
    public static final int kEncoderChannelB = 2;
    public static final boolean kEncoderInverted = false;

    public static final int kTouchSensorChannel = 0;

    public static final double kManualUpPower = 0.4;
    public static final double kManualDownPower = -0.2;

    public static final Distance kHeightOffset = Millimeters.of(430.0);

    public static final Distance kLowestHeight = Millimeters.of(430.0);
    public static final Distance kMaxHeight = Millimeters.of(1800);

    public static final double kMaxOutput = 0.4;
    public static final double kMinOutput = -0.1;

    public static final Distance kInitialHeight = Millimeters.of(430.0);
    public static final Distance kGetCarolHeight = Millimeters.of(485);
    public static final Distance kSecFloor = Millimeters.of(714);
    public static final Distance kTrdFloor = Millimeters.of(1206);
    public static final Distance kTopFloor = Millimeters.of(1770);
    public static final Distance kStepHeight = Millimeters.of(7);
    public static final Distance kToGetSecAlgaeHeight = Millimeters.of(810);
    public static final Distance kToGetTrdAlgaeHeight = Millimeters.of(950);
    public static final double kEncoderDistancePerPulse = (1.0 / 2048.0)
        * Inches.of(1.214 * Math.PI).in(Millimeters) * 2.0;
  }

  public static final class AutoConstants {
    public static final double kPTranslation = 6;
    public static final double kPRotation = 4;
    public static final double kIRotation = 0.5;
    public static final double kDRotation = 1;
  }
}