package frc.robot;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;

public class Constants {

  public static final class ClimberConstant {
    public static final int kClimberSetpoint = 40;
    public static final double kClimbDownSpeed = -0.35;
  }

  public static final class CoralShooterConstant {
    public static final int kOnboard = 0;
    public static final double kDistanceRange = 4;
    public static final int kShooterMotorChannel = 1;
    public static final double kShooterMotorSpeed = 4;
  }

  public static final class AlgaeIntakeConstant {
    public static final int kIntakeMotorChannel = 2;
    public static final int kRotateMotorChannel = 1;
    public static final double kIntakeVoltage = 6.0;
    public static final double kReIntakeVoltage = 3.0;
    public static final double kUpIntakeVoltage = 12.0;
    public static final double kDownIntakeVoltage = -12.0;
  }

  public static final class PowerDistributionConstant {
    // Motor channel
    public static final int kCoralShooterMotorCurrentchannel = 7;
    public static final int kAlgaeIntakeMotorCurrentchannel = 7;
    public static final int kAlgaeRotateMotorCurrentchannel = 7;
    public static final int kClimberMotorCurrentchannel = 7;
    public static final int kRampMotorCurrentchannel = 7;
    // Motor Max Current
    public static final double kCoralShooterMotorMaxCurrent = 40;
    public static final double kAlgaeIntakeMotorMaxCurrent = 40;
    public static final double kAlgaeRotateMotorMaxCurrent = 40;
    public static final double kClimberMotorMaxCurrent = 40;
    public static final double kRampMotorMaxCurrent = 40;
  }

  public static final class ElevatorConstant {
    public static final double kP = 0.0003;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final Distance kInitialHeight = Millimeters.of(0.0);
    public static final Distance kLowestHeight = Millimeters.of(0.0);
    public static final Distance kStartedOffset = Millimeters.of(0.0);
    public static final Distance kSecFloor = Millimeters.of(500);
    public static final Distance kTrdFloor = Millimeters.of(1000);
    public static final Distance kTopFloor = Millimeters.of(1500);
    public static final Distance kMaxHeight = Millimeters.of(2000);
    public static final Distance kStepHeight = Millimeters.of(10);

    public static final double kEncoderDistancePerPulse = 1.0 / 2048.0;

  }

}
