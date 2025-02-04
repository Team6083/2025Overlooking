package frc.robot;

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
    public static final int kalgaeEncoderChannelA = 3;
    public static final int kalgaeEncoderChannelB = 4;
    public static final double kIntakeVoltage = 6.0;
    public static final double kReIntakeVoltage = 3.0;
    public static final int kUpIntakeSetpoint = 130;
    public static final int kDownIntakeSetpoint = 30;
    public static final double UpMotorPIDkP = 0.8;
    public static final double UpMotorPIDkI = 0;
    public static final double UpMotorPIDkD = 0;
    public static final double DownMotorPIDkP = 0.8;
    public static final double DownMotorPIDkI = 0;
    public static final double DownMotorPIDkD = 0;
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
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kInitialHeight = 0.0;
    public static final double kSecFloor = 50.0;
    public static final double kTrdFloor = 100.0;
    public static final double kTopFloor = 150.0;

  }

}
