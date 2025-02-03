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
    public static final int kRotateMotorChannel = 1;
    public static final double kIntakeVoltage = 6.0;
    public static final double kReIntakeVoltage = 3.0;
    public static final double kUpIntakeVoltage = 12.0;
    public static final double kDownIntakeVoltage = -12.0;
  }

  public static final class TagTrackingConstant {
    public static final double kRampHeight = 0.0;
    public static final double kCamHeight = 0.615;
    public static final double kCamPitch = 10.0;
    public static final double kCamToRampDistance = 0.11;
  }

}
