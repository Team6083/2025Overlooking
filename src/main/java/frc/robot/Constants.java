package frc.robot;

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

  public static final class TagTrackingConstant {
    public static final double kRampHeight = 0.0;
    public static final double kCamHeight = 0.615;
    public static final double kCamPitch = 10.0;
    public static final double kCamToRampDistance = 0.11;
  }

}
