package frc.robot;

public class Constants {

  public final static class ClimberConstant {
    public final static int kClimberSetpoint = 40;
    public final static double kClimbDownSpeed = -0.35;
  }

  public final static class CoralShooterConstant {
    public final static int kOnboard = 0;
    public final static double kDistanceRange = 4;
    public final static int kShooterMotorChannel = 1;
    public final static double kShooterMotorSpeed = 4;
  }

  public final static class AlgaeIntakeConstant {
    public final static int kIntakeMotorChannel = 2;
    public final static int kRotateMotorChannel = 1;
    public final static double kIntakeVoltage = 6.0;
    public final static double kReIntakeVoltage = 3.0;
    public final static double kUpIntakeVoltage = 12.0;
    public final static double kDownIntakeVoltage = -12.0;
  }

}
