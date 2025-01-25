package frc.robot;

public class Constants {

    public final static class ClimberConstant {
        public static final int kClimberSetpoint = 40;
        public static final double kClimbDownSpeed = -0.35;
    }

    public final static class CoralShooterConstant {
        public static final int kOnboard = 0;
        public static final double kDistanceRange = 4;
        public static final int kShooterMotorChannel = 1;
        public static final double kShooterMotorSpeed = 4;
    }

    public final static class AlgaeIntakeConstant {
        public static final int kIntakeMotorChannel = 2;
        public static final int kRotateMotorChannel = 1;
        public static final double kIntakeVoltage = 6.0;
        public static final double kReIntakeVoltage = 3.0;
        public static final double kUpIntakeVoltage = 12.0;
        public static final double kDownIntakeVoltage = -12.0;
    }

}
