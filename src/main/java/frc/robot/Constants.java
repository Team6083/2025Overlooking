package frc.robot;

public class Constants {

    public final static class ClimberConstant {
        public static final int kClimberSetpoint = 40;
        public static final double kClimbDownSpeed = -0.35;
    }

    public static final class ModuleConstants {
        public static final double kDdriveClosedLoopRampRate = 0.1;
        public static final double kTurningClosedLoopRampRate = 0.1;
        public static final double kMaxModuleDriveVoltage = 12.0;
        public static final double kMaxModuleTurningVoltage = 4.0;
        public static final double kDesireSpeedtoMotorVoltage = 6.0;
        public static final double kModuleGearRate = 6.75;
        public static final double kWheelGearRate = 0.046;
        public static final double kWheelRadius = 1;
        public static final double kMinSpeed = 1;
    }
    

}
