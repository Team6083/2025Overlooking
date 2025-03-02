package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PowerDistributionConstant;

public class PowerDistribution {
  private edu.wpi.first.wpilibj.PowerDistribution powerDistribution;

  public PowerDistribution() {
    powerDistribution = new edu.wpi.first.wpilibj.PowerDistribution();
    SmartDashboard.putNumber("CoralShooterRightCurrent", 0);
    SmartDashboard.putNumber("CoralShooterLeftCurrent", 0);
    SmartDashboard.putNumber("AlgaeIntakeCurrent", 0);
    SmartDashboard.putNumber("AlgaeRotateCurrent", 0);
    SmartDashboard.putNumber("ClimberCurrent", 0);
    SmartDashboard.putNumber("RampCurrent", 0);
    SmartDashboard.putBoolean("IsCoralShooterOverCurrent", false);
    SmartDashboard.putBoolean("IsAlgaeIntakeOverCurrent", false);
    SmartDashboard.putBoolean("IsAlgaeRotateOverCurrent", false);
    SmartDashboard.putBoolean("IsClimberOverCurrent", false);
    SmartDashboard.putBoolean("IsRampOverCurrent", false);
  }

  public double coralShooterRightCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstant.kCoralShooterRightMotorCurrentChannel);
    SmartDashboard.putNumber("CoralShooterRightCurrent", current);
    return current;
  }

  public double coralShooterLeftCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstant.kCoralShooterLeftMotorCurrentChannel);
    SmartDashboard.putNumber("CoralShooterLeftCurrent", current);
    return current;
  }

  public double algaeIntakeCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstant.kAlgaeIntakeMotorCurrentChannel);
    SmartDashboard.putNumber("AlgaeIntakeCurrent", current);
    return current;
  }

  public double algaeRotateCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstant.kAlgaeRotateMotorCurrentChannel);
    SmartDashboard.putNumber("AlgaeRotateCurrent", current);
    return current;
  }

  public double climberCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstant.kClimberMotorCurrentChannel);
    SmartDashboard.putNumber("ClimberCurrent", current);
    return current;
  }

  public double rampCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstant.kRampMotorCurrentChannel);
    SmartDashboard.putNumber("RampCurrent", current);
    return current;
  }

  public boolean isCoralShooterOverCurrent() {
    var rightCurrent = coralShooterRightCurrent();
    var leftCurrent = coralShooterLeftCurrent();

    var isOverCurrent = rightCurrent > PowerDistributionConstant.kCoralShooterMotorMaxCurrent
        || leftCurrent > PowerDistributionConstant.kCoralShooterMotorMaxCurrent;

    SmartDashboard.putBoolean("IsCoralShooterOverCurrent", isOverCurrent);

    return isOverCurrent;
  }

  public boolean isAlgaeIntakeOverCurrent() {
    boolean isOverCurrent = algaeIntakeCurrent() > PowerDistributionConstant.kAlgaeIntakeMotorMaxCurrent;
    SmartDashboard.putBoolean("IsAlgaeIntakeOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isAlgaeRotateOverCurrent() {
    boolean isOverCurrent = algaeRotateCurrent() > PowerDistributionConstant.kAlgaeRotateMotorMaxCurrent;
    SmartDashboard.putBoolean("IsAlgaeRotateOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isClimberOverCurrent() {
    boolean isOverCurrent = climberCurrent() > PowerDistributionConstant.kClimberMotorMaxCurrent;
    SmartDashboard.putBoolean("IsClimberOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isRampOverCurrent() {
    boolean isOverCurrent = rampCurrent() > PowerDistributionConstant.kRampMotorMaxCurrent;
    SmartDashboard.putBoolean("IsRampOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

}