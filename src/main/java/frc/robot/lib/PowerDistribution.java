package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PowerDistributionConstant;

public class PowerDistribution {
  private edu.wpi.first.wpilibj.PowerDistribution powerDistribution;

  public PowerDistribution() {
    powerDistribution = new edu.wpi.first.wpilibj.PowerDistribution();
    SmartDashboard.putNumber("coralShooterRightCurrent", 0);
    SmartDashboard.putNumber("coralShooterLeftCurrent", 0);
    SmartDashboard.putNumber("algaeIntakeCurrent", 0);
    SmartDashboard.putNumber("algaeRotateCurrent", 0);
    SmartDashboard.putNumber("climberCurrent", 0);
    SmartDashboard.putNumber("rampCurrent", 0);
    SmartDashboard.putBoolean("isCoralShooterOverCurrent", false);
    SmartDashboard.putBoolean("isAlgaeIntakeOverCurrent", false);
    SmartDashboard.putBoolean("isAlgaeRotateOverCurrent", false);
    SmartDashboard.putBoolean("isClimberOverCurrent", false);
    SmartDashboard.putBoolean("isRampOverCurrent", false);
  }

  public double coralShooterRightCurrent() {
    double current = powerDistribution
        .getCurrent(PowerDistributionConstant.kCoralShooterRightMotorCurrentChannel);
    SmartDashboard.putNumber("coralShooterRightCurrent", current);
    return current;
  }

  public double coralShooterLeftCurrent() {
    double current = powerDistribution
        .getCurrent(PowerDistributionConstant.kCoralShooterLeftMotorCurrentChannel);
    SmartDashboard.putNumber("coralShooterLeftCurrent", current);
    return current;
  }

  public double algaeIntakeCurrent() {
    double current = powerDistribution
        .getCurrent(PowerDistributionConstant.kAlgaeIntakeMotorCurrentChannel);
    SmartDashboard.putNumber("algaeIntakeCurrent", current);
    return current;
  }

  public double algaeRotateCurrent() {
    double current = powerDistribution
        .getCurrent(PowerDistributionConstant.kAlgaeRotateMotorCurrentChannel);
    SmartDashboard.putNumber("algaeRotateCurrent", current);
    return current;
  }

  public double climberCurrent() {
    double current = powerDistribution
        .getCurrent(PowerDistributionConstant.kClimberMotorCurrentChannel);
    SmartDashboard.putNumber("climberCurrent", current);
    return current;
  }

  public double rampCurrent() {
    double current = powerDistribution
        .getCurrent(PowerDistributionConstant.kRampMotorCurrentChannel);
    SmartDashboard.putNumber("rampCurrent", current);
    return current;
  }

  public boolean isCoralShooterOverCurrent() {
    boolean isRightMotorOverCurrent = 
        coralShooterRightCurrent() > PowerDistributionConstant.kCoralShooterMotorMaxCurrent;
    boolean isLeftMotorOverCurrent =  
        coralShooterLeftCurrent() > PowerDistributionConstant.kCoralShooterMotorMaxCurrent;
    SmartDashboard.putBoolean("isCoralShooterOverCurrent",
        isLeftMotorOverCurrent || isRightMotorOverCurrent);
    return isLeftMotorOverCurrent || isRightMotorOverCurrent;
  }

  public boolean isAlgaeIntakeOverCurrent() {
    boolean isOverCurrent = 
        algaeIntakeCurrent() > PowerDistributionConstant.kAlgaeIntakeMotorMaxCurrent;
    SmartDashboard.putBoolean("isAlgaeIntakeOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isAlgaeRotateOverCurrent() {
    boolean isOverCurrent = 
        algaeRotateCurrent() > PowerDistributionConstant.kAlgaeRotateMotorMaxCurrent;
    SmartDashboard.putBoolean("isAlgaeRotateOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isClimberOverCurrent() {
    boolean isOverCurrent = 
        climberCurrent() > PowerDistributionConstant.kClimberMotorMaxCurrent;
    SmartDashboard.putBoolean("isClimberOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isRampOverCurrent() {
    boolean isOverCurrent = 
        rampCurrent() > PowerDistributionConstant.kRampMotorMaxCurrent;
    SmartDashboard.putBoolean("isRampOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

}