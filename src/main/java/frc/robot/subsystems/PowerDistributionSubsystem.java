// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowerDistributionConstant;

public class PowerDistributionSubsystem extends SubsystemBase {
	private PowerDistribution powerDistribution;

	public PowerDistributionSubsystem() {
		powerDistribution = new PowerDistribution();
		SmartDashboard.putNumber("coralShooterCurrent", 0);
		SmartDashboard.putNumber("algaeIntakeCurrent", 0);
		SmartDashboard.putNumber("algaeRotateCurrent", 0);
		SmartDashboard.putNumber("climberCurrent", 0);
		SmartDashboard.putNumber("rampCurrent", 0);
		SmartDashboard.putBoolean("isCoralShooterOverCurrent", false);
		SmartDashboard.putBoolean("isAlgaeIntakeOverCurrent", false);
		SmartDashboard.putBoolean("isAlgaeRotateOverCurrent", false);
		SmartDashboard.putBoolean("isAlgaeRotateOverCurrent", false);
		SmartDashboard.putBoolean("isRampOverCurrent", false);
	}

	public double coralShooterCurrent() {
		double current = powerDistribution.getCurrent(PowerDistributionConstant.kCoralShooterMotorCurrentchannel);
		SmartDashboard.putNumber("coralShooterCurrent", current);
		return current;
	}

	public double algaeIntakeCurrent() {
		double current = powerDistribution.getCurrent(PowerDistributionConstant.kAlgaeIntakeMotorCurrentchannel);
		SmartDashboard.putNumber("algaeIntakeCurrent", current);
		return current;
	}

	public double algaeRotateCurrent() {
		double current = powerDistribution.getCurrent(PowerDistributionConstant.kAlgaeRotateMotorCurrentchannel);
		SmartDashboard.putNumber("algaeRotateCurrent", current);
		return current;
	}

	public double climberCurrent() {
		double current = powerDistribution.getCurrent(PowerDistributionConstant.kClimberMotorCurrentchannel);
		SmartDashboard.putNumber("climberCurrent", current);
		return current;
	}

	public double rampCurrent() {
		double current = powerDistribution.getCurrent(PowerDistributionConstant.kRampMotorCurrentchannel);
		SmartDashboard.putNumber("rampCurrent", current);
		return current;
	}

	public boolean isCoralShooterOverCurrent() {
		boolean isOverCurrent = coralShooterCurrent() > PowerDistributionConstant.kCoralShooterMotorMaxCurrent;
		SmartDashboard.putBoolean("isCoralShooterOverCurrent", isOverCurrent);
		return isOverCurrent;
	}

	public boolean isAlgaeIntakeOverCurrent() {
		boolean isOverCurrent = algaeIntakeCurrent() > PowerDistributionConstant.kAlgaeIntakeMotorMaxCurrent;
		SmartDashboard.putBoolean("isAlgaeIntakeOverCurrent", isOverCurrent);
		return isOverCurrent;
	}

	public boolean isAlgaeRotateOverCurrent() {
		boolean isOverCurrent = coralShooterCurrent() > PowerDistributionConstant.kAlgaeIntakeMotorMaxCurrent;
		SmartDashboard.putBoolean("isAlgaeRotateOverCurrent", isOverCurrent);
		return isOverCurrent;
	}

	public boolean isClimbeOverCurrent() {
		boolean isOverCurrent = climberCurrent() > PowerDistributionConstant.kClimberMotorMaxCurrent;
		SmartDashboard.putBoolean("isAlgaeRotateOverCurrent", isOverCurrent);
		return isOverCurrent;
	}

	public boolean isRampOverCurrent() {
		boolean isOverCurrent = rampCurrent() > PowerDistributionConstant.kRampMotorMaxCurrent;
		SmartDashboard.putBoolean("isRampOverCurrent", isOverCurrent);
		return isOverCurrent;
	}

}
