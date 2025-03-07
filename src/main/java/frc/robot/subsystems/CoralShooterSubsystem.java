// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralShooterConstant;
import frc.robot.lib.PowerDistribution;

public class CoralShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralShooterSubsystem. */
  private PowerDistribution powerDistribution;
  private Rev2mDistanceSensor distanceSensor;
  private VictorSPX coralShooterMotor;
  private DutyCycleEncoder coralShooterEncoder;
  private Solenoid ledLeft;
  private Solenoid ledRight;
  private Boolean isInBlink = false;

  public CoralShooterSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;

    // motor
    coralShooterMotor = new VictorSPX(CoralShooterConstant.kMotorChannel);
    coralShooterMotor.setInverted(CoralShooterConstant.kMotorInverted);

    // encoder
    coralShooterEncoder = new DutyCycleEncoder(
        CoralShooterConstant.kEncoderChannel,
        CoralShooterConstant.kEncoderFullRange,
        CoralShooterConstant.kEncoderOffset);
    coralShooterEncoder.setInverted(CoralShooterConstant.kEncoderInverted);

    // distance sensor
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    distanceSensor.setAutomaticMode(true);

    ledLeft = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    ledRight = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  }

  public double getEncoder() {
    return coralShooterEncoder.get();
  }

  public void setMotorSpeed(double speed) {
    coralShooterMotor.set(VictorSPXControlMode.PercentOutput, speed);
  }

  // shooter on
  public void coralShooterIn() {
    if (powerDistribution.isCoralShooterOverCurrent()) {
      setMotorSpeed(0);
      return;
    }
    setMotorSpeed(CoralShooterConstant.kMotorSpeed);
  }

  public void coralShooterOut() {
    if (powerDistribution.isCoralShooterOverCurrent()) {
      setMotorSpeed(0);
      return;
    }
    setMotorSpeed(CoralShooterConstant.kShooterMotorFastSpeed);
  }

  public void coralShooterStop() {
    setMotorSpeed(0);
  }

  public boolean isGetTarget() {
    if (distanceSensor.getRange() <= CoralShooterConstant.kDistanceRange && distanceSensor.getRange() > 0) {
      return true;
    }
    return false;
  }

  public void setLight(boolean isLightOn) {
    ledLeft.set(isLightOn);
    ledRight.set(isLightOn);
  }

  public void setLightBlink() {
    isInBlink = true;
    Thread thread = new Thread(() -> {
      if (isGetTarget()) {
        for (int i = 0; i < 2; i++) {
          ledLeft.set(true);
          ledRight.set(true);
          try {
            Thread.sleep(200);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
          ledLeft.set(false);
          ledRight.set(false);
          try {
            Thread.sleep(200);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        isInBlink = false;
      }
    });
    thread.start();
  }

  @Override
  public void periodic() {
    // put dashboard
    if(!isInBlink){
      setLight(isGetTarget());
    }
    SmartDashboard.putNumber("Distance", distanceSensor.getRange());
    SmartDashboard.putBoolean("IsGetTarget", isGetTarget());
    SmartDashboard.putNumber("CoralShooterEncoder", coralShooterEncoder.get());
  }

  public Command coralShooterInCmd() {
    Command cmd = runEnd(this::coralShooterIn, this::coralShooterStop);
    cmd.setName("coralShooterOn");
    return cmd;
  }

  public Command coralShooterOutCmd() {
    Command cmd = runEnd(this::coralShooterOut, this::coralShooterStop);
    cmd.setName("coralShooterOut");
    return cmd;
  }

  public Command coralShooterStopCmd() {
    Command cmd = runOnce(this::coralShooterStop);
    cmd.setName("coralShooterStop");
    return cmd;
  }

  public Command setLightBlinkCmd() {
    Command cmd = runOnce(() -> setLightBlink());
    cmd.setName("steLightBlink");
    return cmd;
  }
}