// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  SparkMax ElevatorMotor1;
  SparkMax ElevatorMotor2;

  public ElevatorSubsystem() {
    ElevatorMotor1 = new SparkMax(0, MotorType.kBrushless);
    ElevatorMotor2 = new SparkMax(2, MotorType.kBrushless);
  }

  public void getSetPoint() {

  }

  public void ElevatorPID() {

  }

  public void ToTheSecFloor() {

  }

  public void ToTheTrdFloor() {

  }

  public void TotheFourFloor() {
  }

  public void BackToDefault() {
  }

  public void stopMove() {

  }

  public Command SetCoralToSecFloor() {
    Command cmd = runEnd(this::ToTheSecFloor, this::BackToDefault);
    return cmd;
  }

  public Command SetCoralToTrdFloor() {
    Command cmd = runEnd(this::ToTheTrdFloor, this::BackToDefault);
    return cmd;
  }

  public Command SetCoralToFourFloor() {
    Command cmd = runEnd(this::TotheFourFloor, this::BackToDefault);
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
