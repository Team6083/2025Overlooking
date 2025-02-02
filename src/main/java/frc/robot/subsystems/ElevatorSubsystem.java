// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  SparkMax elevatorMotor1;

  public ElevatorSubsystem() {
    elevatorMotor1 = new SparkMax(0, MotorType.kBrushless);
  }

  public void getSetPoint() {

  }

  public void elevatorPID() {

  }

  public void toTheSecFloor() {

  }

  public void toTheTrdFloor() {

  }

  public void toTheFourFloor() {
  }

  public void backToDefault() {
  }

  public void stopMove() {

  }

  public Command setCoralToSecFloor() {
    Command cmd = runEnd(this::toTheSecFloor, this::backToDefault);
    return cmd;
  }

  public Command setCoralToTrdFloor() {
    Command cmd = runEnd(this::toTheTrdFloor, this::backToDefault);
    return cmd;
  }

  public Command setCoralToFourFloor() {
    Command cmd = runEnd(this::toTheFourFloor, this::backToDefault);
    return cmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
