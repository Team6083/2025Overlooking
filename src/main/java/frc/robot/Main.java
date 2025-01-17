// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.chrono.HijrahChronology;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}
HijrahChronology


  private Main() {
    System.err.println("hiiii");
  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
