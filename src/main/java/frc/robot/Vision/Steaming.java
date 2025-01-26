// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Steaming extends SubsystemBase {
  /** Creates a new Steaming. */
  CvSink cvSink;

  public Steaming() {
    CameraServer.startAutomaticCapture();
    cvSink = CameraServer.getVideo();
  }

  public void putVideo(String name, int width, int height) {
    CameraServer.putVideo(name, width, height);
  } 

  public CvSink getFrame() {
    return CameraServer.getVideo();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
