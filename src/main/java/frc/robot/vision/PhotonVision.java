// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PhotonVision extends SubsystemBase {
  /** Creates a new TagTrackingPhotonVIsion. */
  // Query the latest result from PhotonVision
  PhotonCamera camera;
  PhotonPipelineResult result;
  PhotonTrackedTarget target;

  public PhotonVision() {
    camera = new PhotonCamera("photonVision");
    result = camera.getLatestResult();
    target = result.getBestTarget();
  }

  public boolean hasTarget() {
    return result.hasTargets();
  }

  public List<PhotonTrackedTarget> getTargets() {
    return result.getTargets();
  }

  public double getYaw() {
    return target.getYaw();
  }

  public double getPitch() {
    return target.getPitch();
  }

  public double getArea() {
    return target.getArea();
  }

  public double getSkew() {
    return target.getSkew();
  }

  public Transform3d bestCameraToTarget() {
    return target.getBestCameraToTarget();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
