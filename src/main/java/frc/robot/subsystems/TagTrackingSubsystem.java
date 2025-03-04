// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.Optional;

public class TagTrackingSubsystem extends SubsystemBase {
  /** Creates a new TagTrackingSubsystem. */
  private final NetworkTable table;
  private final AprilTagFieldLayout layout;

  public TagTrackingSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight-lyly");
    setLedMode(0);
    setPipeline(0);
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
    } catch (IOException err) {
      throw new RuntimeException();
    }
  }

  private void setLedMode(int ledMode) {
    table.getEntry("ledMode").setNumber(ledMode);
  }

  private void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public double[] getCt() {
    double[] ct = table.getEntry("camtran").getDoubleArray(new double[6]);
    return ct;
  }

  public double getTx() {
    double tx = table.getEntry("tx").getDouble(0);
    return tx;
  }

  public double getTy() {
    double ty = table.getEntry("ty").getDouble(0);
    return ty;
  }

  public double getTv() {
    double tv = table.getEntry("tv").getDouble(0);
    return tv;
  }

  public double getTid() {
    double id = table.getEntry("tid").getDouble(0);
    return id;
  }

  public double[] getBt() {
    double[] bt = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    return bt;
  }

  public double[] getTr() {
    double[] tr = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    return tr;
  }

  public Pose2d getTagPose2d() {
    return getTagPose3d().toPose2d();
  }

  public Pose3d getTagPose3d() {
    return getDesiredTagPose3d(getTid());
  }

  public Pose3d getDesiredTagPose3d(double index) {
    if (getTv() == 1) {
      Optional<Pose3d> tagPose3d = layout.getTagPose((int) index);
      Pose3d tagPose = tagPose3d.isPresent() ? tagPose3d.get() : new Pose3d();
      return tagPose;
    } else {
      return new Pose3d();
    }
  }

  public Pose2d getDesiredTagPose2d(double index) {
    return getDesiredTagPose3d(index).toPose2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
