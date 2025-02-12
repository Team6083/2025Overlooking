package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import java.util.Optional;

public class TagTracking {
  private final NetworkTable table;
  private final AprilTagFieldLayout layout;

  private double tv;
  private double tx;
  private double ty;
  private double id;

  private double[] bt; // botpose_targetspace
  private double[] ct; // camerapose_targetspace

  private double distance;
  private boolean isCamOn = true;

  public TagTracking() {
    table = NetworkTableInstance.getDefault().getTable("lyly");
    setCamMode();
    setLedMode(0);
    setPipeline(0);
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
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
    ct = table.getEntry("camtran").getDoubleArray(new double[6]);
    return ct;
  }

  public double getTx() {
    tx = table.getEntry("tx").getDouble(0);
    return tx;
  }

  public double getTy() {
    ty = table.getEntry("ty").getDouble(0);

    return ty;
  }

  public double getTv() {
    tv = table.getEntry("tv").getDouble(0);
    return tv;
  }

  public double getTid() {
    id = table.getEntry("tid").getDouble(0);
    return id;
  }

  public double[] getBt() {
    bt = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    return bt;
  }

  public double getDistance() {
    // readValue();
    // CHECKSTYLE.OFF: LocalVariableName
    double targetHeight = getBt()[1]; // botpose in targetspace y
    double xDis = getBt()[0];
    double zDis = getBt()[2];
    double horDis = Math.sqrt(Math.pow(xDis, 2) + Math.pow(zDis, 2));
    distance = Math.sqrt(Math.pow(targetHeight, 2) + Math.pow(horDis, 2));
    return distance;
    // CHECKSTYLE.ON: LocalVariableName
  }

  public Pose2d getTagPose2d() {
    if (getTv() == 1) {
      Optional<Pose3d> tagPose3d = layout.getTagPose((int) getTid());
      Pose2d tagPose2d = tagPose3d.isPresent()
          ? tagPose3d.get().toPose2d()
          : new Pose2d();
      return tagPose2d;
    } else {
      return new Pose2d();
    }
  }

  public Pose3d getTagPose3d() {
    if (getTv() == 1) {
      Optional<Pose3d> tagPose3d = layout.getTagPose((int) getTid());
      Pose3d tagPose = tagPose3d.isPresent() ? tagPose3d.get() : new Pose3d();
      return tagPose;
    } else {
      return new Pose3d();
    }
  }

  public Pose2d getDesiredTagPose2d(double index) {
    if (getTv() == 1) {
      Optional<Pose3d> tagPose3d = layout.getTagPose((int) index);
      Pose2d tagPose2d = tagPose3d.isPresent() ? tagPose3d.get().toPose2d() : new Pose2d();
      return tagPose2d;
    } else {
      return new Pose2d();
    }
  }

  public void isVisionOn() {
    isCamOn = !isCamOn;
  }

  public void setAutoCamOn() {
    table.getEntry("camMode").setNumber(0);
  }

  public void setAutoCamOff() {
    table.getEntry("camMode").setNumber(1);
  }
}
