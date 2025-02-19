package frc.robot.lib.sensor.distance;

public interface DistanceSensor {
  public double getTargetDistance();

  public boolean isGetTarget();

  public void setAutomaticMode(boolean isEnable);
}