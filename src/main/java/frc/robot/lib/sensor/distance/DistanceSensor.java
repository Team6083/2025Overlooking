package frc.robot.lib.sensor.distance;

public interface DistanceSensor {
  public double getDistance();

  public boolean isDistanceValid();

  public void setAutomaticMode(boolean modeOn);
}