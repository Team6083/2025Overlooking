package frc.robot.lib.sensor.distance;

public interface DistanceSensor {
  public double getRange();

  public boolean isRangeValid();

  public void setAutomaticMode(boolean modeOn);
}