package frc.robot.lib.sensor.distance;

import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Rev2mDistanceSensor implements DistanceSensor, AutoCloseable, Sendable {

  private com.revrobotics.Rev2mDistanceSensor actualSensor;

  private final SimDevice simSensor;
  private final SimDouble simDistance;

  public Rev2mDistanceSensor(Port port) {
    simSensor = SimDevice.create("Sensor:Rev2mDistanceSensor", 0);
    if (simSensor != null) {
      simDistance = simSensor.createDouble("distance", Direction.kInput, 0);
      actualSensor = null;
    } else {
      simDistance = null;
      actualSensor = new com.revrobotics.Rev2mDistanceSensor(port);
    }
  }

  public com.revrobotics.Rev2mDistanceSensor getActualSensor() {
    return actualSensor;
  }

  public void setAutomaticMode(boolean enabling) {
    if (simSensor != null) {
      return;
    }
    actualSensor.setAutomaticMode(enabling);
  }

  public void resetSensor(Port port) {
    if (simSensor != null) {
      return;
    }

    if (actualSensor != null) {
      actualSensor.setAutomaticMode(false);
    }

    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }

    actualSensor = new com.revrobotics.Rev2mDistanceSensor(port);

  }

  @Override
  public double getDistance() {
    if (simDistance != null) {
      return simDistance.get();
    }
    return actualSensor.getRange();
  }

  @Override
  public boolean isDistanceValid() {
    if (simDistance != null) {
      return true;
    }
    return actualSensor.isRangeValid();
  }

  @Override
  public void close() {
    simSensor.close();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Distance", this::getDistance, null);
    builder.addBooleanProperty("DistanceValid", this::isDistanceValid, null);
  }

}