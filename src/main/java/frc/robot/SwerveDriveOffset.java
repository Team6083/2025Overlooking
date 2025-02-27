// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public class SwerveDriveOffset {
    private double AUCoderMagOffsetFL = Constants.DriveBaseConstant.kFrontLeftCanCoderMagOffset;
   
    public SwerveDriveOffset(){
 Preferences.initDouble(Constants.DriveBaseConstant.AUCanCoderMagOffset, AUCoderMagOffsetFL);
    }
    public void loadPreferences() {
        // Read Preferences for Arm setpoint and kP on entering Teleop
        AUCoderMagOffsetFL = Preferences.getDouble(Constants.DriveBaseConstant.AUCanCoderMagOffset,  AUCoderMagOffsetFL);
       
      }
}
