// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.prefs.Preferences;
/** Add your docs here. */
public class SwerveDriveOffset {
    private double AUCoderMagOffsetFL = Constants.DriveBaseConstant.kFrontLeftCanCoderMagOffset;
    private double AUCoderMagOffsetFR = Constants.DriveBaseConstant.kFrontRightCanCoderMagOffset;
    private double AUCoderMagOffsetBL = Constants.DriveBaseConstant.kBackLeftCanCoderMagOffset;
    private double AUCoderMagOffsetBR = Constants.DriveBaseConstant.kBackRightCanCoderMagOffset;
    private String AUCoderMagOffset = Constants.DriveBaseConstant.AUCanCoderMagOffset;
    private Preferences preferences;
    public SwerveDriveOffset(){
    preferences.putDouble(AUCoderMagOffset, AUCoderMagOffsetFL);
    preferences.putDouble(AUCoderMagOffset, AUCoderMagOffsetFR);
    preferences.putDouble(AUCoderMagOffset, AUCoderMagOffsetBL);
    preferences.putDouble(AUCoderMagOffset, AUCoderMagOffsetBR);
    }
    public void loadPreferences() {
        if(AUCoderMagOffsetFL!=preferences.getDouble(AUCoderMagOffset, AUCoderMagOffsetFL)){
            AUCoderMagOffsetFL =preferences.getDouble(AUCoderMagOffset, AUCoderMagOffsetFL);
        }
}
}
