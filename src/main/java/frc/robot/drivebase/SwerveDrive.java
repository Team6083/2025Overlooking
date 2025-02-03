package frc.robot.drivebase;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveBaseConstants;

public class SwerveDrive extends SubsystemBase {
  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;
   
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final Pigeon2 gyro;

  private double magnification;

  private final Field2d field2d;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  public SwerveDrive() {
    // 設定四個 Swerve 模組在機器人上的相對位置，以機器人中心為原點 (0,0)，單位是 公尺
    frontLeftLocation = new Translation2d(DriveBaseConstants.kRobotLength / 2.0,
                                DriveBaseConstants.kRobotWidth / 2.0);
    frontRightLocation = new Translation2d(DriveBaseConstants.kRobotLength / 2.0,
                                -DriveBaseConstants.kRobotWidth / 2.0);
    backLeftLocation = new Translation2d(-DriveBaseConstants.kRobotLength / 2.0,
                                DriveBaseConstants.kRobotWidth / 2.0);
    backRightLocation = new Translation2d(-DriveBaseConstants.kRobotLength / 2.0,
                                -DriveBaseConstants.kRobotWidth / 2.0);

    // 初始化 Swerve 模組
    frontLeft = new SwerveModule(DriveBaseConstants.kFrontLeftDriveMotorChannel,
                                DriveBaseConstants.kFrontLeftTurningMotorChannel,
                                DriveBaseConstants.kFrontLeftTurningEncoderChannel,
                                DriveBaseConstants.kFrontLeftCanCoder,
                                "frontLeft");
    frontRight = new SwerveModule(DriveBaseConstants.kFrontRightDriveMotorChannel,
                                DriveBaseConstants.kFrontRightTurningMotorChannel,
                                DriveBaseConstants.kFrontRightTurningEncoderChannel,
                                DriveBaseConstants.kFrontRightCanCoder,
                                "frontRight");
    backLeft = new SwerveModule(DriveBaseConstants.kBackLeftDriveMotorChannel,
                                DriveBaseConstants.kBackLeftTurningMotorChannel,
                                DriveBaseConstants.kBackLeftTurningEncoderChannel,
                                DriveBaseConstants.kBackLeftCanCoder,
                                "backLeft");
    backRight = new SwerveModule(DriveBaseConstants.kBackRightDriveMotorChannel,
                                DriveBaseConstants.kBackRightTurningMotorChannel,
                                DriveBaseConstants.kBackRightTurningEncoderChannel,
                                DriveBaseConstants.kBackRightCanCoder,
                                "backRight");

    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);

    // 初始化 Gyro
    gyro = new Pigeon2(1, "rio");

    // 定義 Kinematics 與 Odometry
    kinematics = new SwerveDriveKinematics(
                                frontLeftLocation, frontRightLocation,
                                backLeftLocation, backRightLocation);

    odometry = new SwerveDriveOdometry(
                                kinematics,
                                gyro.getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                backLeft.getPosition(),
                                                backRight.getPosition()
                                });

    field2d = new Field2d();

    // reset the gyro
    gyro.setYaw(0);

    // set the swerve speed equal 0
    drive(0, 0, 0, false);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param yspeed       Speed of the robot in the y direction (forward).
   * @param xspeed       Speed of the robot in the x direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * 
   *                      using the wpi function to set the speed of the swerve
   */

  public void drive(double xspeed, double yspeed, double rot, boolean fieldRelative) {
    swerveModuleStates = kinematics.toSwerveModuleStates(
                                fieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                                                xspeed, yspeed, rot,
                                                                gyro.getRotation2d())
                                                : new ChassisSpeeds(xspeed, yspeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
                                swerveModuleStates, DriveBaseConstants.kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  // 取得當前機器人在場地上的位置與角度
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  // 重設機器人的位置與角度
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
                                gyro.getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                backLeft.getPosition(),
                                                backRight.getPosition()
                                },
                                pose);
  }

  // 更新機器人的場地相對位置
  public void updateOdometry() {
    odometry.update(
                gyro.getRotation2d(),
                new SwerveModulePosition[] {
                                        frontLeft.getPosition(),
                                        frontRight.getPosition(),
                                        backLeft.getPosition(),
                                        backRight.getPosition()
                });
  }

  // 重置所有輪子的 Encoder 與機器人位置
  public void resetPose2dAndEncoder() {
    frontLeft.resetAllEncoder();
    frontRight.resetAllEncoder();
    backLeft.resetAllEncoder();
    backRight.resetAllEncoder();
    resetPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  // 重置陀螺儀的角度
  public void resetGyro() {
    gyro.reset();
  }

  // 取得機器人目前的旋轉角度
  public Rotation2d getRotation2dDegrees() {
    return Rotation2d.fromDegrees(DriveBaseConstants.kGyroOffSet
                                + ((DriveBaseConstants.kGyroInverted) 
                                ? (360.0 - gyro.getRotation2d().getDegrees())
                                : gyro.getRotation2d().getDegrees()));
  }

  public void setMagnification(double magnification) {
    this.magnification = magnification;
  }

  public double getMagnification() {
    return magnification;
  }

  public void putDashboard() {
    SmartDashboard.putNumber("gyro_heading", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("poseX", getPose2d().getX());
    SmartDashboard.putNumber("poseY", getPose2d().getY());
    SmartDashboard.putNumber("poseRotationDegree",
                                getPose2d().getRotation().getDegrees());
  }

  @Override
  public void periodic() {
    updateOdometry();
    field2d.setRobotPose(getPose2d());
    putDashboard();
  }

  public Command gyroResetCmd() {
    Command cmd = this.runOnce(this::resetGyro);
    cmd.setName("gyroResetCmd");
    return cmd;
  }

}
