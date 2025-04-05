package frc.robot.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConfigChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveBaseConstant;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class SwerveDrive extends SubsystemBase {
  public final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private final AHRS gyro;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
  private final StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("States", SwerveModuleState.struct).publish();

  private final StructArrayPublisher<Pose2d> field2dPublisher = NetworkTableInstance
      .getDefault().getStructArrayTopic("PoseArray", Pose2d.struct).publish();

  public SwerveDrive() {
    // 初始化 Swerve 模組
    frontLeft = new SwerveModule(
        DriveBaseConstant.kFrontLeftDriveMotorChannel,
        DriveBaseConstant.kFrontLeftTurningMotorChannel,
        DriveBaseConstant.kFrontLeftCanCoder,
        ConfigChooser.DriveBase.getBoolean("kFrontLeftDriveMotorInverted"),
        DriveBaseConstant.kFrontLeftTurningMotorInverted,
        ConfigChooser.DriveBase.getDouble("kFrontLeftCanCoderMagOffset"), "frontLeft");
    frontRight = new SwerveModule(
        DriveBaseConstant.kFrontRightDriveMotorChannel,
        DriveBaseConstant.kFrontRightTurningMotorChannel,
        DriveBaseConstant.kFrontRightCanCoder,
        ConfigChooser.DriveBase.getBoolean("kFrontRightDriveMotorInverted"),
        DriveBaseConstant.kFrontRightTurningMotorInverted,
        ConfigChooser.DriveBase.getDouble("kFrontRightCanCoderMagOffset"), "frontRight");
    backLeft = new SwerveModule(
        DriveBaseConstant.kBackLeftDriveMotorChannel,
        DriveBaseConstant.kBackLeftTurningMotorChannel,
        DriveBaseConstant.kBackLeftCanCoder,
        ConfigChooser.DriveBase.getBoolean("kBackLeftDriveMotorInverted"),
        DriveBaseConstant.kBackLeftTuringMotorInverted,
        ConfigChooser.DriveBase.getDouble("kBackLeftCanCoderMagOffset"), "backLeft");
    backRight = new SwerveModule(
        DriveBaseConstant.kBackRightDriveMotorChannel,
        DriveBaseConstant.kBackRightTurningMotorChannel,
        DriveBaseConstant.kBackRightCanCoder,
        ConfigChooser.DriveBase.getBoolean("kBackRightDriveMotorInverted"),
        DriveBaseConstant.kBackRightTurningMotorInverted,
        ConfigChooser.DriveBase.getDouble("kBackRightCanCoderMagOffset"), "backRight");

    SmartDashboard.putData("FrontLeft", frontLeft);
    SmartDashboard.putData("FrontRight", frontRight);
    SmartDashboard.putData("BackLeft", backLeft);
    SmartDashboard.putData("BackRight", backRight);

    // 初始化 Gyro
    gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    gyro.reset();

    // 設定四個 Swerve 模組在機器人上的相對位置，以機器人中心為原點 (0,0)，單位是 公尺
    Translation2d frontLeftLocation = new Translation2d(
        DriveBaseConstant.kRobotLength.div(2),
        DriveBaseConstant.kRobotWidth.div(2));
    Translation2d frontRightLocation = new Translation2d(
        DriveBaseConstant.kRobotLength.div(2),
        DriveBaseConstant.kRobotWidth.div(2).unaryMinus());
    Translation2d backLeftLocation = new Translation2d(
        DriveBaseConstant.kRobotLength.div(2).unaryMinus(),
        DriveBaseConstant.kRobotWidth.div(2));
    Translation2d backRightLocation = new Translation2d(
        DriveBaseConstant.kRobotLength.div(2).unaryMinus(),
        DriveBaseConstant.kRobotWidth.div(2).unaryMinus());

    // 定義 Kinematics 與 Odometry
    kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation);

    odometry = new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        getSwerveModulePosition());

    resetPose2dAndEncoder();
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
        this::getPose2d,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPTranslation),
            new PIDConstants(AutoConstants.kPRotation, AutoConstants.kIRotation, AutoConstants.kDRotation)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param ySpeed        Speed of the robot in the y direction (forward).
   * @param xSpeed        Speed of the robot in the x direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * 
   *                      using the wpi function to set the speed of the swerve
   */

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

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  // CHECKSTYLE.OFF: ParameterNameCheck
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot,
                gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // CHECKSTYLE.On: ParameterNameCheck
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveBaseConstant.kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  // 取得當前機器人在場地上的位置與角度
  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public SwerveModulePosition[] getSwerveModulePosition() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  // 重設機器人的位置與角度
  public void resetPose() {
    odometry.resetPosition(
        gyro.getRotation2d(),
        getSwerveModulePosition(),
        new Pose2d(0, 0, new Rotation2d(0)));
  }

  // 更新機器人的場地相對位置
  private void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        getSwerveModulePosition());
  }

  // 重置所有輪子的 Encoder 與機器人位置
  public void resetPose2dAndEncoder() {
    frontLeft.resetAllEncoder();
    frontRight.resetAllEncoder();
    backLeft.resetAllEncoder();
    backRight.resetAllEncoder();
    resetPose();
  }

  // 重置陀螺儀的角度
  public void resetGyro() {
    gyro.reset();
  }

  // 取得機器人目前的旋轉角度
  public Rotation2d getRotation2dDegrees() {
    return Rotation2d.fromDegrees(DriveBaseConstant.kGyroOffSet
        + ((DriveBaseConstant.kGyroInverted)
            ? (360.0 - gyro.getRotation2d().getDegrees())
            : gyro.getRotation2d().getDegrees()));
  }

  public void stop() {
    frontLeft.stopModule();
    frontRight.stopModule();
    backLeft.stopModule();
    backRight.stopModule();
  }

  public void setTurningDegree(double degree) {
    frontLeft.setTurningDegree(degree);
    frontRight.setTurningDegree(degree);
    backLeft.setTurningDegree(degree);
    backRight.setTurningDegree(degree);
  }

  public Command followPathCommand(String pathName) throws FileVersionException, IOException, ParseException {
    RobotConfig config;
    config = RobotConfig.fromGUISettings();

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return new FollowPathCommand(
        path,
        this::getPose2d,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () -> {

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  @Override
  public void periodic() {
    field2dPublisher.set(
        new Pose2d[] { getPose2d(),
            new Pose2d(0, 0, new Rotation2d(0)) });
    swervePublisher.set(swerveModuleStates);

    updateOdometry();

    SmartDashboard.putNumber("Gyro_Heading", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("PoseX", getPose2d().getX());
    SmartDashboard.putNumber("PoseY", getPose2d().getY());
    SmartDashboard.putNumber("PoseRotationDegree",
        getPose2d().getRotation().getDegrees());
  }

  public Command gyroResetCmd() {
    Command cmd = this.runOnce(this::resetGyro);
    cmd.setName("gyroResetCmd");
    return cmd;
  }

  public Command setTurningDegreeCmd(double degree) {
    Command cmd = this.runEnd(() -> setTurningDegree(degree), this::stop);
    cmd.setName("setTurningDegreeCmd");
    return cmd;
  }

  public Command driveForwardCmd() {
    Command cmd = this.runEnd(
        () -> drive(1, 0, 0, false), this::stop);
    return cmd;
  }
}
