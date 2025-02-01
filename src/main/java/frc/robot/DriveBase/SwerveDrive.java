package frc.robot.DriveBase;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveBaseConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.DriveBase.SwerveModule;

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

        private final PIDController trackingPID;

        // Method to drive the robot using joystick info
        double xSpeed;
        double ySpeed;
        double rot;
        boolean fieldRelative;

        private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        public SwerveDrive() {
                // 定義四隻腳的位置，單位為公尺
                frontLeftLocation = new Translation2d(DriveBaseConstants.kRobotLength / 2.0,
                                DriveBaseConstants.kRobotWidth / 2.0);
                frontRightLocation = new Translation2d(DriveBaseConstants.kRobotLength / 2.0,
                                -DriveBaseConstants.kRobotWidth / 2.0);
                backLeftLocation = new Translation2d(-DriveBaseConstants.kRobotLength / 2.0,
                                DriveBaseConstants.kRobotWidth / 2.0);
                backRightLocation = new Translation2d(-DriveBaseConstants.kRobotLength / 2.0,
                                -DriveBaseConstants.kRobotWidth / 2.0);

                frontLeft = new SwerveModule(DriveBaseConstants.kFrontLeftDriveMotorChannel,
                                DriveBaseConstants.kFrontLeftTurningMotorChannel,
                                DriveBaseConstants.kFrontLeftTurningEncoderChannel,
                                DriveBaseConstants.kFrontLeftCanCoder,
                                "frontLeft");
                frontRight = new SwerveModule(DriveBaseConstants.kFrontRightDriveMotorChannel,
                                DriveBaseConstants.kFrontRightTurningMotorChannel,
                                DriveBaseConstants.kFrontRightTurningEncoderChannel,
                                DriveBaseConstants.kFrontLeftCanCoder,
                                "frontRight");
                backLeft = new SwerveModule(DriveBaseConstants.kBackLeftDriveMotorChannel,
                                DriveBaseConstants.kBackLeftTurningMotorChannel,
                                DriveBaseConstants.kBackLeftTurningEncoderChannel, DriveBaseConstants.kBackLeftCanCoder,
                                "backLeft");
                backRight = new SwerveModule(DriveBaseConstants.kBackRightDriveMotorChannel,
                                DriveBaseConstants.kBackRightTurningMotorChannel,
                                DriveBaseConstants.kBackRightTurningEncoderChannel,
                                DriveBaseConstants.kBackRightCanCoder,
                                "backRight");

                gyro = new Pigeon2(1, "rio");

                // 定義 Swerve Drive 的 Kinematics 去將整體的速度算成各腳角度及速度
                kinematics = new SwerveDriveKinematics(
                                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

                                odometry = new SwerveDriveOdometry(
                                        kinematics,
                                        gyro.getRotation2d(),
                                        new SwerveModulePosition[] {
                                            frontLeft.getPosition(),
                                            frontRight.getPosition(),
                                            backLeft.getPosition(),
                                            backRight.getPosition()
                                        });

                // reset the gyro
                gyro.reset();
                // set the swerve speed equal 0
                drive(0, 0, 0, false);

                trackingPID = new PIDController(DriveBaseConstants.kTrackingP, DriveBaseConstants.kTrackingI,
                                DriveBaseConstants.kTrackingD);
                RobotConfig config;
                try {
                        config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                        // Handle exception as needed
                        e.printStackTrace();
                        return;
                }
                AutoBuilder.configure(
                                this::getPose2d, // Robot pose suppier
                                this::resetPose, // Method to reset odometry (will be called if your auto has a starting
                                                 // pose)
                                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the
                                                                                      // robot given ROBOT RELATIVE
                                                                                      // ChassisSpeeds
                                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live
                                                                // in your Constants class
                                                new com.pathplanner.lib.config.PIDConstants(AutoConstants.kPTranslation,
                                                                AutoConstants.kITranslation,
                                                                AutoConstants.kDTranslation), // Translation
                                                // PID
                                                // constants
                                                new com.pathplanner.lib.config.PIDConstants(AutoConstants.kPRotation,
                                                                AutoConstants.kIRotation, AutoConstants.kDRotation) // Rotation
                                // PID
                                ),
                                config,
                                () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
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

        public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
                swerveModuleStates = kinematics.toSwerveModuleStates(
                                fieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                                gyro.getRotation2d())
                                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveBaseConstants.kMaxSpeed);
                frontLeft.setDesiredState(swerveModuleStates[0]);
                frontRight.setDesiredState(swerveModuleStates[1]);
                backLeft.setDesiredState(swerveModuleStates[2]);
                backRight.setDesiredState(swerveModuleStates[3]);
        }

        public Pose2d getPose2d() {
                return odometry.getPoseMeters();
        }

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
}
