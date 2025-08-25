package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.FSLib.vision.LimelightHelpers;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;

public class SwerveDrive implements Swerve {
  private final Pigeon2 pigeon;

  private final SwerveModule[] modules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final SwerveDrivePoseEstimator estimator;

  private final DoublePublisher yawPublisher;
  private final StructPublisher<Pose2d> posePublisher;
  private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
  private final StructArrayPublisher<SwerveModuleState> desiredStatePublisher;

  public SwerveDrive() {
    pigeon = new Pigeon2(SwerveConstants.kPigeonId, RobotConstants.kAlternateBusName);

    modules = new SwerveModule[] {
        new SwerveModule(SwerveConstants.kFrontLeftModule),
        new SwerveModule(SwerveConstants.kFrontRightModule),
        new SwerveModule(SwerveConstants.kBackLeftModule),
        new SwerveModule(SwerveConstants.kBackRightModule)
    };

    kinematics = new SwerveDriveKinematics(
        new Translation2d[] {
            SwerveConstants.kFrontLeftModule.modulePosition(),
            SwerveConstants.kFrontRightModule.modulePosition(),
            SwerveConstants.kBackLeftModule.modulePosition(),
            SwerveConstants.kBackRightModule.modulePosition()
        });

    odometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());

    estimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), new Pose2d());

    yawPublisher = NetworkTableInstance.getDefault()
        .getTable("Swerve").getDoubleTopic("Yaw").publish();
    posePublisher = NetworkTableInstance.getDefault()
        .getTable("Swerve").getStructTopic("Pose", Pose2d.struct).publish();
    moduleStatePublisher = NetworkTableInstance.getDefault()
        .getTable("Swerve").getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    desiredStatePublisher = NetworkTableInstance.getDefault()
        .getTable("Swerve").getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load pathplanner config");
    }

    AutoBuilder.configure(
        this::getOdometryPosition,
        this::setOdometryPosition,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(10.0, 0.0, 0.0),
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

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  public void resetGyro(double deg) {
    pigeon.setYaw(deg);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kModuleMaxSpeed);
    desiredStatePublisher.set(moduleStates);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(moduleStates[i], true);
    }
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getYaw());
  }

  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.kModuleMaxSpeed);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(targetStates[i], false);
    }
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getModuleState();
    }
    return states;
  }

  public void updateOdometry() {
    odometry.update(getYaw(), getModulePositions());
    estimator.update(getYaw(), getModulePositions());
  }

  public Pose2d getOdometryPosition() {
    return odometry.getPoseMeters();
    // return estimator.getEstimatedPosition();
  }

  public void setOdometryPosition(Pose2d pose) {
    resetGyro(pose.getRotation().getDegrees());
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
    estimator.resetPosition(getYaw(), getModulePositions(), pose);
  }

  @Override
  public void periodic() {
    updateOdometry();
    LimelightHelpers.SetRobotOrientation(VisionConstants.kFrontCameraName, getYaw().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kFrontCameraName);
    if (limelightMeasurement != null) {
      if (limelightMeasurement.tagCount != 0 && Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble()) < 360) {
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
        // estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 0.7));
        estimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds
        );
      }
    }

    yawPublisher.set(getYaw().getRadians());
    posePublisher.set(getOdometryPosition());
    SmartDashboard.putNumber("odometry x", getOdometryPosition().getX());
    SmartDashboard.putNumber("odometry y", getOdometryPosition().getY());
    moduleStatePublisher.set(getModuleStates());
  }
}
