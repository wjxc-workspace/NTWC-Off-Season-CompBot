package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveSim implements Swerve {
  private final SelfControlledSwerveDriveSimulation simulatedDrive;

  private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
    .getTable("Swerve").getStructTopic("Pose", Pose2d.struct).publish();

  public SwerveSim() {
    // Create and configure a drivetrain simulation configuration
    final DriveTrainSimulationConfig driveTrainConfig = DriveTrainSimulationConfig.Default()
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(COTS.ofMark4(
            DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
            DCMotor.getNEO(1), // Steer motor is a Falcon 500
            COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
            3)) // L3 Gear ratio
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30));

    simulatedDrive = new SelfControlledSwerveDriveSimulation(
        new SwerveDriveSimulation(driveTrainConfig, new Pose2d(2, 2, new Rotation2d())));

    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
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
        robotConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    simulatedDrive.runChassisSpeeds(
        new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
        new Translation2d(),
        fieldRelative,
        true);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), false, true);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    simulatedDrive.runSwerveStates(desiredStates);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
  }

  public Rotation2d getGyroYaw() {
    return simulatedDrive.getActualPoseInSimulationWorld().getRotation();
  }

  public Pose2d getOdometryPosition() {
    return simulatedDrive.getOdometryEstimatedPose();
  }

  public void setOdometryPosition(Pose2d pose) {
    simulatedDrive.setSimulationWorldPose(pose);
    simulatedDrive.resetOdometry(pose);
  }

  public void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds) {
    simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    simulatedDrive.periodic();
    posePublisher.set(getOdometryPosition());
  }
}
