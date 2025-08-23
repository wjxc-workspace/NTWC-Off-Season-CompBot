package frc.robot;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;

public class VisionSim {
  private final Swerve swerve;
  
  private final VisionSystemSim visionSim = new VisionSystemSim("main");
  private final PhotonCamera frontCamera;
  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;
  
  private final PhotonCameraSim frontCameraSim;
  private final PhotonCameraSim leftCameraSim;
  private final PhotonCameraSim rightCameraSim;

  @Getter
  private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  
  @Getter
  private PhotonPipelineResult frontCameraLatestResult;
  @Getter
  private PhotonPipelineResult leftCameraLatestResult;
  @Getter
  private PhotonPipelineResult rightCameraLatestResult;

  private final StructArrayPublisher<Pose3d> frontCameraVisiableTargetPublisher = NetworkTableInstance.getDefault()
    .getTable("Vision").getStructArrayTopic("FrontCameraTargets", Pose3d.struct).publish();

  public VisionSim(Swerve swerve) {
    this.swerve = swerve;

    visionSim.addAprilTags(tagLayout);

    SimCameraProperties frontCameraProp = new SimCameraProperties();
    frontCameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(62.5));
    frontCameraProp.setCalibError(0.25, 0.08);
    frontCameraProp.setFPS(20);
    frontCameraProp.setAvgLatencyMs(35);
    frontCameraProp.setLatencyStdDevMs(5);
    frontCamera = new PhotonCamera(VisionConstants.kFrontCameraName);
    frontCameraSim = new PhotonCameraSim(frontCamera, frontCameraProp);
    frontCameraSim.enableRawStream(true);
    frontCameraSim.enableProcessedStream(true);
    frontCameraSim.enableDrawWireframe(true);

    SimCameraProperties leftCameraProp = new SimCameraProperties();
    leftCameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(62.5));
    leftCameraProp.setCalibError(0.25, 0.08);
    leftCameraProp.setFPS(20);
    leftCameraProp.setAvgLatencyMs(35);
    leftCameraProp.setLatencyStdDevMs(5);
    leftCamera = new PhotonCamera(VisionConstants.kLeftCameraName);
    leftCameraSim = new PhotonCameraSim(leftCamera, leftCameraProp);
    leftCameraSim.enableRawStream(true);
    leftCameraSim.enableProcessedStream(true);
    leftCameraSim.enableDrawWireframe(true);

    SimCameraProperties rightCameraProp = new SimCameraProperties();
    rightCameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(62.5));
    rightCameraProp.setCalibError(0.25, 0.08);
    rightCameraProp.setFPS(20);
    rightCameraProp.setAvgLatencyMs(35);
    rightCameraProp.setLatencyStdDevMs(5);
    rightCamera = new PhotonCamera(VisionConstants.kRightCameraName);
    rightCameraSim = new PhotonCameraSim(rightCamera, rightCameraProp);
    rightCameraSim.enableRawStream(true);
    rightCameraSim.enableProcessedStream(true);
    rightCameraSim.enableDrawWireframe(true);

    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    visionSim.addCamera(frontCameraSim, VisionConstants.kRobotToFrontCamera);
    visionSim.addCamera(leftCameraSim, VisionConstants.kRobotToLeftCamera);
    visionSim.addCamera(rightCameraSim, VisionConstants.kRobotToRightCamera);
  }  

  public void simulationPeriodic() {
    visionSim.update(swerve.getOdometryPosition());
    swerve.addVisionMeasurement(visionSim.getRobotPose().toPose2d(), 0.02);

    List<Pose3d> frontCameraVisiableTargetPoses = new LinkedList<>();

    List<PhotonPipelineResult> frontCameraResults = frontCamera.getAllUnreadResults();
    if (!frontCameraResults.isEmpty()) {
      frontCameraLatestResult = frontCameraResults.get(frontCameraResults.size() - 1);
    }

    List<PhotonPipelineResult> leftCameraResults = leftCamera.getAllUnreadResults();
    if (!leftCameraResults.isEmpty()) {
      leftCameraLatestResult = leftCameraResults.get(leftCameraResults.size() - 1);
    }

    List<PhotonPipelineResult> rightCameraResults = rightCamera.getAllUnreadResults();
    if (!rightCameraResults.isEmpty()) {
      rightCameraLatestResult = rightCameraResults.get(rightCameraResults.size() - 1);
    }

    if (frontCameraLatestResult.hasTargets()) {
      for (PhotonTrackedTarget target : frontCameraLatestResult.getTargets()) {
        if (target.getPoseAmbiguity() < 0.2 && target.getBestCameraToTarget().getTranslation().getNorm() < 2) {
          frontCameraVisiableTargetPoses.add(tagLayout.getTagPose(target.fiducialId).get());
        }
      }
    }

    frontCameraVisiableTargetPublisher.set(frontCameraVisiableTargetPoses.toArray(new Pose3d[frontCameraVisiableTargetPoses.size()]));
  }
}
