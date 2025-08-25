package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.FSLib.swerve.SwerveModuleConstants;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.superstructure.SuperstructureState;

public class Constants {
  public static final class RobotConstants {
    public static final String kAlternateBusName = "GTX7130";
  }

  public static final class ButtonBoxConstants {
    public enum Port {
      k1, k2
    }

    public enum ButtonType {
      kAxis, kButton
    }

    public enum Button {
      kRightCoralStation(Port.k2, 6, ButtonType.kButton),
      kLeftCoralStation(Port.k1, 11, ButtonType.kButton),
      kL1(Port.k1, 12, ButtonType.kButton),
      kL2(Port.k1, 1, ButtonType.kAxis, false),
      kL3(Port.k2, 4, ButtonType.kButton),
      kL4(Port.k2, 5, ButtonType.kButton),
      kRedLeft(Port.k2, 10, ButtonType.kButton),
      kRedRight(Port.k2, 7, ButtonType.kButton),
      kYellowLeftTop(Port.k2, 11, ButtonType.kButton),
      kA(Port.k1, 1, ButtonType.kAxis, true),
      kB(Port.k1, 5, ButtonType.kButton),
      kC(Port.k1, 6, ButtonType.kButton),
      kD(Port.k1, 1, ButtonType.kButton),
      kE(Port.k1, 10, ButtonType.kButton),
      kF(Port.k1, 9, ButtonType.kButton),
      kG(Port.k1, 8, ButtonType.kButton),
      kH(Port.k1, 7, ButtonType.kButton),
      kI(Port.k1, 4, ButtonType.kButton),
      kJ(Port.k1, 3, ButtonType.kButton),
      kK(Port.k1, 2, ButtonType.kButton),
      kL(Port.k2, 3, ButtonType.kButton);

      public final Port port;
      public final int id;
      public final ButtonType type;
      public final boolean positiveAxis;

      Button(Port port, int id, ButtonType type) {
        this.port = port;
        this.id = id;
        this.type = type;
        this.positiveAxis = true;
      }

      Button(Port port, int id, ButtonType type, boolean positiveAxis) {
        this.port = port;
        this.id = id;
        this.type = type;
        this.positiveAxis = positiveAxis;
      }
    }
  }

  public static final class SwerveConstants {
    public static final double kModuleMaxSpeed = 4.5; // meters per second
    public static final double kRotationalSpeed = 6; // rad per second

    public static final double kDriveGearRatio = 6.122449;
    public static final double kWheelPerimeter = 2 * Math.PI * Units.inchesToMeters(2);

    public static final double kWheelBase = 0.57785;

    public static final double kDriveS = 0;
    public static final double kDrivekV = 2.294455;

    public static final double kDriveP = 0;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;

    public static final double kSteerP = 3;
    public static final double kSteerI = 0;
    public static final double kSteerD = 0.1;

    public static final int kPigeonId = 40;

    public static final SwerveModuleConstants kFrontLeftModule = SwerveModuleConstants.builder()
      .driveMotorId(1)
      .steerMotorId(2)
      .cancoderId(0)
      .cancoderOffset(-0.281006)
      .modulePosition(new Translation2d(kWheelBase / 2.0, kWheelBase / 2.0))
      .build();

    public static final SwerveModuleConstants kBackLeftModule = SwerveModuleConstants.builder()
      .driveMotorId(11)
      .steerMotorId(12)
      .cancoderId(1)
      .cancoderOffset(0.136475)
      .modulePosition(new Translation2d(-kWheelBase / 2.0, kWheelBase / 2.0))
      .build();

    public static final SwerveModuleConstants kFrontRightModule = SwerveModuleConstants.builder()
      .driveMotorId(31)
      .steerMotorId(32)
      .cancoderId(3)
      .cancoderOffset(-0.294922)
      .modulePosition(new Translation2d(kWheelBase / 2.0, -kWheelBase / 2.0))
      .build();

    public static final SwerveModuleConstants kBackRightModule = SwerveModuleConstants.builder()
      .driveMotorId(21)
      .steerMotorId(22)
      .cancoderId(2)
      .cancoderOffset(-0.046631)
      .modulePosition(new Translation2d(-kWheelBase / 2.0, -kWheelBase / 2.0))
      .build();
  }

  public static final class SuperstructureConstants {
    public static final Map<SuperstructureState, List<SuperstructureState>> kAdjList = new HashMap<>();
    static {
      kAdjList.put(SuperstructureState.kDefault, List.of(
          SuperstructureState.kL1,
          SuperstructureState.kL2,
          SuperstructureState.kL3,
          // SuperstructureState.kL4,
          SuperstructureState.kL1Hit,
          SuperstructureState.kL2Hit,
          SuperstructureState.kPreCoralStation));
      kAdjList.put(SuperstructureState.kL1Hit, List.of(
          SuperstructureState.kDefault,
          SuperstructureState.kL2Hit));
      kAdjList.put(SuperstructureState.kL2Hit, List.of(
          SuperstructureState.kDefault,
          SuperstructureState.kL1Hit));
      kAdjList.put(SuperstructureState.kL1, List.of(
          SuperstructureState.kDefault,
          SuperstructureState.kL2,
          SuperstructureState.kL3,
          // SuperstructureState.kL4,
          SuperstructureState.kL1Eject));
      kAdjList.put(SuperstructureState.kL1Eject, List.of(
          SuperstructureState.kL1));
      kAdjList.put(SuperstructureState.kL2, List.of(
          SuperstructureState.kDefault,
          SuperstructureState.kL1,
          SuperstructureState.kL3,
          // SuperstructureState.kL4,
          SuperstructureState.kL2Eject));
      kAdjList.put(SuperstructureState.kL2Eject, List.of(
          SuperstructureState.kL2));
      kAdjList.put(SuperstructureState.kL3, List.of(
          SuperstructureState.kDefault,
          SuperstructureState.kL1,
          SuperstructureState.kL2,
          // SuperstructureState.kL4,
          SuperstructureState.kL3Eject));
      kAdjList.put(SuperstructureState.kL3Eject, List.of(
          SuperstructureState.kL3));
      // kAdjList.put(SuperstructureState.kL4, List.of(
      //     SuperstructureState.kDefault,
      //     SuperstructureState.kL1,
      //     SuperstructureState.kL2,
      //     SuperstructureState.kL3,
      //     SuperstructureState.kL4Eject));
      // kAdjList.put(SuperstructureState.kL4Eject, List.of(
      //     SuperstructureState.kL4));
      kAdjList.put(SuperstructureState.kPreCoralStation, List.of(
          SuperstructureState.kDefault,
          SuperstructureState.kCoralStation,
          SuperstructureState.kCoralStationReverse));
      kAdjList.put(SuperstructureState.kCoralStation, List.of(
          SuperstructureState.kPreCoralStation));
      kAdjList.put(SuperstructureState.kCoralStationReverse, List.of(
          SuperstructureState.kPreCoralStation));
    }
  }

  public static final class ElevatorConstants {
    // motor constants
    public static final int kLeftMotorId = 40;
    public static final int kRightMotorId = 41;

    // mechanical constants
    public static final double kGearRatio = 9.0;
    public static final double kDrumRadius = 0.02;
    public static final double kDrumPerimeter = 2 * Math.PI * kDrumRadius;
    public static final double kMaxHeight = 1.4;
    public static final double kMinHeight = -0.01;

    // physics constants
    public static final double kCarrageMass = 10;

    // state constants
    public static final double kDefaultHeight = 0;
    public static final double kL1Height = 0;
    public static final double kL2Height = 0.21;
    public static final double kL3Height = 0.62;
    public static final double kL4Height = 1.33;
    public static final double kCoralStationHeight = 0;

    // pid constants
    public static final double kP = 60;
    public static final double kI = 0.5;
    public static final double kD = 0;
    public static final double kPIDTolerance = 0.01;
  }

  public static final class GrabberConstants {
    // motor constants
    public static final int kLeftElbowMotorId = 44;
    public static final int kRightElbowMotorId = 45;
    public static final int kLeftMotorId = 43;
    public static final int kRightMotorId = 42;
    public static final int kEncoderId = 5;
    public static final int kElbowMotorCurrentLimit = 40;
    public static final int kIntakeMotorCurrentLimit = 40;

    public static final int kLimitSwitchPort = 9;

    // cancoder constants
    public static final double kAbsoluteSensorDiscontinuityPoint = 1;
    public static final double kMagnetOffset = -0.619629;
    public static final SensorDirectionValue kSensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    // mechanical constants
    public static final double kGearRatio = 36.0;
    public static final double kLength = 0.2;
    public static final double kMaxAngle = 2 * Math.PI;
    public static final double kMinAngle = 2.5;

    // physics constants
    public static final double kMOI = 0.2;

    // state constants
    public static final double kDefaultAngle = 4.71;
    public static final double kL1Angle = 5.67;
    public static final double kL2Angle = 5.67;
    public static final double kL3Angle = 5.67;
    public static final double kL4Angle = 5.65;
    public static final double kCoralStationAngle = 2.84;
    public static final double kHitAngle = 6.2;

    public static final double kIntakeSpeed = 0.5;
    public static final double kOuttakeSpeed = -0.3;

    public static final double kIntakeStallCurrent = 15;

    // pid constants
    public static final double kP = 3.5;
    public static final double kI = 0.8;
    public static final double kD = 0.01;
    public static final double kElbowAngleTolerance = 0.03;
  }

  public static final class IntakerConstants {
    // motor constants
    public static final int kIntakeAngleMasterId = 18;
    public static final int kIntakeMotorId = 45;

    // cancoder constants
    public static final int kEncoderId = 4;
    public static final double kMagnetOffset = -0.474121;

    // mechanical constants
    public static final double kGearRatio = 18;
    public static final double kLength = 0.5;
    public static final double kMaxAngle = 1.19;
    public static final double kMinAngle = -0.01;

    // state constants (rad)
    public static final double kIntakeAngle = 0.35;
    public static final double kHoldingAngle = 0.8;
    public static final double kDefaultAngle = 1.1;

    public static final double kIntakeSpeed = 0.5;
    public static final double kHoldSpeed = 0.4;
    public static final double kEjectSpeed = -1.0;

    // pid constants
    public static final double kP = 5.0;
    public static final double kI = 0.5;
    public static final double kD = 0.2;
    public static final double kPIDTolerance = 0.03;

    // control constants
    public static final double kStallCurrent = 20.0;

    public static final Map<IntakeState, List<IntakeState>> adjList = new HashMap<>();
    static {
      adjList.put(IntakeState.kDefault, List.of(
          IntakeState.kIntake));
      adjList.put(IntakeState.kIntake, List.of(
          IntakeState.kDefault,
          IntakeState.kHold));
      adjList.put(IntakeState.kHold, List.of(
          IntakeState.kEject));
      adjList.put(IntakeState.kEject, List.of(
          IntakeState.kDefault));
    }
  }

  public static final class HammerConstants {
    // motor constants
    public static final int kHamsterId = 5;

    // control constants
    public static final double kMoveUpVoltage = 6.0;
    public static final double kMoveDownVoltage = -6.0;
    public static final double kTimeToHitPosition = 0.5;
    public static final double kStallCurrent = 22;
  }

  public static final class VisionConstants {
    public static final String kFrontCameraName = "limelight-front";
    public static final String kLeftCameraName = "limelight-left";
    public static final String kRightCameraName = "limelight-right";

    public static final Transform3d kRobotToFrontCamera = new Transform3d(0.35191, 0.01, 0.1946, new Rotation3d());
    public static final Transform3d kRobotToLeftCamera = new Transform3d(0.21282, 0.28983, 0.703, new Rotation3d(0, Units.degreesToRadians(45), 0));
    public static final Transform3d kRobotToRightCamera = new Transform3d(0.21282, -0.28983, 0.703, new Rotation3d(0, Units.degreesToRadians(45), 0));
  }

  public static final class AutoScoreConstants {
    public static final double kDistanceToPreReef = 0.9;
    public static final double kDistanceToReef = 0.6;

    // y+ pointing right in Limelight's Robot Space
    public static final double kLeftReefPosition = 0.1602;
    public static final double kRightReefPosition = -0.1602;

    public static final Pose2d kRemoveLowAlgaeFirstPose = new Pose2d(-0.6, 0, Rotation2d.kZero);
    public static final Pose2d kRemoveLowAlgaeSecondPose = new Pose2d(-0.9, 0, Rotation2d.kZero);
    public static final Pose2d kRemoveHighAlgaeFirstPose = new Pose2d(-0.58, 0, Rotation2d.kZero);

    public static final double kDistanceP = 2.7;
    public static final double kRotationP = 0.058;
  }

  public static final class LEDConstants {
    public static final int kPort = 9;
    public static final int kLength = 36;
  }

}
