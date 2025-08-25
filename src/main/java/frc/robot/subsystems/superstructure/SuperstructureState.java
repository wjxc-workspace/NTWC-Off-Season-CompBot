package frc.robot.subsystems.superstructure;

import frc.robot.Constants.GrabberConstants;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import frc.robot.Constants.ElevatorConstants;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
  kDefault(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kDefaultHeight)
          .grabberAngle(GrabberConstants.kDefaultAngle)
          .grabberSpeed(0)
          .build()),
  kL1(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kL1Height)
          .grabberAngle(GrabberConstants.kL1Angle)
          .grabberSpeed(0)
          .build()),
  kL1Eject(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kL1Height)
          .grabberAngle(GrabberConstants.kL1Angle)
          .grabberSpeed(GrabberConstants.kOuttakeSpeed)
          .build()),
  kL2(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kL2Height)
          .grabberAngle(GrabberConstants.kL2Angle)
          .grabberSpeed(0)
          .build()),
  kL2Eject(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kL2Height)
          .grabberAngle(GrabberConstants.kL2Angle)
          .grabberSpeed(GrabberConstants.kOuttakeSpeed)
          .build()),
  kL3(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kL3Height)
          .grabberAngle(GrabberConstants.kL3Angle)
          .grabberSpeed(0)
          .build()),
  kL3Eject(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kL3Height)
          .grabberAngle(GrabberConstants.kL3Angle)
          .grabberSpeed(GrabberConstants.kOuttakeSpeed)
          .build()),
//   kL4(
//       SuperstructureStateData.builder()
//           .elevatorHeight(ElevatorConstants.kL4Height)
//           .grabberAngle(GrabberConstants.kL4Angle)
//           .grabberSpeed(0)
//           .build()),
//   kL4Eject(
//       SuperstructureStateData.builder()
//           .elevatorHeight(ElevatorConstants.kL4Height)
//           .grabberAngle(GrabberConstants.kL4Angle)
//           .grabberSpeed(GrabberConstants.kOuttakeSpeed)
//           .build()),
  kPreCoralStation(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kCoralStationHeight)
          .grabberAngle(GrabberConstants.kCoralStationAngle)
          .grabberSpeed(0)
          .build()),
  kCoralStation(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kCoralStationHeight)
          .grabberAngle(GrabberConstants.kCoralStationAngle)
          .grabberSpeed(GrabberConstants.kIntakeSpeed)
          .build()),
  kCoralStationReverse(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kCoralStationHeight)
          .grabberAngle(GrabberConstants.kCoralStationAngle)
          .grabberSpeed(GrabberConstants.kOuttakeSpeed)
          .build()),
  kL1Hit(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kL2Height - 0.08)
          .grabberAngle(GrabberConstants.kHitAngle)
          .grabberSpeed(GrabberConstants.kOuttakeSpeed)
          .build()),
  kL2Hit(
      SuperstructureStateData.builder()
          .elevatorHeight(ElevatorConstants.kL3Height - 0.1)
          .grabberAngle(GrabberConstants.kHitAngle)
          .grabberSpeed(GrabberConstants.kOuttakeSpeed)
          .build());;

  @Builder
  public record SuperstructureStateData(double elevatorHeight, double grabberAngle, double grabberSpeed) {
  }

  private final SuperstructureStateData data;
}
