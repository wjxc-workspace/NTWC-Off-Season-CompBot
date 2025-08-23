package frc.robot.subsystems.intake;

import frc.robot.Constants.IntakerConstants;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public enum IntakeState {
  kDefault(
      IntakeStateData.builder()
          .intakerAngle(IntakerConstants.kDefaultAngle)
          .intakerSpeed(0)
          .build()),
  kIntake(
      IntakeStateData.builder()
          .intakerAngle(IntakerConstants.kIntakeAngle)
          .intakerSpeed(IntakerConstants.kIntakeSpeed)
          .build()),
  kHold(
      IntakeStateData.builder()
          .intakerAngle(IntakerConstants.kHoldingAngle)
          .intakerSpeed(IntakerConstants.kHoldSpeed)
          .build()),
  kEject(
      IntakeStateData.builder()
          .intakerAngle(IntakerConstants.kHoldingAngle)
          .intakerSpeed(IntakerConstants.kEjectSpeed)
          .build());

  @Builder
  public record IntakeStateData(double intakerAngle, double intakerSpeed) {
  }

  private final IntakeStateData data;
}
