package frc.robot.subsystems.intake;

import frc.FSLib.statemachine.StateSubsystemBase;
import frc.robot.Constants.IntakerConstants;
import frc.robot.subsystems.intake.components.Intaker;

public class Intake extends StateSubsystemBase<IntakeState> {
  private final IntakeMech mech = new IntakeMech();
  private final Intaker intaker = new Intaker(mech);

  public Intake() {
    addComponents(intaker);
    createFSM(IntakerConstants.adjList, IntakeState.kDefault);
  }

  @Override
  protected void transitionAction(IntakeState state) {
    intaker.setElbowAngle(state.getData().intakerAngle());
    intaker.setIntakeMotorSpeed(state.getData().intakerSpeed());
  }

  @Override
  protected boolean isAtState(IntakeState state) {
    switch (state) {
      case kIntake:
        return hasAlgae();
      case kEject:
        return !hasAlgae();
      default:
        return intaker.atAngleGoal();
    }
  }

  @Override
  protected void onEndBehavior(IntakeState state) {
    intaker.stopAngleMotor();
    intaker.stopIntakeMotor();
  }

  public boolean hasAlgae() {
    return intaker.hasAlgae();
  }
}
