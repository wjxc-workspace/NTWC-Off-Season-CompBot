package frc.robot.subsystems.superstructure;

import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.FSLib.statemachine.StateSubsystemBase;
import frc.robot.LEDCenter;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.superstructure.componenets.Elevator;
import frc.robot.subsystems.superstructure.componenets.Grabber;

public class Superstructure extends StateSubsystemBase<SuperstructureState> {
  private final SuperstructureMech mech = new SuperstructureMech();
  private final Elevator elevator = new Elevator(mech);
  private final Grabber grabber = new Grabber(mech);

  private final Map<SuperstructureState, SuperstructureState> ejectStateMap = Map.of(
    SuperstructureState.kL1, SuperstructureState.kL1Eject,
    SuperstructureState.kL2, SuperstructureState.kL2Eject,
    SuperstructureState.kL3, SuperstructureState.kL3Eject
    // SuperstructureState.kL4, SuperstructureState.kL4Eject
  );

  private final Timer coralStationReverseTimer = new Timer();

  public Superstructure() {
    addComponents(elevator, grabber);
    createFSM(SuperstructureConstants.kAdjList, SuperstructureState.kDefault);
  }

  public void toEjectState() {
    SuperstructureState currentState = getCurrentState();
    SuperstructureState ejectState = ejectStateMap.get(currentState);
    if (ejectState == null)
      return;
    setGoalState(ejectState);
  }

  public Command toEjectStateCommand() {
    return Commands.defer(() -> {
      SuperstructureState currentState = getCurrentState();
      SuperstructureState ejectState = ejectStateMap.get(currentState);
      if (ejectState == null)
        return Commands.none();
      return setGoalStateCommand(ejectState);
    }, Set.of(this));
  }

  public boolean hasCoral() {
    return grabber.hasCoral();
  }

  public Command toggleHasCoral() {
    return Commands.runOnce(() -> grabber.toggleHasCoral()).ignoringDisable(true);
  }

  @Override
  protected void transitionAction(SuperstructureState state) {
    elevator.setHeight(state.getData().elevatorHeight());
    grabber.setAngle(state.getData().grabberAngle());
    grabber.setIntakeMotor(state.getData().grabberSpeed());
    if (state == SuperstructureState.kCoralStationReverse && !coralStationReverseTimer.isRunning()) {
      coralStationReverseTimer.start();
    }
    
    if (state == SuperstructureState.kDefault) {
      if (getCurrentState() == SuperstructureState.kDefault) {
        // stay at default
        LEDCenter.getInstance().setDefault();
      } else {
        // from any state to default
        LEDCenter.getInstance().setBlink();
      }
    } else if (state == SuperstructureState.kCoralStation) {
      // coral station instake
      LEDCenter.getInstance().setCoralIntake();
    } else {
      // not a good logic but works, when elevator is not at default
      if (elevator.getHeight() / ElevatorConstants.kL3Height > 0.1) {
        LEDCenter.getInstance().setProgress(elevator.getHeight() / ElevatorConstants.kL3Height);
      } else {
        LEDCenter.getInstance().setDefault();
      }
    }
  }

  @Override
  protected boolean isAtState(SuperstructureState state) {
    switch (state) {
      case kCoralStation:
        return hasCoral();
      case kL1Eject:
      case kL2Eject:
      case kL3Eject:
      // case kL4Eject:
        return !hasCoral();
      case kCoralStationReverse:
        if (coralStationReverseTimer.hasElapsed(0.5)) {
          coralStationReverseTimer.reset();
          return true;
        } else {
          return false;
        }
      default:
        return elevator.atHeightGoal() && grabber.atAngleGoal();
    }
  }

  @Override
  protected void onEndBehavior(SuperstructureState state) {
    elevator.stopElevatorMotor();
    grabber.stopAngleMotor();
    grabber.stopIntakeMotor();
  }

}
