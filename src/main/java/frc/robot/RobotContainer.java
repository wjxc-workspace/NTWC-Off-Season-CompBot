// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AutoScoreManager.ScoreLevel;
import frc.robot.Constants.ButtonBoxConstants.Button;
import frc.robot.commands.auto.AlignToReef;
import frc.robot.commands.teleop.TeleopSwerveWithReduction;
import frc.robot.commands.test.TestSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveSim;

public class RobotContainer {
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandJoystick js1 = new CommandJoystick(1);
  private final CommandJoystick js2 = new CommandJoystick(2);
  private final ButtonBox buttonBox = new ButtonBox(js1, js2);

  private final Swerve swerve;
  private final Superstructure superstructure;
  private final Intake intake;

  private final VisionSim visionSim;

  private final TeleopSwerveWithReduction teleopSwerve;

  private final AutoScoreManager autoScoreManager;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    if (Robot.isReal()) {
      swerve = new SwerveDrive();
      visionSim = null;
    } else {
      swerve = new SwerveSim();
      visionSim = new VisionSim(swerve);
    }
    superstructure = new Superstructure();
    intake = new Intake();

    teleopSwerve = new TeleopSwerveWithReduction(swerve, driverXbox.getHID());

    autoScoreManager = new AutoScoreManager();

    String positionValues[] = {"kA", "kB", "kC", "kD", "kE", "kF", "kG", "kH", "kI", "kJ", "kK", "kL"};
    for (String position : positionValues) {
      NamedCommands.registerCommand("SetScorePosition" + position, Commands.runOnce(() -> autoScoreManager.setScorePosition(position)));
    }
    String levelValues[] = {"kL1", "kL2", "kL3", "kL4"};
    for (String level : levelValues) {
      NamedCommands.registerCommand("SetScoreLevel" + level, Commands.runOnce(() -> autoScoreManager.setScoreLevel(ScoreLevel.valueOf(level))));
      NamedCommands.registerCommand(level, superstructure.setGoalStateCommand(SuperstructureState.valueOf(level)));
    }
    NamedCommands.registerCommand("Score", autoScore());
    NamedCommands.registerCommand("Eject", superstructure.toEjectStateCommand());
    NamedCommands.registerCommand("CoralStation", superstructure.setGoalStateCommand(SuperstructureState.kCoralStation));
    
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();

    swerve.setDefaultCommand(teleopSwerve);

    SmartDashboard.putData("AutoChooser", autoChooser);
    SmartDashboard.putData(superstructure);
    SmartDashboard.putData(intake);
  }

  private void configureBindings() {
    // Commands for toggling the enable/disable of AutoScoreCoral/AutoRemoveAlgae
    buttonBox.getButton(Button.kRedLeft)
      .onTrue(Commands.runOnce(() -> autoScoreManager.toggleEnable()));

    // When "AutoScore" is enabled, the buttons register the autoscore commands
    // when disabled, it directly drives the superstructure to the corresponding
    // state
    String levelValues[] = {"kL1", "kL2", "kL3", "kL4"};
    for (String level : levelValues) {
      buttonBox.getButton(Button.valueOf(level))
        .onTrue(
          Commands.either(
            Commands.runOnce(() -> autoScoreManager.setScoreLevel(ScoreLevel.valueOf(level))).ignoringDisable(true),
            superstructure.setGoalStateCommand(SuperstructureState.valueOf(level)),
            autoScoreManager::isAutoScoreCoal
          )
          .ignoringDisable(true)
        );  
    }

    // Binding button box buttons to commands that set
    String positionValues[] = {"kA", "kB", "kC", "kD", "kE", "kF", "kG", "kH", "kI", "kJ", "kK", "kL"};
    for (String position : positionValues) {
      buttonBox.getButton(Button.valueOf(position))
        .onTrue(
          Commands.runOnce(() -> autoScoreManager.setScorePosition(position)).ignoringDisable(true)
        );
    }

    // Command for auto-scoring corals and removing algaes
    driverXbox.leftBumper()
      .onTrue(
        autoScore()
          .until(isDriving())
          .beforeStarting(() -> CommandScheduler.getInstance().schedule(rumbule(0.2, 0.3)))
          .finallyDo(() -> CommandScheduler.getInstance().schedule(rumbule(0.2, 0.3)))
      );

    // Command for eject coral and algae manually, since autoscore command
    // automatically eject coral, this is more like a debug one
    driverXbox.rightTrigger(0.1)
      .and(superstructure::hasCoral)
      .onTrue(
        superstructure.toEjectStateCommand()
          .finallyDo(
            () -> {
              superstructure.setGoalState(SuperstructureState.kDefault);
              CommandScheduler.getInstance().schedule(rumbule(0.2, 0.3));
            }
          )
        );

    // Coral station states
    driverXbox.povRight()
      .onTrue(
        superstructure.setGoalStateCommand(SuperstructureState.kCoralStation)
          .finallyDo(() -> superstructure.setGoalState(SuperstructureState.kDefault))
      );
    driverXbox.povLeft()
      .onTrue(
        superstructure.setGoalStateCommand(SuperstructureState.kCoralStation)
          .finallyDo(() -> superstructure.setGoalState(SuperstructureState.kDefault))
      );
    buttonBox.getButton(Button.kLeftCoralStation)
      .onTrue(
        superstructure.setGoalStateCommand(SuperstructureState.kCoralStation)
          .finallyDo(() -> superstructure.setGoalState(SuperstructureState.kDefault))
      );
    buttonBox.getButton(Button.kRightCoralStation)
      .onTrue(
        superstructure.setGoalStateCommand(SuperstructureState.kCoralStation)
          .finallyDo(() -> superstructure.setGoalState(SuperstructureState.kDefault))
      );

    // A special Command to reverse the grabber if the coral stucks in the grabber
    // during intake, i.e. the current state is PreCoralStation and the next state is CoralStation
    driverXbox.rightTrigger(0.1)
      .and(() -> superstructure.getNextState() == SuperstructureState.kCoralStation)
      .onTrue(
        superstructure.setGoalStateCommand(SuperstructureState.kCoralStationReverse)
          .andThen(superstructure.setGoalStateCommand(SuperstructureState.kCoralStation))
          .andThen(superstructure.setGoalStateCommand(SuperstructureState.kDefault))
      );

    // Intake algae
    driverXbox.b()
      .onTrue(
        intake.setGoalStateCommand(IntakeState.kIntake)
          .andThen(intake.setGoalStateCommand(IntakeState.kHold)) 
      );

    // Eject algae
    driverXbox.rightTrigger(0.1)
      .and(intake::hasAlgae)
      .onTrue(
        intake.setGoalStateCommand(IntakeState.kEject)
          .finallyDo(
            () -> {
              intake.setGoalState(IntakeState.kDefault);
              CommandScheduler.getInstance().schedule(rumbule(0.2, 0.3));
            }
          )
      );
    
    // all-to-default-state command
    driverXbox.y().onTrue(
      new ParallelCommandGroup(
        superstructure.setGoalStateCommand(SuperstructureState.kDefault),
        intake.setGoalStateCommand(IntakeState.kDefault)
      )
    );

    driverXbox.a().onTrue(superstructure.toggleHasCoral());

    // Hit algae states
    driverXbox.povDown().onTrue(superstructure.setGoalStateCommand(SuperstructureState.kL1Hit));
    driverXbox.povUp().onTrue(superstructure.setGoalStateCommand(SuperstructureState.kL2Hit));
  }

  public Trigger isDriving() {
    return
      driverXbox.axisGreaterThan(0, 0.1)
        .or(driverXbox.axisGreaterThan(1, 0.1))
        .or(driverXbox.axisGreaterThan(4, 0.1));
  }

  public Command rumbule(double seconds, double strength) {
    return 
      Commands.run(() -> driverXbox.setRumble(RumbleType.kBothRumble, strength))
        .withTimeout(seconds)
        .finallyDo(() -> driverXbox.setRumble(RumbleType.kBothRumble, 0));
  }

  public Command autoScore() {
    return new DeferredCommand(
      () -> {
        if (!autoScoreManager.isAutoScoreCoal() || !superstructure.hasCoral() || autoScoreManager.getScorePosition() == null) {
          return Commands.none();
        }
        return new
          ParallelCommandGroup(
            new AlignToReef(swerve, autoScoreManager.getScorePosition()),
            superstructure.setGoalStateCommand(autoScoreManager.getScoreLevel().getState())
          )
          .andThen(superstructure.toEjectStateCommand())
          .finallyDo(() -> superstructure.setGoalState(SuperstructureState.kDefault));
      }, Set.of(swerve, superstructure)
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getTestCommand() {
    return Commands.sequence(
      superstructure.setGoalStateCommand(SuperstructureState.kDefault),
      new TestSwerve(swerve),
      superstructure.setGoalStateCommand(SuperstructureState.kCoralStation),
      superstructure.setGoalStateCommand(SuperstructureState.kL1),
      superstructure.setGoalStateCommand(SuperstructureState.kL2),
      superstructure.setGoalStateCommand(SuperstructureState.kL3),
      superstructure.setGoalStateCommand(SuperstructureState.kL2),
      new WaitUntilCommand(driverXbox.rightTrigger()),
      superstructure.toEjectStateCommand(),
      superstructure.setGoalStateCommand(SuperstructureState.kDefault)
    );
  }

  public void simulationPeriodic() {
  }
}
