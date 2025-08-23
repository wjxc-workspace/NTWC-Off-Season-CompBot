package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class TestSwerve extends Command {
  private final Swerve swerve;

  private final Timer timer = new Timer();

  public TestSwerve(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    if (timer.get() < 1.5) {
      swerve.drive(new Translation2d(1, 0), 0, false);
    } else if (timer.get() < 3) {
      swerve.drive(new Translation2d(-1, 0), 0, false);
    }  else if (timer.get() < 4.5) {
      swerve.drive(new Translation2d(0, -1), 0, false);
    } else if (timer.get() < 6) {
      swerve.drive(new Translation2d(0, 1), 0, false);
    } else if (timer.get() < 7.5) {
      swerve.drive(new Translation2d(0, 0), 1, false);
    } else {
      swerve.drive(new Translation2d(0, 0), -1, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(9);
  }
  
}
