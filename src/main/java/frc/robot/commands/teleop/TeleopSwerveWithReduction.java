package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

public class TeleopSwerveWithReduction extends Command {
  private final Swerve swerve;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(5.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(5.0);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(8.0);

  private double xSpeed = 0.0;
  private double ySpeed = 0.0;
  private double rotSpeed = 0.0;

  private int reductionCounter = 0;

  private XboxController controller;

  public TeleopSwerveWithReduction(Swerve swerve, XboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    if (controller.getStartButtonPressed()) {
      swerve.resetGyro(0);
    }
    
    if (controller.getRightBumperButton()) reductionCounter = 10;
    double reduction = reductionCounter > 0 ? 0.4 : 1;

    xSpeed = xLimiter.calculate(-controller.getLeftY() * reduction);
    ySpeed = yLimiter.calculate(-controller.getLeftX() * reduction);
    rotSpeed = rotLimiter.calculate(-controller.getRightX() * reduction);
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.04);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.04);
    rotSpeed = MathUtil.applyDeadband(rotSpeed, 0.04);

    // square the input to inprove driving experience
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);

    swerve.drive(
        new Translation2d(xSpeed, ySpeed).times(SwerveConstants.kModuleMaxSpeed),
        rotSpeed * SwerveConstants.kRotationalSpeed,
        true);

    reductionCounter = reductionCounter > 0 ? reductionCounter - 1 : 0;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false);
  }
}
