package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Swerve extends Subsystem {
  public void drive(Translation2d translation, double rotation, boolean fieldRelative);
  public Pose2d getOdometryPosition();
  default void addVisionMeasurement(Pose2d pos, double dt) {};
  default void resetGyro(double deg) {};
}
