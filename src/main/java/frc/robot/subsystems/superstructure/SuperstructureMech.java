package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SuperstructureMech {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d elevator;
  private final MechanismLigament2d grabber;

  private double stage2Height;

  private final StructPublisher<Pose3d> stage1Publisher = NetworkTableInstance.getDefault()
    .getTable("SuperstructureMech").getStructTopic("Stage1", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> stage2Publisher = NetworkTableInstance.getDefault()
    .getTable("SuperstructureMech").getStructTopic("Stage2", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> grabberPublisher = NetworkTableInstance.getDefault()
    .getTable("SuperstructureMech").getStructTopic("Grabber", Pose3d.struct).publish();

  public SuperstructureMech() {
    mech = new Mechanism2d(0.2, 0.2);
    root = mech.getRoot("superstructure", 0.1, 0.1);
    elevator = root.append(new MechanismLigament2d("elevator", 0.05, 90, 8, new Color8Bit(Color.kOrange)));
    grabber = root.append(new MechanismLigament2d("grabber", 0.03, 0, 8, new Color8Bit(Color.kRed)));
    SmartDashboard.putData("SuperstructureMech", mech);
  }

  public void setHeight(double height) {
    elevator.setLength(height);
    double stage1Height = height > 0.8 ? 0.8 : height;
    stage2Height = height;
    stage1Publisher.set(new Pose3d(0, 0, stage1Height, new Rotation3d()));
    stage2Publisher.set(new Pose3d(0, 0, stage2Height, new Rotation3d()));
  }

  public void setAngle(double angle) {
    grabber.setAngle(angle);
    grabberPublisher.set(new Pose3d(0.14, 0, stage2Height + 0.81, new Rotation3d(0, Units.degreesToRadians(-angle), 0)));
  }
}
