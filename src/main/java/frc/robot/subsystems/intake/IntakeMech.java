package frc.robot.subsystems.intake;

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

public class IntakeMech {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d intaker;

  private final StructPublisher<Pose3d> intakerPublisher = NetworkTableInstance.getDefault()
    .getTable("IntakeMech").getStructTopic("Intaker", Pose3d.struct).publish();

  public IntakeMech() {
    mech = new Mechanism2d(0.2, 0.2);
    root = mech.getRoot("intake", 0.1, 0.1);
    intaker = root.append(new MechanismLigament2d("intaker", 0.03, 0, 8, new Color8Bit(Color.kBlue)));
    SmartDashboard.putData("IntakeMech", mech);
  }

  public void setAngle(double angle) {
    intaker.setAngle(angle);
    intakerPublisher.set(new Pose3d(-0.15, 0, 0.195, new Rotation3d(0, Units.degreesToRadians(angle), 0)));
  }
}
