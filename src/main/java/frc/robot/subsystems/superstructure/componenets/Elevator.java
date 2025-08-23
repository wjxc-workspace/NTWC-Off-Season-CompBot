package frc.robot.subsystems.superstructure.componenets;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.FSLib.statemachine.StateSubsystemComponent;
import frc.FSLib.util.SparkUtil;
import frc.FSLib.util.Util;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.SuperstructureMech;

public class Elevator implements StateSubsystemComponent {
  // motors
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  // motion contoller
  private final PIDController pid;

  // publisher
  private final DoublePublisher heightPublisher;
  private final BooleanPublisher atGoalPublisher;
  private final BooleanPublisher atLimitPublisher;

  // simulation
  private final SuperstructureMech mech;
  private SparkMaxSim simMotor;
  private ElevatorSim simElevator;

  public Elevator(SuperstructureMech mech) {
    leftMotor = new SparkMax(ElevatorConstants.kLeftMotorId, MotorType.kBrushless);
    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.inverted(true);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.smartCurrentLimit(40, 10);
    leftMotorConfig.encoder.positionConversionFactor(ElevatorConstants.kDrumPerimeter / ElevatorConstants.kGearRatio);
    SparkUtil.assertOk(
        () -> leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    leftMotor.getEncoder().setPosition(0);

    rightMotor = new SparkMax(ElevatorConstants.kRightMotorId, MotorType.kBrushless);
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.smartCurrentLimit(40, 10);
    rightMotorConfig.follow(leftMotor, false);
    SparkUtil.assertOk(
        () -> rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    pid.setTolerance(ElevatorConstants.kPIDTolerance);

    heightPublisher = NetworkTableInstance.getDefault()
        .getTable("Elevator").getDoubleTopic("Height").publish();
    atGoalPublisher = NetworkTableInstance.getDefault()
        .getTable("Elevator").getBooleanTopic("AtGoal").publish();
    atLimitPublisher = NetworkTableInstance.getDefault()
        .getTable("Elevator").getBooleanTopic("AtLimit").publish();

    this.mech = mech;
    if (Robot.isSimulation()) {
      simMotor = new SparkMaxSim(leftMotor, DCMotor.getNEO(1));
      simElevator = new ElevatorSim(
          DCMotor.getNEO(2),
          ElevatorConstants.kGearRatio,
          ElevatorConstants.kCarrageMass,
          ElevatorConstants.kDrumRadius,
          0,
          ElevatorConstants.kMaxHeight,
          true,
          0);
    }
  }

  private void setMotorVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
  }

  private void safeSetMotorVoltage(double voltage) {
    if (atLimit()) {
      stopElevatorMotor();
    } else {
      setMotorVoltage(voltage);
    }
  }

  public void setHeight(double height) {
    double pidOut = pid.calculate(getHeight(), height);
    safeSetMotorVoltage(pidOut + 0.4);
  }

  public void resetEncoder() {
    leftMotor.getEncoder().setPosition(0);
  }

  public void stopElevatorMotor() {
    setMotorVoltage(0);
  }

  // getters
  public double getHeight() {
    return leftMotor.getEncoder().getPosition();
  }

  // state functions
  public boolean atHeightGoal() {
    return pid.atSetpoint();
  }

  public boolean atLimit() {
    return !Util.isWithin(getHeight(), ElevatorConstants.kMinHeight, ElevatorConstants.kMaxHeight);
  }

  @Override
  public void periodic() {
    heightPublisher.set(getHeight());
    atGoalPublisher.set(atHeightGoal());
    atLimitPublisher.set(atLimit());
    mech.setHeight(getHeight());
  }

  @Override
  public void simulationPeriodic() {
    simElevator.setInput(simMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
    simElevator.update(0.02);
    simMotor.iterate(
        simElevator.getVelocityMetersPerSecond() / ElevatorConstants.kDrumPerimeter * 60,
        RoboRioSim.getVInVoltage(), 0.02);
  }
}
