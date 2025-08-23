package frc.robot.subsystems.intake.components;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.FSLib.statemachine.StateSubsystemComponent;
import frc.FSLib.util.Util;
import frc.robot.Robot;
import frc.robot.Constants.IntakerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.intake.IntakeMech;

public class Intaker implements StateSubsystemComponent {
  // motors
  private final TalonFX elbowMotor;
  private final TalonFX intakeMotor;

  private final CANcoder elbowEncoder;

  // motion controllers
  private final PIDController elbowPid;

  // state variables
  private boolean hasAlgae;
  private int intakeCounter = 0;

  // publisher
  private final DoublePublisher elbowAnglePublisher;
  private final BooleanPublisher atGoalPublisher;
  private final BooleanPublisher atLimitPublisher;
  private final BooleanPublisher hasAlgaePublisher;

  private final IntakeMech mech;
  private SingleJointedArmSim simArm;

  public Intaker(IntakeMech mech) {
    elbowMotor = new TalonFX(IntakerConstants.kIntakeAngleMasterId, RobotConstants.kAlternateBusName);
    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    angleMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elbowMotor.getConfigurator().apply(angleMotorConfig);

    intakeMotor = new TalonFX(IntakerConstants.kIntakeMotorId, RobotConstants.kAlternateBusName);
    TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    elbowEncoder = new CANcoder(IntakerConstants.kEncoderId, RobotConstants.kAlternateBusName);
    CANcoderConfiguration angleEncoderConfig = new CANcoderConfiguration();
    angleEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    angleEncoderConfig.MagnetSensor.MagnetOffset = IntakerConstants.kMagnetOffset;
    angleEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    elbowEncoder.getConfigurator().apply(angleEncoderConfig);

    elbowPid = new PIDController(IntakerConstants.kP, IntakerConstants.kI, IntakerConstants.kD);
    elbowPid.setTolerance(IntakerConstants.kPIDTolerance);

    hasAlgae = false;

    elbowAnglePublisher = NetworkTableInstance.getDefault()
        .getTable("Intaker").getDoubleTopic("Angle").publish();
    atGoalPublisher = NetworkTableInstance.getDefault()
        .getTable("Intaker").getBooleanTopic("AtGoal").publish();
    atLimitPublisher = NetworkTableInstance.getDefault()
        .getTable("Intaker").getBooleanTopic("AtLimit").publish();
    hasAlgaePublisher = NetworkTableInstance.getDefault()
        .getTable("Intaker").getBooleanTopic("HasAlgae").publish();

    this.mech = mech;
    if (Robot.isSimulation()) {
      simArm = new SingleJointedArmSim(
        DCMotor.getFalcon500Foc(1),
        IntakerConstants.kGearRatio,
        0.5,
        IntakerConstants.kLength,
        IntakerConstants.kMinAngle,
        IntakerConstants.kMaxAngle,
        true,
        IntakerConstants.kMaxAngle);
    }

    SmartDashboard.putData(elbowPid);
  }

  // elbow motor setters
  private void setElbowVoltage(double voltage) {
    elbowMotor.setVoltage(voltage);
  }

  public void stopAngleMotor() {
    setElbowVoltage(0);
  }

  public void safeSetElbowVoltage(double voltage) {
    if (atLimit()) {
      stopAngleMotor();
    } else {
      setElbowVoltage(voltage);
    }
  }

  public void setElbowAngle(double angle) {
    double pidOut = elbowPid.calculate(getAngle(), angle);
    safeSetElbowVoltage(pidOut);
  }

  // elbow sensor getters
  public double getAngle() {
    return elbowEncoder.getAbsolutePosition().getValue().in(Radians);
  }

  // intake motor setters
  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
    if (Math.abs(intakeMotor.get()) > 0.1) {
      if (intakeMotor.getTorqueCurrent().getValueAsDouble() > IntakerConstants.kStallCurrent) {
        intakeCounter++;
        if (intakeCounter > 25) {
          hasAlgae = true;
          intakeCounter = 0;
        }
      } else {
        hasAlgae = false;
        intakeCounter = 0;
      }
    }
  }

  public void stopIntakeMotor() {
    setIntakeMotorSpeed(0);
  }

  // state functions
  public boolean atAngleGoal() {
    return elbowPid.atSetpoint();
  }

  public boolean atLimit() {
    return !Util.isWithin(getAngle(), IntakerConstants.kMinAngle, IntakerConstants.kMaxAngle);
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }

  @Override
  public void periodic() {
    elbowAnglePublisher.set(getAngle());
    atGoalPublisher.set(atAngleGoal());
    atLimitPublisher.set(atLimit());
    hasAlgaePublisher.set(hasAlgae());
  };

  @Override
  public void simulationPeriodic() {
    TalonFXSimState simMotor = intakeMotor.getSimState();
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    Voltage motorVoltage = simMotor.getMotorVoltageMeasure();
    simArm.setInput(motorVoltage.in(Volts));
    simArm.update(0.02);
    simMotor.setRawRotorPosition(Units.radiansToRotations(simArm.getAngleRads()) * IntakerConstants.kGearRatio);
    simMotor.setRotorVelocity(Units.radiansPerSecondToRotationsPerMinute(simArm.getVelocityRadPerSec()) / 60 * IntakerConstants.kGearRatio);
    CANcoderSimState simCancoder = elbowEncoder.getSimState();
    simCancoder.setSupplyVoltage(RobotController.getBatteryVoltage());
    simCancoder.setRawPosition(Units.radiansToRotations(simArm.getAngleRads()));
    mech.setAngle(getAngle());
  }
}
