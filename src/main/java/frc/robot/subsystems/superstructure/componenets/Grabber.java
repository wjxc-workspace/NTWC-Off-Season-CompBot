package frc.robot.subsystems.superstructure.componenets;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.FSLib.statemachine.StateSubsystemComponent;
import frc.FSLib.util.SparkUtil;
import frc.FSLib.util.TalonUntil;
import frc.FSLib.util.Util;
import frc.robot.Robot;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.superstructure.SuperstructureMech;

public class Grabber implements StateSubsystemComponent {
  // motors and sensors
  private final SparkMax leftElbowMotor;
  private final SparkMax rightElbowMotor;
  private final SparkMax leftIntakeMotor;
  private final SparkMax rightIntakeMotor;

  private final CANcoder encoder;

  private final DigitalInput limitSwitch;

  // motion controllers
  private final PIDController elbowPid;

  // state variables
  private boolean hasCoral = false;

  // Whether to trush cancoder or not, since the cnacoder sometimes disconnect during regoinal events
  // due to some wiring error.
  private boolean trustCancoder = true;
  // A counter to wait until the coral is fully intaked. The counter starts whenever the output current
  // is greater than 20 amps, and when it is larger than 25 (approx. 0.5s), it is condiered the coral
  // has successfully intaked.
  // Since the limit switch is working properly right now, we are not using this.
  private int counter = 0;

  // publishers
  private final DoublePublisher anglePublisher;
  private final BooleanPublisher atLimitPublisher;
  private final BooleanPublisher atGoalPublisher;
  private final BooleanPublisher hasCoralPublisher;

  private final BooleanPublisher trustCancoderPublisher;
  private final BooleanSubscriber trustCancoderSubscriber;

  // simulation
  private final SuperstructureMech mech;
  private SparkMaxSim simMotor;
  private SingleJointedArmSim simArm;

  public Grabber(SuperstructureMech mech) {
    leftElbowMotor = new SparkMax(GrabberConstants.kLeftElbowMotorId, MotorType.kBrushless);
    SparkMaxConfig leftElbowMotorConfig = new SparkMaxConfig();
    leftElbowMotorConfig.inverted(true);
    leftElbowMotorConfig.encoder.positionConversionFactor(2 * Math.PI / GrabberConstants.kGearRatio);
    SparkUtil.assertOk(() -> leftElbowMotor.configure(leftElbowMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters));

    rightElbowMotor = new SparkMax(GrabberConstants.kRightElbowMotorId, MotorType.kBrushless);
    SparkMaxConfig rightElbowMotorConfig = new SparkMaxConfig();
    rightElbowMotorConfig.follow(leftElbowMotor, true);
    SparkUtil.assertOk(() -> rightElbowMotor.configure(rightElbowMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters));

    leftIntakeMotor = new SparkMax(GrabberConstants.kLeftMotorId, MotorType.kBrushless);
    SparkMaxConfig leftIntakeMotorConfig = new SparkMaxConfig();
    leftIntakeMotorConfig.inverted(true);
    leftIntakeMotorConfig.smartCurrentLimit(GrabberConstants.kIntakeMotorCurrentLimit);
    SparkUtil.assertOk(() -> leftIntakeMotor.configure(leftIntakeMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters));

    rightIntakeMotor = new SparkMax(GrabberConstants.kRightMotorId, MotorType.kBrushless);
    SparkMaxConfig rightIntakeMotorConfig = new SparkMaxConfig();
    rightIntakeMotorConfig.follow(leftIntakeMotor, true);
    rightIntakeMotorConfig.smartCurrentLimit(GrabberConstants.kIntakeMotorCurrentLimit);
    SparkUtil.assertOk(() -> rightIntakeMotor.configure(rightIntakeMotorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters));

    encoder = new CANcoder(GrabberConstants.kEncoderId);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoderConfig.MagnetSensor.MagnetOffset = -0.209717;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    TalonUntil.assertOk(() -> encoder.getConfigurator().apply(encoderConfig));

    limitSwitch = new DigitalInput(GrabberConstants.kLimitSwitchPort);

    elbowPid = new PIDController(GrabberConstants.kP, GrabberConstants.kI, GrabberConstants.kD);
    elbowPid.enableContinuousInput(0, 2 * Math.PI);
    elbowPid.setTolerance(GrabberConstants.kElbowAngleTolerance);

    anglePublisher = NetworkTableInstance.getDefault()
        .getTable("Grabber").getDoubleTopic("Angle").publish();
    atLimitPublisher = NetworkTableInstance.getDefault()
        .getTable("Grabber").getBooleanTopic("AtLimit").publish();
    atGoalPublisher = NetworkTableInstance.getDefault()
        .getTable("Grabber").getBooleanTopic("AtGoal").publish();
    hasCoralPublisher = NetworkTableInstance.getDefault()
        .getTable("Grabber").getBooleanTopic("HasCoral").publish();
    trustCancoderPublisher = NetworkTableInstance.getDefault()
        .getTable("Grabber").getBooleanTopic("TrustCancoder").publish();
    trustCancoderSubscriber = NetworkTableInstance.getDefault()
        .getTable("Grabber").getBooleanTopic("TrustCancoder").subscribe(trustCancoder);
      
    trustCancoderPublisher.set(trustCancoder);

    this.mech = mech;
    if (Robot.isSimulation()) {
      simMotor = new SparkMaxSim(leftElbowMotor, DCMotor.getNEO(1));
      simArm = new SingleJointedArmSim(
          DCMotor.getNEO(2),
          GrabberConstants.kGearRatio,
          0.08,
          GrabberConstants.kLength,
          GrabberConstants.kMinAngle,
          GrabberConstants.kMaxAngle,
          true,
          2 * Math.PI);
      simMotor.setPosition(2 * Math.PI);
    }

    if (encoder.isConnected()) {
      leftElbowMotor.getEncoder().setPosition(encoder.getAbsolutePosition().getValue().in(Radians));
    } else {
      leftElbowMotor.getEncoder().setPosition(GrabberConstants.kCoralStationAngle);
    }
  }

  private void setAngleMotorVoltage(double voltage) {
    leftElbowMotor.setVoltage(voltage);
  }

  public void stopAngleMotor() {
    leftElbowMotor.setVoltage(0);
  }

  private void safeSetAngleVoltage(double voltage) {
    if (atLimit()) {
      stopAngleMotor();
    } else {
      setAngleMotorVoltage(voltage);
    }
  }

  public void setAngle(double targetRadians) {
    double pidOut = elbowPid.calculate(getElbowRadians(), targetRadians);
    safeSetAngleVoltage(pidOut);
  }

  // elbow encoder getters
  public double getElbowRadians() {
    if (trustCancoder) {
      return encoder.getAbsolutePosition().getValue().in(Radians);
    } else {
      return leftElbowMotor.getEncoder().getPosition();
    }
  }

  // intake motor setters
  public void setIntakeMotor(double speed) {
    leftIntakeMotor.set(speed);
    if (Math.abs(leftIntakeMotor.getAppliedOutput()) > 0.1) {
      if (Math.abs(leftIntakeMotor.getOutputCurrent()) < 15) {
        counter++;
        if (counter > 45) {
          counter = 0;
          hasCoral = false;
        }
      } else {
        // counter++;
        // if (counter > GrabberConstants.kIntakeStallCurrent) {
          
        //   hasCoral = true;
        //   counter = 0;
        // }
        if (limitSwitch.get()) {
          hasCoral = true;
        }
      }
    }
  }

  public void stopIntakeMotor() {
    leftIntakeMotor.set(0);
  }

  // state functions
  public boolean atAngleGoal() {
    return elbowPid.atSetpoint();
  }

  public boolean atLimit() {
    return !Util.isWithin(
        getElbowRadians(),
        GrabberConstants.kMinAngle,
        GrabberConstants.kMaxAngle);
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  public void toggleHasCoral() {
    hasCoral = !hasCoral;
  }

  @Override
  public void periodic() {
    anglePublisher.set(getElbowRadians());
    atLimitPublisher.set(atLimit());
    atGoalPublisher.set(atAngleGoal());
    hasCoralPublisher.set(hasCoral());
    mech.setAngle(getElbowRadians());
    trustCancoder = trustCancoderSubscriber.get() && encoder.isConnected();
    trustCancoderPublisher.set(trustCancoder);
  }

  @Override
  public void simulationPeriodic() {
    simArm.setInput(simMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
    simArm.update(0.02);
    simMotor.iterate(
        Units.radiansPerSecondToRotationsPerMinute(simArm.getVelocityRadPerSec() * GrabberConstants.kGearRatio),
        RoboRioSim.getVInVoltage(), 0.02);
  }
}
