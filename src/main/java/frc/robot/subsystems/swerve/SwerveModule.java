package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.FSLib.swerve.SwerveModuleConstants;
import frc.FSLib.util.SparkUtil;
import frc.FSLib.util.TalonUntil;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final SparkMax steerMotor;

  private final CANcoder cancoder;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController drivePid;
  private final PIDController steerPid;

  private Rotation2d lastSteerAngle;

  public SwerveModule(SwerveModuleConstants moduleConstants) {
    driveMotor = new TalonFX(moduleConstants.driveMotorId(), RobotConstants.kAlternateBusName);
    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    TalonUntil.assertOk(() -> driveMotor.getConfigurator().apply(driveMotorConfig));

    steerMotor = new SparkMax(moduleConstants.steerMotorId(), MotorType.kBrushless);
    SparkMaxConfig steerMotorConfig = new SparkMaxConfig();
    steerMotorConfig.inverted(true);
    steerMotorConfig.idleMode(IdleMode.kBrake);
    steerMotorConfig.smartCurrentLimit(40);
    SparkUtil.assertOk(() -> steerMotor.configure(steerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    cancoder = new CANcoder(moduleConstants.cancoderId(), RobotConstants.kAlternateBusName);
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.cancoderOffset();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    TalonUntil.assertOk(() -> cancoder.getConfigurator().apply(cancoderConfig));

    driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.kDriveS, SwerveConstants.kDrivekV);
    drivePid = new PIDController(SwerveConstants.kDriveP, SwerveConstants.kDriveI, SwerveConstants.kDriveD);

    steerPid = new PIDController(SwerveConstants.kSteerP, SwerveConstants.kSteerI, SwerveConstants.kSteerD);
    steerPid.enableContinuousInput(-0.5, 0.5);

    lastSteerAngle = new Rotation2d();
  }

  private Rotation2d getSteerAngle() {
    return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());
  }

  private double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble() * SwerveConstants.kWheelPerimeter;
  }

  private double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.kWheelPerimeter;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        getDrivePosition(), getSteerAngle());
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        getDriveVelocity(), getSteerAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean optimizeLowSpeedAngle) { 
    if (optimizeLowSpeedAngle) {
      desiredState.angle = Math.abs(desiredState.speedMetersPerSecond) > 0.03 ? desiredState.angle : lastSteerAngle;
    }
    desiredState.optimize(getSteerAngle());
    steerMotor.set(steerPid.calculate(getSteerAngle().getRotations(), desiredState.angle.getRotations()));
    lastSteerAngle = desiredState.angle;
    driveMotor.setVoltage(
      driveFeedforward.calculate(desiredState.speedMetersPerSecond)
      + drivePid.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond)
    );
  }
}
