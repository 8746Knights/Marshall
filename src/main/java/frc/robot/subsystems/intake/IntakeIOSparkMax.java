
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSparkMax implements IntakeIO {
  SparkMax algaeMotor1;
  SparkMax algaeMotor2;
  SparkMax coralIntake;
  SparkMax coralWrist;
  RelativeEncoder wristEncoder;

  public IntakeIOSparkMax() {
    // find actual motor IDs
    algaeMotor1 = new SparkMax(CANConstants.ALGAE_ONE_SM, MotorType.kBrushless);
    algaeMotor2 = new SparkMax(CANConstants.ALGAE_TWO_SM, MotorType.kBrushless);
    coralIntake = new SparkMax(CANConstants.CORALINTAKE_SM, MotorType.kBrushless);
    coralWrist = new SparkMax(CANConstants.WRIST_SM, MotorType.kBrushless);

    // ask about gear ratios for all motors
    wristEncoder = coralWrist.getEncoder();

    SparkMaxConfig coralWristConfig = new SparkMaxConfig();
    coralWristConfig
        .invert(false)
        .setSmartCurrentLimit(IntakeConstants.WRIST_AMP)
        .idleMode(IdleMode.kBrake)
    coralWristConfig
        .closedLoop
            .pidf(IntakeConstants.CORAL_KP, IntakeConstants.CORAL_KI, IntakeConstants.CORAL_KD, IntakeConstants.CORAL_KFF);

    coralWrist.configure(coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig algae1Config = new SparkMaxConfig();
    algae1Config
        .restoreFactoryDefaults()
        .setSmartCurrentLimit(IntakeConstants.ALGAE_AMP)
        .idleMode(IdleMode.kBrake)

    algaeMotor1.configure(algae1Config, ResetMode.kResetSafeParameters, PersistMode,kPersistParameters);

    SparkMaxConfig algae2Config = new SparkMaxConfig();
    algae2Config
        .restoreFactoryDefaults()
        .setSmartCurrentLimit(IntakeConstants.ALGAE_AMP)
        .idleMode(IdleMode.kBrake)

    algaeMotor2.configure(algae2Config, ResetMode.kResetSafeParameters, PersistMode,kPersistParameters);

    SparkMaxConfig coralIntakeConfig = new SparkMaxConfig();
    coralIntakeConfig
        .setSmartCurrentLimit(IntakeConstants.CORAL_AMP)
        .idleMode(IdleMode.kBrake)

    coralIntake.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode,kPersistParameters);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.coralWristCurrent = coralWrist.getOutputCurrent();
    inputs.coralWristVelocity = coralWrist.getEncoder().getVelocity();
    inputs.coralWristPosition = coralWrist.getEncoder().getPosition();
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    algaeMotor1.setVoltage(voltage);
    algaeMotor2.setVoltage(-voltage);
  }

  @Override
  public void setCoralIntakeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }

  @Override
  public void adjustAngle(double angleRadians) {
    coralWrist.getEncoder().setPosition(coralWrist.getEncoder().getPosition() + angleRadians);
  }

  @Override
  public void wristAngle(double position) {
    // System.out.println("Wrist position: " + getWristPosition());
    coralWrist.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  @Override
  public void setWristVoltage(double voltage) {
    // System.out.println("Wrist position: " + getWristPosition());
    coralWrist.set(voltage);
  }
}