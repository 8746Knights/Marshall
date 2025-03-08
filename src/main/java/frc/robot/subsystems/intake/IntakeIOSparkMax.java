
package frc.robot.subsystems.intake;


import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

import frc.robot.Constants.*;

public class IntakeIOSparkMax implements IntakeIO {
  SparkMax algaeMotor1;
  SparkMax algaeMotor2;
  SparkMax coralIntake;
  SparkMax coralWrist;
  RelativeEncoder wristEncoder;
  ArmFeedforward feedforward;

  public IntakeIOSparkMax() {
    // find actual motor IDs
    algaeMotor1 = new SparkMax(CANConstants.ALGAE_ONE_SM, MotorType.kBrushless);
    algaeMotor2 = new SparkMax(CANConstants.ALGAE_TWO_SM, MotorType.kBrushless);
    coralIntake = new SparkMax(CANConstants.CORALINTAKE_SM, MotorType.kBrushless);
    coralWrist = new SparkMax(CANConstants.WRIST_SM, MotorType.kBrushless);

    // ask about gear ratios for all motors
    wristEncoder = coralWrist.getEncoder();

    // wrist feed forward (ArmFeedforward = angular)
    feedforward = new ArmFeedforward(IntakeConstants.WRIST_KS, 
                                     IntakeConstants.WRIST_KG, 
                                     IntakeConstants.WRIST_KV, 
                                     IntakeConstants.WRIST_KA);

    SparkMaxConfig coralWristConfig = new SparkMaxConfig();
    coralWristConfig
        .inverted(true)
        .smartCurrentLimit(IntakeConstants.WRIST_AMP)
        .idleMode(IdleMode.kBrake);
    coralWristConfig
        .closedLoop
            .pidf(IntakeConstants.CORAL_KP, 
                  IntakeConstants.CORAL_KI, 
                  IntakeConstants.CORAL_KD, 
                  feedforward.calculate(getWristPosition(), 
                                        IntakeConstants.WRIST_KV, 
                                        IntakeConstants.WRIST_KA));

    coralWrist.configure(coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig algae1Config = new SparkMaxConfig();
    algae1Config
        .smartCurrentLimit(IntakeConstants.ALGAE_AMP)
        .idleMode(IdleMode.kBrake);

    algaeMotor1.configure(algae1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig algae2Config = new SparkMaxConfig();
    algae2Config
        .smartCurrentLimit(IntakeConstants.ALGAE_AMP)
        .idleMode(IdleMode.kBrake);

    algaeMotor2.configure(algae2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig coralIntakeConfig = new SparkMaxConfig();
    coralIntakeConfig
        .smartCurrentLimit(IntakeConstants.CORAL_AMP)
        .idleMode(IdleMode.kBrake);

    coralIntake.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    System.out.println("Wrist position: " + getWristPosition());
    coralWrist.getClosedLoopController().setReference(position, ControlType.kPosition);
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

  @Override
  public void setWristSpeed(double speed) {
    coralWrist.set(speed);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    coralIntake.set(speed);
  }

  @Override
  public void setAlgaeSpeed(double speed)
  {
    algaeMotor1.set(speed);
    algaeMotor2.set(speed);
  }

  @Override
  public void resetPosition()
  {
    wristEncoder.setPosition(0);
  }
}