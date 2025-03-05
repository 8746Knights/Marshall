package frc.robot.subsystems.elevator;



import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax leadMotor;
  private final SparkMax followerMotor;
  private final RelativeEncoder encoder;

  // Constructor
  public ElevatorIOSparkMax() {
    // Initialize the CANSparkMax motors for main and follower
    leadMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEAD_SM, MotorType.kBrushless);
    followerMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_SM, MotorType.kBrushless);

    
    // Initialize the encoder for main
    encoder = leadMotor.getEncoder();

    //get PID Controller

    SparkMaxConfig leadConfig = new SparkMaxConfig();
    leadConfig
        .idleMode(IdleMode.kBrake);
    leadConfig
        .closedLoop
            .pidf(ElevatorConstants.ELEVATOR_KP,ElevatorConstants.ELEVATOR_KI,ElevatorConstants.ELEVATOR_KD, ElevatorConstants.ELEVATOR_KFF);

    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .follow(leadMotor,true)
        .idleMode(IdleMode.kBrake);
    followerConfig
        .closedLoop
            .pidf(ElevatorConstants.ELEVATOR_KP,ElevatorConstants.ELEVATOR_KI,ElevatorConstants.ELEVATOR_KD, ElevatorConstants.ELEVATOR_KFF);

    followerMotor.configure(followerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  @Override
  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  @Override
  public double getPosition() {
    // Get the position from the encoder
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    // Get the velocity from the encoder
    return encoder.getVelocity();
  }

  @Override
  public void resetPosition() {
    // Reset the encoder to the specified position
    encoder.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    leadMotor.getClosedLoopController().setReference(position,ControlType.kPosition);
  }

  @Override
  public void stop() {
    leadMotor.setVoltage(0);
  }
}