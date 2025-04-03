package frc.robot.subsystems.elevator;



import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants.ElevatorConstants;

import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax leadMotor;
  private final SparkMax followerMotor;
  private final RelativeEncoder encoder;
  // private final RelativeEncoder absEncoder;
  // private final AbsoluteEncoder absEncoder;

  // private final RelativeEncoder kPrimaryEncoder;
  // private final RelativeEncoder kAlternateOrExternalEncoder;
  // private final AbsoluteEncoder kAbsoluteEncoder;
  
  private final SparkLimitSwitch limitSwitch;


  // Constructor
  public ElevatorIOSparkMax() {
    // Initialize the CANSparkMax motors for main and follower
    leadMotor = new SparkMax(ElevatorConstants.ELEVATOR_LEAD_SM, MotorType.kBrushless);
    followerMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_SM, MotorType.kBrushless);
  
    encoder = leadMotor.getEncoder();

    limitSwitch = leadMotor.getReverseLimitSwitch();

    // kPrimaryEncoder = leadMotor.getEncoder();
    // kAlternateOrExternalEncoder = leadMotor.getEncoder();
    // kAbsoluteEncoder = leadMotor.getAbsoluteEncoder();
    
    // AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();

    // initialize feed forward
    ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.ELEVATOR_KS, 
                                                              ElevatorConstants.ELEVATOR_KG, 
                                                              ElevatorConstants.ELEVATOR_KV, 
                                                              ElevatorConstants.ELEVATOR_KA);

    // Initialize the encoder for main
    // encoder = leadMotor.getEncoder();

    //get PID Controller


    SparkMaxConfig leadConfig = new SparkMaxConfig();
    leadConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.ELEV_AMP)
        .inverted(true);
    leadConfig
        .closedLoop
            .pidf(ElevatorConstants.ELEVATOR_KP,
                  ElevatorConstants.ELEVATOR_KI,
                  ElevatorConstants.ELEVATOR_KD, 
                  feedforward.calculate(ElevatorConstants.ELEVATOR_KV,
                                        ElevatorConstants.ELEVATOR_KA));
    // leadConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .follow(leadMotor, true)
        .smartCurrentLimit(ElevatorConstants.ELEV_AMP)
        .idleMode(IdleMode.kBrake);
    followerConfig
        .closedLoop
            .pidf(ElevatorConstants.ELEVATOR_KP,
                  ElevatorConstants.ELEVATOR_KI,
                  ElevatorConstants.ELEVATOR_KD, 
                  feedforward.calculate(ElevatorConstants.ELEVATOR_KV,
                                        ElevatorConstants.ELEVATOR_KA));


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
    // absEncoder.reset();
    encoder.setPosition(0);
    
  }

  @Override
  public void resetPositionTo(double encoderValue)
  {
    // encoder.setPosition(encoderValue);
  }

 


@Override 
public void setPosition(double position) {
  
  
  while (encoder.getPosition() < position)
  {
    leadMotor.set(ElevatorConstants.AUTO_ELEVATOR_SPEED);
  }
  leadMotor.stopMotor();

}

  /*
  @Override
  public void setPosition(double position) {
    leadMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }
  */

  @Override
  public void stop() {
    leadMotor.setVoltage(0);
  }

  public SparkLimitSwitch getLimitSwitch()
  {
    return limitSwitch;
  }
}