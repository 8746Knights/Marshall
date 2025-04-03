
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;



import frc.robot.Constants.*;

public class IntakeIOSparkMax implements IntakeIO {
  // SparkMax algaeMotor1;
  // SparkMax algaeMotor2;
  SparkMax coralIntake;
  SparkMax coralWrist;
  // RelativeEncoder wristEncoder;
  ArmFeedforward feedforward;
  SparkMax algaeMotor;

  AbsoluteEncoder absWristEncoder;

  public IntakeIOSparkMax() {
    // find actual motor IDs
   // algaeMotor1 = new SparkMax(CANConstants.ALGAE_ONE_SM, MotorType.kBrushless);
   // algaeMotor2 = new SparkMax(CANConstants.ALGAE_TWO_SM, MotorType.kBrushless);
    coralIntake = new SparkMax(CANConstants.CORALINTAKE_SM, MotorType.kBrushless);
    coralWrist = new SparkMax(CANConstants.WRIST_SM, MotorType.kBrushless);
    algaeMotor = new SparkMax(CANConstants.ALGAE_SM, MotorType.kBrushless);

    // ask about gear ratios for all motors
    // wristEncoder = coralWrist.getEncoder();

    absWristEncoder = coralWrist.getAbsoluteEncoder();

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
    
    coralWristConfig.closedLoop.maxMotion
                .maxVelocity(0.05)
                .maxAcceleration(0.01)
                .allowedClosedLoopError(0.001);                
    
    coralWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    coralWrist.configure(coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    
    
    SparkMaxConfig algaeConfig = new SparkMaxConfig();
    algaeConfig
        .smartCurrentLimit(IntakeConstants.ALGAE_AMP)
        .idleMode(IdleMode.kBrake);

    // algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // SparkMaxConfig algae2Config = new SparkMaxConfig();
    // algae2Config
    //     .smartCurrentLimit(IntakeConstants.ALGAE_AMP)
    //     .idleMode(IdleMode.kBrake);

    // algaeMotor2.configure(algae2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
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
    algaeMotor.setVoltage(voltage);
  
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
    // return wristEncoder.getPosition();
    return absWristEncoder.getPosition();
  }

  @Override 
  public void setWristPosition(double position) {

    coralWrist.getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl);


  }


  @Override
  public void setWristVoltage(double voltage) {
    System.out.println("Wrist position: " + getWristPosition());
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
    algaeMotor.set(speed);
  }


//   @Override
//   public void resetPosition()
//   {
//     absWristEncoder.setPosition(0);
//   }

}