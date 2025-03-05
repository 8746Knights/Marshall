package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberConstants;

/** Add your docs here. */
public class ClimberIOSparkMax implements ClimberIO {


  private SparkMax motor = null;

  public ClimberIOSparkMax() {
    motor = new SparkMax(CANConstants.CLIMBER_SM, MotorType.kBrushless);

    /* 
    motor1.restoreFactoryDefaults();
    motor1.setCANTimeout(250);
    motor1.setSmartCurrentLimit(40);
    motor1.enableVoltageCompensation(12);
    motor1.setIdleMode(IdleMode.kBrake);

    motorRelativeEncoder = motor1.getEncoder();
    motorRelativeEncoder.setPositionConversionFactor(1. / MOTOR_GEAR_RATIO);
    */

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false)
        .smartCurrentLimit(ClimberConstants.CLIMBER_AMP)
        .idleMode(IdleMode.kBrake);


    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void stopMotor() {
    motor.stopMotor();
  }
}