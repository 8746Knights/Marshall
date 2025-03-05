
import com.revrobotics.spark.SparkBase.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.CANConstants;

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
        .restoreFactoryDefaults()
        .invert(false)
        .setSmartCurrentLimit(ClimberConstants.CLIMBER_AMP)
        .idleMode(IdleMode.kBrake)
        .setCANTimeout(ClimberConstants.CAN_TIMEOUT)
    motorConfig
        .closedLoop
            .pidf(IntakeConstants.CORAL_KP, IntakeConstants.CORAL_KI, IntakeConstants.CORAL_KD, IntakeConstants.CORAL_KFF);


    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void setMotorVoltage(double volts) {
    motor1.setVoltage(volts);
  }

  public void stopMotor() {
    motor1.stopMotor();
  }
}