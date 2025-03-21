package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  IntakeIO io;
  //IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setAlgaeVoltage(double voltage) {
    io.setAlgaeVoltage(voltage);
  }

  public void setCoralIntakeVoltage(double voltage) {
    io.setCoralIntakeVoltage(voltage);
  }

  private double targetPosition = 0.0;
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577, 0.0);

  public void setWristPositionDegrees(double position) {
    double targetPosition = Math.toRadians(position);
  }

  public double getTargetWristPosition() {
    return targetPosition;
  }

  @Override
  public void periodic() {
  //  io.updateInputs(inputs);
  }

  public double getCoralWristIntakeCurrent() {
 //   return inputs.coralWristCurrent;
    return targetPosition;
  }

  public double getWristPosition() {
    return io.getWristPosition();
  }

  public void wristAngle(double position) {
    io.wristAngle(position);
  }

  public void setWristVoltage(double voltage) {
    System.out.println(getWristPosition());
    io.setWristVoltage(voltage);
  }

  public void resetAngle(double radians) {
  }

  public void setWristSpeed(double speed) {
    io.setWristSpeed(speed);
  }

  public void setIntakeSpeed(double speed) {
    io.setIntakeSpeed(speed);
  }

  public void setAlgaeSpeed(double speed) {
    io.setAlgaeSpeed(speed);
  }

  public void resetPosition()
  {
    io.resetPosition();
  }

  public void setWristPosition(double position)
  {
    io.setWristPosition(position);
  }
}