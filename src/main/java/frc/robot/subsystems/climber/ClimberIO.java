package frc.robot.subsystems.climber;

public interface ClimberIO {

  public static class ClimberIOInputs {
    public double motorCurrent = 0;
    public double motorVoltage = 0;
    public double motorAngle = 0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setMotorVoltage(double volts) {}

  public default void stopMotor() {}

  public default void set(double speed) {}
}