package frc.robot.subsystems.climber;

    
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final ClimberIO io;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  public void stopMotor() {
    io.stopMotor();
  }

}
