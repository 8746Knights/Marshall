package frc.robot.subsystems.elevator;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;;

public class ElevatorSubsystem extends SubsystemBase {


    //setup variables for accessing motors
    private final DCMotor elevatorGearbox = DCMotor.getNEO(0);
    private final SparkMax elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_SPARKMAX,SparkLowLevel.MotorType.kBrushless);
    //private final SparkMaxSim elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorGearbox);

    private final ProfiledPIDController elevatorController = new ProfiledPIDController(ElevatorConstants.ELEVATOR_KP, 
                                                                                ElevatorConstants.ELEVATOR_KI, 
                                                                                ElevatorConstants.ELEVATOR_KD, 
                                                                                new Constraints(ElevatorConstants.MAX_VELOCITY,
                                                                                    ElevatorConstants.MAX_ACCELERATION));

    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.ELEVATOR_KS,
                                                                                ElevatorConstants.ELEVATOR_KG,
                                                                                ElevatorConstants.ELEVATOR_KV,
                                                                                ElevatorConstants.ELEVATOR_KA);

    private final ElevatorSim elevatorSim = null;
    


}
