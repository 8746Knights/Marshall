// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(4);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class ElevatorConstants {
    public static final int ELEVATOR_LEAD_SM = CANConstants.ELEVATOR_LEAD_SM;
    public static final int ELEVATOR_FOLLOWER_SM = CANConstants.ELEVATOR_FOLLOWER_SM;

    /* Constants for the profiled PID Controller
     * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ProfiledPIDController.html
     * 
     * Kp - The proportional coefficient. Must be >= 0.
     * Ki - The integral coefficient. Must be >= 0.
     * Kd - The differential coefficient. Must be >= 0.
     * Kff - The feed forward coefficient
      */
    public static final double ELEVATOR_KP = .027;
    public static final double ELEVATOR_KI = 0;
    public static final double ELEVATOR_KD = 0;
    // feed forward not used because ElevatorFeedforward is calculated in ElevatorIOSparkMax class
    public static final double ELEVATOR_KFF = .10; // 0.10 = 10%; was 0.0005 in Penn State code (but they have greater gear ratio)

    public static final int ELEV_AMP = 40;

    //public static final double MAX_VELOCITY = Meters.of(4).per(Second).in(MetersPerSecond);
    //public static final double MAX_ACCELERATION = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);


    /**
     * contsants for the elevator feed forward
     * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/controller/ElevatorFeedforward.html
     * ks - The static gain.
     * kg - The gravity gain.
     * kv - The velocity gain.
     * ka - The acceleration gain.
     */
    //voltage to overcome static friction
    public static final double ELEVATOR_KS = 0.02;
    //voltage to overcome gravity
    public static final double ELEVATOR_KG = 0.9;
    //voltage to overcome velocity
    public static final double ELEVATOR_KV = 3.8;
    //voltage to overcome acceleration
    public static final double ELEVATOR_KA = 0.17;

    public static final double ELEVATOR_SPEED = 0.65;
    public static final double AUTO_ELEVATOR_SPEED = 0.45;                                                                     ; // 0 - 1, multiplier on the press

  }
  public static class CANConstants {

    // Elevator
    public static final int ELEVATOR_LEAD_SM = 21;
    public static final int ELEVATOR_FOLLOWER_SM = 22;

    //Coral Intake
    public static final int WRIST_SM = 18;
    public static final int CORALINTAKE_SM = 19;

    // Algae intake
    public static final int ALGAE_ONE_SM = 17;
    public static final int ALGAE_TWO_SM = 23;

    public static final int ALGAE_SM = 20;

    // climber 
    public static final int CLIMBER_SM = 22;

    // Pigeon - 14 (Also set in JSON Files under modules)
    public static final int PIGEON_ID = 14;

    // PDH - 25 (not used in code)

    // Swerve - ( 1-12 ) - Set in JSON Files under modules

  }

  public static class IntakeConstants {

    public static final double CORAL_KP = 0.01; 
    public static final double CORAL_KI = 0;
    public static final double CORAL_KD = 0;
    public static final double CORAL_KFF = 0.00375;

    public static final int CORAL_AMP = 15;
    public static final int WRIST_AMP = 40;

    public static final int ALGAE_AMP = 15;

    public static final double WRIST_SPEED = 0.05; // speed between 0 (0%) and 1 (100%)
    public static final double INTAKE_SPEED = 0.5; // speed between 0 (0%) and 1 (100%)
    public static final double OUTAKE_SPEED = 0.5;
    public static final double ALGAE_SPEED = 0.8; // speed between 0 (0%) and 1 (100%)
    public static final double AUTO_WRIST_SPEED = 0.05;
    /**
     * contsants for the elevator feed forward
     * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/controller/ElevatorFeedforward.html
     * ks - The static gain.
     * kg - The gravity gain.
     * kv - The velocity gain.
     * ka - The acceleration gain.
     */
    //voltage to overcome static friction
    public static final double WRIST_KS = 0.02;
    //voltage to overcome gravity
    public static final double WRIST_KG = 0.9;
    //voltage to overcome velocity
    public static final double WRIST_KV = 3.8;
    //voltage to overcome acceleration
    public static final double WRIST_KA = 0.17;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_AMP = 40;
    public static final int MOTOR_GEAR_RATIO = 100;
    public static final int CAN_TIMEOUT = 250;

    public static final double CLIMB_SPEED = 1.00;
  }

  public static class ReefscapeConstants {
    public static final double PROCESSOR_HEIGHT = 0;     
    public static final double SOURCE_HEIGHT =  9.5;
    public static final double L1_HEIGHT = 17; // 11.499929; // 3;
    public static final double L2_HEIGHT = 18; // 19 was good for shooting it
    public static final double L3_HEIGHT = 40; // 40; // 39.594841;
    public static final double L4_HEIGHT = 65; // 64; // 65.239532;
    public static final double TOP_ALGAE_HEIGHT = 50; // PROCESSOR_HEIGHT + 34.5;
  
    public static final double START_ANGLE = 0.065342;
    public static final double GREATEST_ANGLE = 0.33;
    public static final double SOURCE_ANGLE = 0.1;
    public static final double L1_ANGLE = 0.3;
    public static final double L2_ANGLE = 0.3;
    public static final double L3_ANGLE = 0.3;
    public static final double L4_ANGLE = 0.3;
    public static final double TEST_ANGLE = 0.15;


    // public static final double WRIST_CONVERSION_FACTOR = 243 / 9; // equals 27
    // public static final double PROCESSOR_ANGLE = 0;
    // public static final double SOURCE_ANGLE = 0.15 * WRIST_CONVERSION_FACTOR;
    // public static final double L1_ANGLE = 5; // 0.1 * WRIST_CONVERSION_FACTOR; // 0.35 * WRIST_CONVERSION_FACTOR;
    // public static final double L2_ANGLE = 5; // 6.595242; // 0.255 * WRIST_CONVERSION_FACTOR;
    // public static final double L3_ANGLE = 5; // 6.595342; //0.27 * WRIST_CONVERSION_FACTOR;
    // public static final double L4_ANGLE = 5; // 6.595342; // 0.335 * WRIST_CONVERSION_FACTOR;
    // public static final double TOP_ALGAE_ANGLE = 0;

    public static final double DRIVE_OFF_LINE = 1.0;
    public static final double DRIVE_OFF_SPEED = .40;
  }

}
