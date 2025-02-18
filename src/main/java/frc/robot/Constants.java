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
    public static final double MAX_SPEED  = Units.feetToMeters(8);
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
    public static final double TURN_CONSTANT    = 6;
  }
  public static class ElevatorConstants {
    public static final int ELEVATOR_SPARKMAX = 15;

    /* Constants for the profiled PID Controller
     * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ProfiledPIDController.html
     * 
     * Kp - The proportional coefficient. Must be >= 0.
     * Ki - The integral coefficient. Must be >= 0.
     * Kd - The differential coefficient. Must be >= 0.
      */
    public static final double ELEVATOR_KP = 5;
    public static final double ELEVATOR_KI = 5;
    public static final double ELEVATOR_KD = 5;


    public static final double MAX_VELOCITY = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double MAX_ACCELERATION = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);


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

  }
}
