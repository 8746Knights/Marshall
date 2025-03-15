// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.climber.*;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

    // set up the Joystick
    final         CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    //final CommandXboxController m_driverController = new CommandXboxController(0);
    final CommandXboxController m_otherController = new CommandXboxController(1);

  // instantiate a SwerveSubsystem pointing to the deploy/swerve configuration 
  SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  //     
  //SwerveDrive swerveDrive = null;                                                                         


 /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis (m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                            m_driverController::getRightY)
                                                           .headingWhile(true);

/* 
 SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -m_driverController.getLeftY(),
                                                                   () -> -m_driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!

  */
  /*
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                      m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                  m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);
*/
  private Elevator elevator;
  private Climber climber;
  private Intake intake;
  private DigitalInput bottomLimitSwitch;

    // Dashboard inputs
    private static SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    elevator = new Elevator(new ElevatorIOSparkMax());
    elevator.resetPosition();
    climber = new Climber(new ClimberIOSparkMax());
    intake = new Intake(new IntakeIOSparkMax());
    intake.resetPosition();
    bottomLimitSwitch = new DigitalInput(26);
    

   // autoChooser = new SendableChooser<>();
   SmartDashboard.putData(autoChooser);

    autoChooser = AutoBuilder.buildAutoChooser();


    // ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    // autoTab.add(autoChooser);

   // Command autoCommand = new RunCommand (() -> drivebase.driveToDistanceCommand(ReefscapeConstants.DRIVE_OFF_LINE,ReefscapeConstants.DRIVE_OFF_SPEED),drivebase);
   // autoChooser.addOption("Drive Off line", autoCommand);
    
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I AM MJ"));
    /*try {
     SwerveDrive swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Units.feetToMeters(14.5));
    } catch ( Exception e) {
      System.out.println("Caught Exception instantiating SwerveDrive" + e);
     }*/
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  
    // Command driveFieldOrientedDirectAngle         = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY());
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);

    Command driveRobotOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> m_driverController.getRightX(),
      () -> m_driverController.getRightY());
  Command driveRobotOrientedAnglularVelocity    = drivebase.drive(driveAngularVelocity);

    Command driveSetpointGen                      = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
   // Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
   // Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    // Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
       // driveDirectAngleSim);

      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

    if (RobotBase.isSimulation())
    {
      // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    Pigeon2 pigeon = new Pigeon2(CANConstants.PIGEON_ID);
    pigeon.reset();

    /*
     * CLIMBER COMMANDS & CONTROLS
     */
    //TODO: Set these numbers??
    Command climbUpCommand =
        new StartEndCommand(() -> climber.set(Constants.ClimberConstants.CLIMB_SPEED), () -> climber.stopMotor(), climber);
    m_driverController.povUp().whileTrue(climbUpCommand);

    Command climbDownCommand =
        new StartEndCommand(() -> climber.set(-Constants.ClimberConstants.CLIMB_SPEED), () -> climber.stopMotor(), climber);
    m_driverController.povDown().whileTrue(climbDownCommand);

    Command climbHoldCommand =
        new StartEndCommand(
            () -> climber.setMotorVoltage(-0.75), () -> climber.stopMotor(), climber);
    m_driverController.povLeft().whileTrue(climbHoldCommand);
   


    // ADDED FOR TESTING????
/*
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.back().whileTrue(drivebase.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
      m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      m_driverController.start().whileTrue(Commands.none());
      m_driverController.back().whileTrue(Commands.none());
      m_driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.rightBumper().onTrue(Commands.none());
    }

*/

    /*
     * ALGAE COMMANDS & CONTROLS
     */
    Command ejectAlgaeCommand =
        new StartEndCommand(
            () -> intake.setAlgaeVoltage(12), () -> intake.setAlgaeVoltage(0), intake);
    m_otherController.rightBumper().whileTrue(ejectAlgaeCommand);

    Command intakeAlgaeCommand =
        new StartEndCommand(
            () -> intake.setAlgaeVoltage(-12), () -> intake.setAlgaeVoltage(0), intake);
               
    // m_driverController.rightTrigger().whileTrue(intakeAlgaeCommand);
    m_otherController.rightTrigger(0.25).whileTrue(intakeAlgaeCommand);

    /*
     * ALGAE PROCESSOR
     */
    Command liftToProcessorCommand =
        new RunCommand(() -> elevator.setPosition(ReefscapeConstants.PROCESSOR_HEIGHT), elevator);
    Command wristToProcessorCommand =
        new RunCommand(() -> intake.wristAngle(ReefscapeConstants.PROCESSOR_ANGLE), intake);
    //ParallelCommandGroup processorCommandGroup =
        //new ParallelCommandGroup(liftToProcessorCommand, wristToProcessorCommand);
    // m_otherController.povDown().onTrue(liftToProcessorCommand);


    /*
     * CORAL INTAKE
     */
    Command intakeCoralCommand =
      new StartEndCommand(
            () -> intake.setIntakeSpeed(-IntakeConstants.INTAKE_SPEED), () -> intake.setCoralIntakeVoltage(0), intake);
    m_otherController.leftTrigger(0.25).whileTrue(intakeCoralCommand);

    Command ejectCoralCommand =
        new StartEndCommand(
            () -> intake.setIntakeSpeed(IntakeConstants.INTAKE_SPEED), () -> intake.setCoralIntakeVoltage(0), intake);
    m_otherController.leftBumper().whileTrue(ejectCoralCommand);

        // set elevator and wrist to intake position
    Command liftToSourceCommand =
        new RunCommand(() -> elevator.setPosition(ReefscapeConstants.SOURCE_HEIGHT), elevator);
    Command wristToSourceCommand = new RunCommand(() -> intake.wristAngle(ReefscapeConstants.SOURCE_ANGLE), intake);
    //ParallelCommandGroup sourceCommandGroup =
      //  new ParallelCommandGroup(liftToSourceCommand, wristToSourceCommand);
    // m_otherController.povLeft().onTrue(liftToSourceCommand);

    /*
     * REEF STATES
     */
    // L1 state
    Command liftToL1Command = new RunCommand(() -> elevator.setPosition(ReefscapeConstants.L1_HEIGHT), elevator);
    Command wristToL1Command = new RunCommand(() -> intake.wristAngle(ReefscapeConstants.L1_ANGLE), intake);
    // ParallelCommandGroup l1CommandGroup =
        // new ParallelCommandGroup(liftToL1Command, wristToL1Command);
    // m_otherController.a().onTrue(liftToL1Command);
    
  
    // L2 state
    Command liftToL2Command = new RunCommand(() -> elevator.setPosition(ReefscapeConstants.L2_HEIGHT), elevator);
    Command wristToL2Command = new RunCommand(() -> intake.wristAngle(ReefscapeConstants.L2_ANGLE), intake);
    ParallelCommandGroup l2CommandGroup =
        new ParallelCommandGroup(liftToL2Command, wristToL2Command);
    // m_otherController.b().onTrue(l2CommandGroup);

    // L3 state
    Command liftToL3Command = new RunCommand(() -> elevator.setPosition(ReefscapeConstants.L3_HEIGHT), elevator);
    Command wristToL3Command = new RunCommand(() -> intake.wristAngle(ReefscapeConstants.L3_ANGLE), intake);
    //ParallelCommandGroup l3CommandGroup =
        //new ParallelCommandGroup(liftToL3Command, wristToL3Command);
    // m_otherController.y().onTrue(liftToL3Command);

    // L4 state
    Command liftToL4Command = new RunCommand(() -> elevator.setPosition(ReefscapeConstants.L4_HEIGHT), elevator);
    Command wristToL4Command = new RunCommand(() -> intake.wristAngle(ReefscapeConstants.L4_ANGLE), intake);
    //ParallelCommandGroup l4CommandGroup =
       // new ParallelCommandGroup(liftToL4Command, wristToL4Command);
    // m_otherController.x().onTrue(liftToL4Command);

    // Top algae state
    Command liftToTopAlgaeCommand =
        new RunCommand(() -> elevator.setPosition(ReefscapeConstants.TOP_ALGAE_HEIGHT), elevator);
    Command wristToTopAlgaeCommand =
        new RunCommand(() -> intake.wristAngle(ReefscapeConstants.TOP_ALGAE_ANGLE), intake);
    //ParallelCommandGroup topAlgaeCommandGroup =
        //new ParallelCommandGroup(liftToTopAlgaeCommand, wristToTopAlgaeCommand);
    // m_otherController.povUp().onTrue(liftToTopAlgaeCommand);

    /*
     * Manual Controls
     */
    Command manualLift =
        new RunCommand(() -> elevator.setVoltage(-m_otherController.getLeftY() * Constants.ElevatorConstants.ELEVATOR_SPEED), elevator);
    Command manualWrist =
        new RunCommand(() -> intake.setWristVoltage(m_otherController.getRightY() * 0.25),
     intake);
     // ParallelCommandGroup manualCommandGroup = new ParallelCommandGroup(manualLift, manualWrist);
    // m_otherController.start().whileTrue(manualCommandGroup);
     m_otherController.leftStick().onTrue(manualLift);
    m_otherController.rightStick().onTrue(manualWrist);

    Command resetElevatorPigeon =
      new RunCommand(() -> elevator.resetPosition(), elevator);
    m_otherController.start().onTrue(resetElevatorPigeon);

    Command resetWristPigeon = 
      new RunCommand(() -> intake.resetPosition(), intake);
    m_driverController.start().onTrue(resetWristPigeon);



    // Auto Place L4 and Back
    /* 
    Command autoPlaceL4AndGoBackElevator = 
      new RunCommand(() -> elevator.setPosition(ReefscapeConstants.L4_HEIGHT), elevator);
    Command autoPlaceL4AndGoBackWrist = 
      new RunCommand(() -> intake.wristAngle(ReefscapeConstants.L4_ANGLE), intake);
    Command autoPlaceL4AndGoBackIntake =
      new RunCommand(() -> intake.setIntakeSpeed
      */     
      
      /*
  if (bottomLimitSwitch.get() == true)
  {
    elevator.resetPosition();
  }
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    // return drivebase.getAutonomousCommand("New Auto");
    return new PathPlannerAuto("RedLeftSideAuto");
  }

  public void execute() {
    SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
    SmartDashboard.putNumber("Wrist Position", intake.getWristPosition());
  }
  
}
