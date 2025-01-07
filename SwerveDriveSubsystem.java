package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
    // Kraken drive motors
    private final CANSparkMax frontLeftDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax frontRightDriveMotor = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax backLeftDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax backRightDriveMotor = new CANSparkMax(4, MotorType.kBrushless);

    // NEO turning motors
    private final CANSparkMax frontLeftTurnMotor = new CANSparkMax(5, MotorType.kBrushless);
    private final CANSparkMax frontRightTurnMotor = new CANSparkMax(6, MotorType.kBrushless);
    private final CANSparkMax backLeftTurnMotor = new CANSparkMax(7, MotorType.kBrushless);
    private final CANSparkMax backRightTurnMotor = new CANSparkMax(8, MotorType.kBrushless);

    public SwerveDriveSubsystem() {
        // Initialize motors if needed
    }

    public void drive(double forward, double strafe, double rotation) {
        // Calculate the desired speed and angle for each swerve module
        // This is a simplified example, you would need to implement the full kinematics for a real robot

        double frontLeftSpeed = forward + strafe + rotation;
        double frontRightSpeed = forward - strafe - rotation;
        double backLeftSpeed = forward - strafe + rotation;
        double backRightSpeed = forward + strafe - rotation;

        // Set the drive motors' speeds
        frontLeftDriveMotor.set(frontLeftSpeed);
        frontRightDriveMotor.set(frontRightSpeed);
        backLeftDriveMotor.set(backLeftSpeed);
        backRightDriveMotor.set(backRightSpeed);

        // Set the turn motors' positions (this is a placeholder, you would need to calculate the actual angles)
        frontLeftTurnMotor.set(0);
        frontRightTurnMotor.set(0);
        backLeftTurnMotor.set(0);
        backRightTurnMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}

// filepath: /src/main/java/frc/robot/RobotContainer.java
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();

    // Logitech Extreme 3D Pro joystick
    private final Joystick joystick = new Joystick(0);

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Example of binding joystick axes to swerve drive
        CommandScheduler.getInstance().registerSubsystem(swerveDriveSubsystem);
        CommandScheduler.getInstance().schedule(() -> {
            double forward = -joystick.getY(); // Forward/backward
            double strafe = joystick.getX(); // Left/right
            double rotation = joystick.getTwist(); // Rotation

            swerveDriveSubsystem.drive(forward, strafe, rotation);
        });
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}

// filepath: /src/main/java/frc/robot/Robot.java
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }
}