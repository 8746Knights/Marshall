package frc.robot.subsystems.swervedrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ReverseDoubleSupplier implements DoubleSupplier {

    CommandXboxController xboxController;

    public ReverseDoubleSupplier (CommandXboxController target){
        xboxController = target;
    }

     @Override
    public double getAsDouble() {
        double value = xboxController.getRightX();

        return (value * -1);
    }

}
