package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final XboxController driverController = new XboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(new RunCommand(
            () -> swerveSubsystem.drive(
                driverController.getLeftY(),
                driverController.getLeftX(),
                driverController.getRightX()
            ), swerveSubsystem
        ));
    }

    public Command getAutonomousCommand() {
        return null; // Add autonomous commands here
    }
}
