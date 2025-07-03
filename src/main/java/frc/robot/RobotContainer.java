package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ElevatorManualCommand;
import frc.robot.commands.ElelvatorPositionCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem(0);

    private final XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
        ));

        elevator.setDefaultCommand(new ElevatorManualCommand(
                elevator,
                () -> -driverJoystick.getLeftY()
        ));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, 2)
                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));

        new JoystickButton(driverJoystick, 1)
                .onTrue(new ElelvatorPositionCommand(elevator, 1));

        new JoystickButton(driverJoystick, 3)
                .onTrue(new ElelvatorPositionCommand(elevator, 1.2));
    }
}
