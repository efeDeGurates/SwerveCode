package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorManualCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier speedSupplier;

    public ElevatorManualCommand(ElevatorSubsystem elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setManualSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
