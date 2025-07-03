package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElelvatorPositionCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double target;

    public ElelvatorPositionCommand(ElevatorSubsystem elevator, double target) {
        this.elevator = elevator;
        this.target = target;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setTargetPosition(target);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getPosition() - target) < 0.01;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
