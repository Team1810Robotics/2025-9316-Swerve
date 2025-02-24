package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final Double targetPosition;
    private final Double adjustment;  // For manual adjustment

    // Constructor for set positions
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = targetPosition;
        this.adjustment = null;
        addRequirements(elevatorSubsystem);
    }

    // Constructor for manual adjustment
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, boolean isUp) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = null;
        this.adjustment = isUp ? ElevatorSubsystem.MANUAL_ADJUST_INCREMENT : -ElevatorSubsystem.MANUAL_ADJUST_INCREMENT;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        if (targetPosition != null) {
            System.out.println("Moving Elevator to: " + targetPosition);
        } else {
            System.out.println("Adjusting Elevator by: " + adjustment);
        }
    }

    @Override
    public void execute() {

        if (elevatorSubsystem.coralHandler.isElevatorLocked()) {
            elevatorSubsystem.stop();
            System.out.println("Elevator Locked - Stopping Command");
            return; // Skip the rest if locked
        }

        if (targetPosition != null) {
            elevatorSubsystem.setElevatorPosition(targetPosition);
        } else {
            double newSetpoint = elevatorSubsystem.getElevatorPosition() + adjustment;
            elevatorSubsystem.setElevatorPosition(newSetpoint);
        }
    }

    @Override
    public boolean isFinished() {
        if (targetPosition != null) {
            return Math.abs(elevatorSubsystem.getElevatorPosition() - targetPosition) <= 0.5;
        }
        return true; // Manual adjustments finish immediately after the adjustment
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Elevator Command Interrupted");
        } else {
            System.out.println("Elevator Command Complete");
        }
    }
}
