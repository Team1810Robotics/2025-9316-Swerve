package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralHandlerConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private double targetPosition;
    private double adjustment;  // For manual adjustment

    // Constructor for set positions
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = targetPosition;
        this.adjustment = 0.0;
        addRequirements(elevatorSubsystem);
    }

    // Constructor for manual adjustment
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, boolean isUp) {
        adjustment = 0.0;
        this.elevatorSubsystem = elevatorSubsystem;
        this.adjustment = isUp ? ElevatorSubsystem.MANUAL_ADJUST_INCREMENT : -ElevatorSubsystem.MANUAL_ADJUST_INCREMENT;
        targetPosition += adjustment;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
/*         if (targetPosition != null) {
            System.out.println("Moving Elevator to: " + targetPosition);
        } else {
            System.out.println("Adjusting Elevator by: " + adjustment);
        } */
    }

    @Override
    public void execute() {

        if (adjustment == 0) {
            elevatorSubsystem.setElevatorPosition(targetPosition);
        } else {
            elevatorSubsystem.setElevatorPosition(targetPosition, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        /* if (targetPosition != null) {
            return false;
        }
        return true; // Manual adjustments finish immediately after the adjustment */
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
