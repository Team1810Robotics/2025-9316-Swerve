package frc.robot.commands;

import java.io.Console;

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

    // TODO make manual adjustment work
    // Currently instead of setting the target position to the manual adjust it instead sets it to -.5 or .5
    //I actually just made some changes but they prob still will not work - needs testing
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, boolean isUp) {
        this.adjustment = 0.0;
        this.elevatorSubsystem = elevatorSubsystem;
        this.adjustment = isUp ? ElevatorSubsystem.MANUAL_ADJUST_INCREMENT+elevatorSubsystem.getElevatorPosition() : -ElevatorSubsystem.MANUAL_ADJUST_INCREMENT+elevatorSubsystem.getElevatorPosition();
       if (elevatorSubsystem.isWithinBounds(this.adjustment)){
        this.targetPosition = this.adjustment;
       } else {
        System.out.println("OUT OF BOUNDS!");
       }
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
        elevatorSubsystem.setElevatorPosition(targetPosition);
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
