package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    
    private final ElevatorSubsystem elevatorSubsystem;
    private final double motorSpeed;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double motorSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.motorSpeed = motorSpeed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setPower(motorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setPower(0); // Stop the motor
    }
}
