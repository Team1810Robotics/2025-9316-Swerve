package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeCommand extends Command {
    private AlgaeSubsystem algaeSubsystem;
 private boolean eject;
    public AlgaeCommand(AlgaeSubsystem algaeSubsystem, boolean eject){
        this.algaeSubsystem = algaeSubsystem;
        this.eject = eject;
        addRequirements(algaeSubsystem);
    }
 
    @Override
    public void initialize() {
        //algaeSubsystem.distanceSensor.setAutomaticMode(true);
    }

    
        // Reset any other variables to factory defaults
   
    @Override
    public void execute() {
        //algaeSubsystem.setSpeed(1);
        /* if the distance is over 1 meter or not determined, than the motor stops
         * if the distance is less than 1m and greater than 8 cm than the motor runs normally
         * if the distance is less than 8 cm then the motor runs slowly.
         */
        double distance = algaeSubsystem.getDistanceSensor();

        if (eject){
            algaeSubsystem.setSpeed(-1);
           
        }else {
            if (distance > 8 && distance <= 100) {
                algaeSubsystem.setSpeed(0.4);
            } else if (distance <= 8 && distance > 0) {
                algaeSubsystem.setSpeed(0.05);
            } else if (distance <= 0 || distance > 100) {
                algaeSubsystem.stop();
            }
        }
       
    }

    // @Override
    // public boolean isFinished() {
        // double distance = algaeSubsystem.getDistanceSensor();
        // if (distance <= 0 && distance > 0) {
        //     //TODO: tune first value may be 7cm 
        //     return true;
        // } else if(distance < 0){
        //     CommandScheduler.getInstance().schedule(Commands.print("Invalid range"));
        //     return true;
        // }

        // return false;
    // }

    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stop();
    }
}
