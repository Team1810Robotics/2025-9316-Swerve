package frc.robot.subsystems;

import java.util.Optional;
import java.util.logging.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.ElevatorCommand;

public class AutoSubsystem extends SubsystemBase {

    private static final Logger logger = Logger.getLogger(AutoSubsystem.class.getName());

    public enum AutoMode {
        goOffline,
        IdealAuto,
        ReefProcessor, // Auto
        OL_CL2,
        }

    /**
     * Retrieves the autonomous command based on the specified AutoMode.
     * 
     * @param autoMode The AutoMode to retrieve the command for.
     * @return An Optional containing the Command if successful, or empty if failed.
     */
    public static Command getAutoCommand(AutoMode autoMode) {
        String autoName = autoMode.name();
        logger.info("Auto Selected: " + autoName);
        
        Command autoCommand = AutoBuilder.buildAuto(autoName);
        if (autoCommand == null) {
            logger.severe("Failed to build auto command for: " + autoName);
            return new InstantCommand(() -> logger.warning("No valid auto command found for " + autoName));
        }
        return autoCommand;
    }


    /**
     * Executes the Start Reef autonomous mode.
     * 
     * @return The command for the Start Reef mode.
     */
    public static Command StartReef() {
        logger.info("Executing Start Reef Auto Mode");
        // Implement the functionality for Start Reef here
        return new Command() {
            @Override
            public void initialize() {
                // Initialization logic for Start Reef
            }

            @Override
            public void execute() {
                // Execution logic for Start Reef
            }

            @Override
            public boolean isFinished() {
                return false; // Change this based on your logic
            }
        };
    }
    //Drop the Coral, get Algae at L2 
    public static Command AutoExchange(CoralHandlerSubsystem coralHandlerSubsystem, ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem){
    return new SequentialCommandGroup(

        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.L2_POSITION).withTimeout(2),
        //new WaitCommand(2.5),

        // Step 1: Output Coral
        //new WaitCommand(1.5)
        //new InstantCommand(() -> coralHandlerSubsystem.stopCoralHandler(), coralHandlerSubsystem),
        //new WaitCommand(1.5),
        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.INTAKE_POSITION).withTimeout(2)      
    );
    }
    public static Command Scoral(CoralHandlerSubsystem coralHandlerSubsystem){
        return new SequentialCommandGroup(
            new InstantCommand(() -> coralHandlerSubsystem.startOuttake(), coralHandlerSubsystem).withTimeout(2),
        );
    }

    
    //Eject Algae
    public static Command EjectAlgae(AlgaeSubsystem algaeSubsystem){
        logger.info("Executing Eject Algae Auto Mode");
        return new AlgaeCommand(algaeSubsystem, true, false);
    }


}