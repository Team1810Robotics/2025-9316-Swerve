package frc.robot.subsystems;

import java.util.Optional;
import java.util.logging.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {

    private static final Logger logger = Logger.getLogger(AutoSubsystem.class.getName());

    public enum AutoMode {
        ReefProcessor, // Auto
        StartReef,
    }

    /**
     * Retrieves the autonomous command based on the specified AutoMode.
     * 
     * @param autoMode The AutoMode to retrieve the command for.
     * @return An Optional containing the Command if successful, or empty if failed.
     */
    public static Optional<Command> getAutoCommand(AutoMode autoMode) {
        String autoName = autoMode.name();
        logger.info("Auto Selected: " + autoName);
        
        Command autoCommand = AutoBuilder.buildAuto(autoName);
        if (autoCommand == null) {
            logger.severe("Failed to build auto command for: " + autoName);
            return Optional.empty();
        }
        return Optional.of(autoCommand);
    }

    /**
     * Executes the Reef Processor autonomous mode.
     * 
     * @return The command for the Reef Processor mode.
     */
    public static Command ReefProcessor() {
        logger.info("Executing Reef Processor Auto Mode");
        // Implement the functionality for Reef Processor here
        return new Command() {
            @Override
            public void initialize() {
                // Initialization logic for Reef Processor
            }

            @Override
            public void execute() {
                // Execution logic for Reef Processor
            }

            @Override
            public boolean isFinished() {
                return false; // Change this based on your logic
            }
        };
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

    public static Command AutoExchange() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'AutoExchange'");
    }
}