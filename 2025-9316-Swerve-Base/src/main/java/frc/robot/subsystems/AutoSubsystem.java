package frc.robot.subsystems;
import java.util.logging.*;

import java.io.Console;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class AutoSubsystem extends SubsystemBase{

    private static final Logger logger = Logger.getLogger(AutoSubsystem.class.getName());

    public static Command getAutoCommand(String autoName) {
        logger.info("Auto Selected: " + autoName);
        return AutoBuilder.buildAuto(autoName); // Load PathPlanner auto
    }
    public enum AutoMode {
        ReefProcessor, // Auto
        StartReef,
    }
    public static Command getAutoCommand(AutoMode autoMode) {
        String autoName = autoMode.name();
        logger.info("Auto Selected: " + autoName);
        
        Command autoCommand = AutoBuilder.buildAuto(autoName);
        if (autoCommand == null) {
            logger.severe("Failed to build auto command for: " + autoName);
        }
        return autoCommand; // Load PathPlanner auto
    }
    public static Command ReefProcessor() {
        return new InstantCommand();
    // logger.info("Executing Reef Processor Auto Mode");
    // throw new UnsupportedOperationException("Unimplemented method 'ReefProcessor'");
    }

    public static Command AutoExchange() {
        return new InstantCommand();
        //logger.info("Executing AutoExchange");
        // throw new UnsupportedOperationException("Unimplemented method 'AutoExchange'");
    }
}
