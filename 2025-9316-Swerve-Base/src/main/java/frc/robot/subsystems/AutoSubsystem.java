package frc.robot.subsystems;
import java.util.logging.*;

import java.io.Console;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class AutoSubsystem extends SubsystemBase{

    private static final Logger logger = Logger.getLogger(AutoSubsystem.class.getName());

    public static Command getAutoCommand(String autoName) {
        logger.info("Auto Selected: " + autoName);
        return AutoBuilder.buildAuto(autoName); // Load PathPlanner auto
    }
}

