// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.HashMap;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.ElevatorCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AutoSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import org.elasticsearch.action.index.IndexRequest;
import org.elasticsearch.client.RequestOptions;
import org.elasticsearch.client.RestHighLevelClient;
import org.elasticsearch.client.RestClient;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.elasticsearch.client.RequestOptions;
import org.elasticsearch.client.RestHighLevelClient;
import org.elasticsearch.client.RestClient;
import org.elasticsearch.action.index.IndexRequest;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused") // For now :) 

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController xbox = new CommandXboxController(1);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final CoralHandlerSubsystem coralHandler = new CoralHandlerSubsystem();
    
    public final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(coralHandler); // Initialize Elevator Subsystem
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
   public RobotContainer(){
        algaeSubsystem.setDefaultCommand(new AlgaeCommand(algaeSubsystem, false));
        setElastic();
        configureBindings();
        configureAutoChooser();
    }

    public void setElastic(){
     //   teleopTab.addDouble("Match Time", () -> DriverStation.getMatchTime());
     //   teleopTab.addDouble("Algae Distance", () -> algaeSubsystem.getDistanceSensor());
    }
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on a button press
        joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Algae Control
        xbox.leftTrigger().whileTrue(new AlgaeCommand(algaeSubsystem, true));

        // Elevator Controls
        xbox.a().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.INTAKE_POSITION));    // Intake
        xbox.b().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.L1_POSITION));        // L1
        xbox.x().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.L2_POSITION));        // L2
        xbox.y().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.L3_POSITION));        // L3
        xbox.rightBumper().onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.HIGH_ALGAE_POSITION)); // HighAlgae

        // Elevator Emergency Stop
        xbox.back().onTrue(new InstantCommand(() -> elevatorSubsystem.stop()));

        //Eject Coral
        xbox.rightTrigger().whileTrue(new InstantCommand(() -> coralHandler.startOuttake()))
                   .onFalse(new InstantCommand(() -> coralHandler.stopCoralHandler()));

        // Manual Adjustments for Elevator
        xbox.povUp().onTrue(new ElevatorCommand(elevatorSubsystem, true));     // Manual Up
        xbox.povDown().onTrue(new ElevatorCommand(elevatorSubsystem, false));  // Manual Down
        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureAutoChooser() {
        // Set default option
      //  autoChooser.setDefaultOption("No Auto", new InstantCommand(() -> AutoSubsystem.getAutoCommand("NoPath")));

        // Add PathPlanner paths
      //  autoChooser.addOption("2 Left Auto", AutoSubsystem.getAutoCommand("2LeftAuto"));
      //  autoChooser.addOption("2 Right Auto", AutoSubsystem.getAutoCommand("2RightAuto"));
      //  autoChooser.addOption("Left Auto", AutoSubsystem.getAutoCommand("LeftAuto"));
      //  autoChooser.addOption("Right Auto", AutoSubsystem.getAutoCommand("RightAuto"));
      //  autoChooser.addOption("Middle Auto", AutoSubsystem.getAutoCommand("MiddleAuto"));

        // Display on SmartDashboard
        //SmartDashboard.putData("Auto choices", autoChooser);
    }
    public Command getAutonomousCommand() {
     //    if (autoChooser.getSelected() != null){
     //       return autoChooser.getSelected();
     //   } else {
            return Commands.print("No autonomous command configured, if a path was chosen, this is an error.");
     //   }
    }
}
