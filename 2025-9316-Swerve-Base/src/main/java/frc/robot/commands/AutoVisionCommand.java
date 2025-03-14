package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


public class AutoVisionCommand  extends Command{
    private final VisionSubsystem visionSubsystem;
    private final CoralHandlerSubsystem coralHandlerSubsystem;
    public final CommandSwerveDrivetrain drivetrain;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.RobotCentric visDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public AutoVisionCommand(CoralHandlerSubsystem coralHandlerSubsystem, VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain){
        this.visionSubsystem = visionSubsystem;
        this.coralHandlerSubsystem = coralHandlerSubsystem;
        this.drivetrain = drivetrain;
    }


    @Override
    public void execute() {
        drivetrain.applyRequest(() -> visDrive
        // Forward/backward movement based on distance from target
        .withVelocityX(-visionSubsystem.calculateXPower(
           0,
            0.26,
            true) * MaxSpeed)
            
        // Left/right movement to center with the target
        .withVelocityY(visionSubsystem.calculateYPower(
            0,
            -10,
            true) * MaxSpeed)
            
        // Rotation to align parallel to the target
        .withRotationalRate(visionSubsystem.calculateParallelRotationPower(
            0,
            true) * MaxAngularRate)
    );
    }
    public boolean isFinished() {
        return false;
        //return visionSubsystem.getRange().orElse(0.0) < 0.5;
    }

}
