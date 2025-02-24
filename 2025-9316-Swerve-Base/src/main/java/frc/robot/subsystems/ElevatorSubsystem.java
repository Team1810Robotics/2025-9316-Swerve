package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final SparkMax elevatorMotor2;
    private final Encoder elevatorEncoder;
    private final PIDController elevatorPID;

    private static final double MAX_HEIGHT = 72.0;
    private static final double MIN_HEIGHT = 0.0;
    public static final double INTAKE_POSITION = 0.0;
    public static final double L1_POSITION = 10.0;       
    public static final double L2_POSITION = 18.0;
    public static final double L3_POSITION = 36.0;
    public static final double HIGH_ALGAE_POSITION = 64;
    public static final double MANUAL_ADJUST_INCREMENT = 2.0; // Small adjustment for manual control

    // Elastic Dashboard via NetworkTables
    private final NetworkTable dashboardTable;
    private final NetworkTableEntry positionEntry;
    private final NetworkTableEntry currentEntry;

    public final CoralHandlerSubsystem coralHandler;

    public ElevatorSubsystem(CoralHandlerSubsystem coralHandler) {
        this.coralHandler = coralHandler;

        // Initialize Motors
        elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);

        // Configure Motors
        SparkMaxConfig config_ = new SparkMaxConfig();
        SparkMaxConfig config_2 = new SparkMaxConfig();

        config_.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.smartCurrentLimit);
        config_2.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.smartCurrentLimit)
                .follow(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID, true); // Inverted follow

        elevatorMotor.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        elevatorMotor2.configure(config_2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // Initialize Encoder
        elevatorEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        double pulsesPerRevolution = 1024;
        double inchesPerRevolution = Math.PI * 2; // Assuming 2-inch pulley
        elevatorEncoder.setDistancePerPulse(inchesPerRevolution / pulsesPerRevolution);

        // Initialize PID Controller
        elevatorPID = new PIDController(0.05, 0.0, 0.0); // Adjust constants as needed
        elevatorPID.setTolerance(0.5); // Allowable error range

        // Elastic Dashboard via NetworkTables
        dashboardTable = NetworkTableInstance.getDefault().getTable("Elevator");
        positionEntry = dashboardTable.getEntry("Elevator Position");
        currentEntry = dashboardTable.getEntry("Motor Current");
    }

    // Setpoint method for PID control
    public void setElevatorPosition(double targetPosition) {

        if (coralHandler.isElevatorLocked()) {
            stop(); //stop movement if locked
            System.out.println("Elevator is LOCKEd by Coral Handler");
            return;
        }

        if (isWithinBounds(targetPosition)) {
            double power = elevatorPID.calculate(getElevatorPosition(), targetPosition);
            elevatorMotor.set(power);
        } else {
            stop();
        }
    }

    public void stop() {
        elevatorMotor.set(0);
    }

    public double getElevatorPosition() {
        return elevatorEncoder.getDistance();
    }

    public boolean isWithinBounds(double position) {
        return position >= MIN_HEIGHT && position <= MAX_HEIGHT;
    }

    @Override
    public void periodic() {
        // Update Elastic Dashboard via NetworkTables
        positionEntry.setDouble(getElevatorPosition());
        currentEntry.setDouble(elevatorMotor.getOutputCurrent());
    }
}
