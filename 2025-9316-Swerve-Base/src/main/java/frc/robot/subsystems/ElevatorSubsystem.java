package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final SparkMax elevatorMotor2;
    private final Encoder elevatorEncoder;
    private final PIDController elevatorPID;
    private final ElevatorFeedforward elevatorFF;
    private static final double MAX_HEIGHT = 40;
    private static final double MIN_HEIGHT = 0;
    public static final double INTAKE_POSITION = 1.175;
    public static final double L1_POSITION = 6;        //10    
    public static final double L2_POSITION = 10;        //18
    public static final double L3_POSITION = 28;        //36
    public static final double HIGH_ALGAE_POSITION = 70;  //64
    public static final double MANUAL_ADJUST_INCREMENT = 2.0; // Small adjustment for manual control
        private static final double TICKS_PER_INCH = 185.0;
    
        private double lastRawValue = 0.0;
        private int rotationCount = 0;
    
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
    
            // Set Feed Forward
            elevatorFF = new ElevatorFeedforward(0.0, 0.3, 0.0); //TUNE

            // Configure Motors
            SparkMaxConfig config_ = new SparkMaxConfig();
            SparkMaxConfig config_2 = new SparkMaxConfig();
    
            config_.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.smartCurrentLimit);
            config_2.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.smartCurrentLimit)
                    .follow(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID, true); // Inverted follow
    
            elevatorMotor.configure(config_, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
            elevatorMotor2.configure(config_2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
            // Initialize Encoder
            elevatorEncoder = new Encoder(8, 9);
            // elevatorEncoder.setConnectedFrequencyThreshold(100);//
    
            // Initialize PID Controller
            elevatorPID = new PIDController(0.15, 0.01, 0.00001); // Adjust constants as needed
            elevatorPID.setTolerance(0.025); // Allowable error range
    
            // Elastic Dashboard via NetworkTables
            dashboardTable = NetworkTableInstance.getDefault().getTable("Elevator");
            positionEntry = dashboardTable.getEntry("Elevator Position");
            currentEntry = dashboardTable.getEntry("Motor Current");

            Shuffleboard.getTab("Elevator").addNumber("Encoder Raw", () -> -elevatorEncoder.get());
            Shuffleboard.getTab("Elevator").addNumber("Elevator Height", () -> getElevatorPosition());
        }
    
        // Setpoint method for PID control
        public void setElevatorPosition(double targetPosition) {
    
            if (coralHandler.isElevatorLocked()) {
                stop(); //stop movement if locked
                System.out.println("Elevator is LOCKEd by Coral Handler");
                return;
            }
    
            if (isWithinBounds(targetPosition)) {
                double currentVelocity = (targetPosition - getElevatorPosition()) / 0.02;
                double pidOutput = elevatorPID.calculate(getElevatorPosition(), targetPosition);
                double feedforward = elevatorFF.calculate(currentVelocity); //apply feedforward
                double totalPower = pidOutput /*+ feedforward*/;
                //elevatorPID.calculate(getElevatorPosition(), targetPosition); //pre feedforward
                elevatorMotor.set(totalPower);
            } else {
                stop();
            }
        }
    
        public void stop() {
            elevatorMotor.set(0);
        }
    
        public double getElevatorPosition() {
                double currentRawValue = -elevatorEncoder.get() ;
    
                //Rollover Counter...
                // if (currentRawValue - lastRawValue > 0.5) {
                //     rotationCount--;
                // } else if (lastRawValue - currentRawValue > 0.5) {
                //     rotationCount++;
                // }
                
                lastRawValue = currentRawValue;
    
                double totalRotations = rotationCount + currentRawValue;
    
                return (totalRotations / TICKS_PER_INCH);
    }

    public boolean isWithinBounds(double position) {
        return position >= MIN_HEIGHT && position <= MAX_HEIGHT;
    }

    @Override
    public void periodic() {
        // Update Elastic Dashboard via NetworkTables
        double elevatorPosition = getElevatorPosition();
        //System.out.println("Elevator Position: " + elevatorPosition + "inches");
        positionEntry.setDouble(getElevatorPosition());
        currentEntry.setDouble(elevatorMotor.getOutputCurrent());
    }
}
