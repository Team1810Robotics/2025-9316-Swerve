package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevatorMotor;
    private SparkMax elevatorMotor2;
    private Encoder elevatorEncoder;
    private PIDController elevatorPID;
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    private static final double MAX_HEIGHT = 72.0;
    private static final double MIN_HEIGHT = 0.0;

    public ElevatorSubsystem() { // Corrected constructor name
        elevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);
        SparkMaxConfig config_ = new SparkMaxConfig();
        SparkMaxConfig config_2 = new SparkMaxConfig();
        config_.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.smartCurrentLimit);
        config_2.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.smartCurrentLimit).follow(ElevatorConstants.ELEVATOR_MOTOR_1_ID, true);
        elevatorMotor.configure(config_, SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);
        elevatorMotor2.configure(config_2, SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);

        elevatorEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X); // Correct Encoder instantiation

        elevatorEncoder.setDistancePerPulse(1.0); // Calibrate this value!

        elevatorPID = new PIDController(1.0, 0.0, 0.0); // Correct PIDController instantiation

        elevatorPID.setSetpoint(0); // Correct method call

        elevatorPID.setTolerance(1.0); // Correct method call

        // Initialize limit switches (replace 2 and 3 with your actual pin numbers)
        upperLimitSwitch = new DigitalInput(2);
        lowerLimitSwitch = new DigitalInput(3);

        SmartDashboard.putNumber("Motor Power", elevatorMotor.getOutputCurrent());
    }

    public void controlElevator(double speed) { // Correct method signature
        // Check limits *before* moving
        if ((speed > 0 && !upperLimitSwitch.get()) || (speed < 0 && !lowerLimitSwitch.get())) { // Active low
            // Check software limits if hardware limits are not triggered
            if ((speed > 0 && elevatorEncoder.getDistance() < MAX_HEIGHT)
                    || (speed < 0 && elevatorEncoder.getDistance() > MIN_HEIGHT)) {
                elevatorMotor.set(speed);
                elevatorMotor2.set(speed);
            } else {
                stopElevator();
            }
        } else {
            stopElevator(); // Stop if limit switch is active
        }
    }
    
    public void setPower(double power) {
        elevatorMotor.set(power);
        elevatorMotor2.set(power);
    }
    public void stopElevator() {
        elevatorMotor.set(0);
        elevatorMotor2.set(0);
    }
}