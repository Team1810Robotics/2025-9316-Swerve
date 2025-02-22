package frc.robot.subsystems;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.jni.DistanceSensorJNIWrapper;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.*;


import com.revrobotics.spark.SparkLowLevel.MotorType;

//Note Details on distance center driver installed from here: https://github.com/REVrobotics/2m-Distance-Sensor/?tab=readme-ov-file
public class AlgaeSubsystem extends SubsystemBase {
   private SparkMax motor;

   public Rev2mDistanceSensor distanceSensor;

    public AlgaeSubsystem() {
        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        motor = new SparkMax(AlgaeConstants.MOTOR_ID, MotorType.kBrushless);
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }
    public void reset(){
       
    }

     public double getDistanceSensor() {
         if (distanceSensor.isRangeValid()){
             return distanceSensor.getRange();
         }
         return -1;
     }

    public void stop(){
        motor.stopMotor();
    }
}
