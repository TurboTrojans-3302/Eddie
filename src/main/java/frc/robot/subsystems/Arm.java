// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Arm extends SubsystemBase {

    
    private static final double ELBOW_MIN = 5.0;
    private static final double ELBOW_MAX = 50.0;
    private static final int WRIST_MAX = 55;
    private static final double WRIST_MIN = -55;
    //private Compressor compressor;
    private DoubleSolenoid shoulderValve; 
    public DoubleSolenoid extensionValve;
    private DoubleSolenoid gripperValve;
    private CANSparkMax elbowMotor;
    private CANSparkMax wristMotor;
    public RelativeEncoder wristEncoder; //angle of wrist
    public RelativeEncoder elbowEncoder; //angle of elbow
    


 





    /**
    *
    */
    public Arm() {
        //compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        shoulderValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.SHOULDER_VALVE_FWD, RobotMap.SHOULDER_VALVE_BACK);
        extensionValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.EXTENSION_VALVE_IN, RobotMap.EXTENSION_VALVE_OUT);
        gripperValve = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.GRIPPER_VALVE_OPEN, RobotMap.GRIPPER_VALVE_CLOSE);

        shoulderValve.set(Value.kReverse);
        extensionValve.set(Value.kReverse);
        gripperValve.set(Value.kForward);

        elbowMotor = new CANSparkMax(RobotMap.ELBOW_MOTOR_ID, MotorType.kBrushless);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setInverted(true);
        wristMotor = new CANSparkMax(RobotMap.WRIST_MOTOR_ID, MotorType.kBrushless);

        wristEncoder = wristMotor.getEncoder();
        elbowEncoder = elbowMotor.getEncoder();
        
    }

   public final double elbowDegreesPerEncoderCount =  1.0;

    public double getElbowAngle(){
        return elbowDegreesPerEncoderCount * elbowEncoder.getPosition();
    }

    public double getElbowSpeed(){
        return elbowDegreesPerEncoderCount * elbowEncoder.getVelocity();
    }

    public final double wristDegreesPerEncoderCount = (112.4/143.7);

    public double getWristAngle(){
        return wristDegreesPerEncoderCount * wristEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Wrist Angle", getWristAngle());
        SmartDashboard.putNumber("Elbow Angle", getElbowAngle());
        SmartDashboard.putNumber("Elbow Rate", getElbowSpeed());
        SmartDashboard.putString("Shoulder", shoulderValve.get().toString());
        SmartDashboard.putString("Claw", getClawClosed() ? "Closed":"Open");
        SmartDashboard.putString("Extension", getExtensionOut() ? "Out":"In");
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void shoulderForward(boolean fwd){
        if (fwd){
            shoulderValve.set(Value.kForward);
        } else {
            shoulderValve.set(Value.kReverse);
        }
        
    }



    public void elbowMove(double speed){
        if(getElbowAngle() >= ELBOW_MAX){
            speed = Math.min(0, speed);
        } 
        if(getElbowAngle() <= ELBOW_MIN){
            speed = Math.max(0, speed);
        }
        elbowMotor.set(speed);
    }

    public void extensionOut(boolean extend){
        if (extend){
            extensionValve.set(Value.kForward);
        } else {
            extensionValve.set(Value.kReverse);
        }
    }

    public void clawClosed(boolean closed){
        if (closed){
            gripperValve.set(Value.kForward);
        } else {
            gripperValve.set(Value.kReverse);
        }
    }

    public boolean getClawClosed(){
        return Value.kForward == gripperValve.get();
    }

    public boolean getShoulderForward(){
        return Value.kForward == shoulderValve.get();
        
    }

    public boolean getExtensionOut(){
        return Value.kForward == extensionValve.get();
    } 

    public void wristSpin(double speed){
        speed = -speed;

        if(getWristAngle() >= WRIST_MAX ){
            speed = Math.min(0, speed);
        }

        if(getWristAngle() <= WRIST_MIN){
            speed = Math.max(0, speed);
        }

        wristMotor.set(speed);


    }



}
