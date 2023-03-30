package frc.robot;

public class RobotMap {
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2; 
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 9; 
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1; 

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 8; 
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 12; 
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 7; 

    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 4; 
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 10; 
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 3; 

    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 6; 
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 11; 
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 5; 
    public static final int ELBOW_MOTOR_ID = 13;
    public static final int WRIST_MOTOR_ID = 14;
   //just added and need to be put into the rest of the code
    public static final int SHOULDER_ANGLE_ENCODER = 15;
    public static final int ELBOW_ANGLE_ENCODER = 16;
    public static final int WRIST_ANGLE_ENCODER = 17;
    public static final int EXTENSION_AMOUNT_ENCODER = 18;
    //maximum position in degrees
    //just an estimate change later
    public static final double shoulderMaximumPosition = 90.0;


//pneumatics id's
    public static final int SHOULDER_VALVE_FWD  = 0;
    public static final int SHOULDER_VALVE_BACK = 1;
    public static final int EXTENSION_VALVE_OUT = 5;
    public static final int EXTENSION_VALVE_IN  = 4;
    public static final int GRIPPER_VALVE_CLOSE = 3;
    public static final int GRIPPER_VALVE_OPEN  = 2;

    //PWM Channels
    public static final int BLINKIN_LED_PWM_CHANNEL = 0;

}
