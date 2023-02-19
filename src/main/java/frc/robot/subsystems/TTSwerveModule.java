package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;
import com.revrobotics.*;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import static com.swervedrivespecialties.swervelib.rev.RevUtils.checkNeoError;


public class TTSwerveModule implements SwerveModule {

    private SteerControllerImplementation mSteerController;
    private DriveControllerImplementation mDriveController;
    private Mk4ModuleConfiguration mModuleConfiguration;

    public TTSwerveModule(
        ModuleConfiguration mechanicalConfiguration,
        int driveMotorPort,
        int steerMotorPort,
        int steerEncoderPort,
        double steerOffset
    ){
        mModuleConfiguration = new Mk4ModuleConfiguration();

        // make the drive controller. copied from NeoDriveControllerFactoryBuilder

        double nominalVoltage = mModuleConfiguration.getNominalVoltage();
        double currentLimit = mModuleConfiguration.getDriveCurrentLimit();
    
        CANSparkMax driveMotor = new CANSparkMax(driveMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        driveMotor.setInverted(mechanicalConfiguration.isDriveInverted());
    
        checkNeoError(driveMotor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
    
        checkNeoError(driveMotor.setSmartCurrentLimit((int) currentLimit), "Failed to set current limit for NEO");
    
        checkNeoError(driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
        checkNeoError(driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
        checkNeoError(driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
        // Set neutral mode to brake
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
        // Setup encoder
        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        double positionConversionFactor = Math.PI * mechanicalConfiguration.getWheelDiameter() * mechanicalConfiguration.getDriveReduction();
        driveEncoder.setPositionConversionFactor(positionConversionFactor);
        driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60.0);

        mDriveController = new DriveControllerImplementation(driveMotor, driveEncoder);

        // create steer controller. copied from NeoSteerControllerFactoryBuilder

        CanCoderAbsoluteConfiguration encoderconfig = new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset);
        NeoSteerConfiguration<CanCoderAbsoluteConfiguration> steerConfiguration = new NeoSteerConfiguration<>(steerMotorPort, encoderconfig);


        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(encoderconfig.getOffset());
        config.sensorDirection = false;

        CANCoder encoder = new CANCoder(encoderconfig.getId());
        CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

        CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250), "Failed to configure CANCoder update rate");

        EncoderImplementation absoluteEncoder = new EncoderImplementation(encoder);
        absoluteEncoder.setInverted(true);

        //AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> encoderFactory = new CanCoderFactoryBuilder().withReadingUpdatePeriod(100).build();
        //AbsoluteEncoder absoluteEncoder = encoderFactory.create(encoderconfig);

        CANSparkMax steerMotor = new CANSparkMax(steerConfiguration.getMotorPort(), CANSparkMaxLowLevel.MotorType.kBrushless);
        checkNeoError(steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
        checkNeoError(steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
        checkNeoError(steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
        checkNeoError(steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
        steerMotor.setInverted(!mechanicalConfiguration.isSteerInverted());
            checkNeoError(steerMotor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
            checkNeoError(steerMotor.setSmartCurrentLimit((int) Math.round(currentLimit)), "Failed to set NEO current limits");

        RelativeEncoder integratedEncoder = steerMotor.getEncoder();
        checkNeoError(integratedEncoder.setPositionConversionFactor(2.0 * Math.PI * mechanicalConfiguration.getSteerReduction()), "Failed to set NEO encoder conversion factor");
        checkNeoError(integratedEncoder.setVelocityConversionFactor(2.0 * Math.PI * mechanicalConfiguration.getSteerReduction() / 60.0), "Failed to set NEO encoder conversion factor");
        checkNeoError(integratedEncoder.setPosition(absoluteEncoder.getAbsoluteAngle()), "Failed to set NEO encoder position");

        final double pidProportional = 1.0;
        final double pidIntegral = 0.0;
        final double pidDerivative = 0.1;
        
        SparkMaxPIDController controller = steerMotor.getPIDController();
        checkNeoError(controller.setP(pidProportional), "Failed to set NEO PID proportional constant");
        checkNeoError(controller.setI(pidIntegral), "Failed to set NEO PID integral constant");
        checkNeoError(controller.setD(pidDerivative), "Failed to set NEO PID derivative constant");

        checkNeoError(controller.setFeedbackDevice(integratedEncoder), "Failed to set NEO PID feedback device");

        mSteerController = new SteerControllerImplementation(steerMotor, absoluteEncoder);
    }            

    public void calibrateSterrRelativeEncoder(){

        mSteerController.calibrateRelativeEncoder();

    }

    private static class DriveControllerImplementation implements DriveController {
        private final CANSparkMax motor;
        private final RelativeEncoder encoder;

        private DriveControllerImplementation(CANSparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getStateVelocity() {
            return encoder.getVelocity();
        }
    }
    
    
    public static class SteerControllerImplementation implements SteerController {

        @SuppressWarnings({"FieldCanBeLocal", "unused"})
        private final CANSparkMax motor;
        private final SparkMaxPIDController controller;
        private final RelativeEncoder motorEncoder;
        private final EncoderImplementation absoluteEncoder;

        private double referenceAngleRadians = 0;

        public SteerControllerImplementation(CANSparkMax motor, EncoderImplementation absoluteEncoder) {
            this.motor = motor;
            this.controller = motor.getPIDController();
            this.motorEncoder = motor.getEncoder();
            this.absoluteEncoder = absoluteEncoder;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }
        
        public void calibrateRelativeEncoder(){
            double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
            motorEncoder.setPosition(absoluteAngle);
        }




        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motorEncoder.getPosition();

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            this.referenceAngleRadians = referenceAngleRadians;

            controller.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);
        }



        @Override
        public double getStateAngle() {
            double motorAngleRadians = motorEncoder.getPosition();
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }

        public double getAbsoluteAngle(){
            return absoluteEncoder.getAbsoluteAngle();
        }
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANCoder encoder;

        private EncoderImplementation(CANCoder encoder) {
            this.encoder = encoder;
        }

        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition());
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public double getPosition() {
            return encoder.getPosition();
        }

        @Override
        public double getVelocity() {
            return encoder.getVelocity();
        }

        @Override
        public REVLibError setPositionConversionFactor(double factor) {
            // TODO Auto-generated method stub
            return REVLibError.kNotImplemented;
        }

        @Override
        public double getPositionConversionFactor() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public REVLibError setVelocityConversionFactor(double factor) {
            // TODO Auto-generated method stub
            return REVLibError.kNotImplemented;
        }

        @Override
        public double getVelocityConversionFactor() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public REVLibError setInverted(boolean inverted) {
            if( ErrorCode.OK == encoder.configSensorDirection(inverted)){
                return REVLibError.kOk;
            } else { 
                return REVLibError.kError;
            }
        }

        @Override
        public boolean getInverted() {
            return encoder.configGetSensorDirection();
        }

        @Override
        public REVLibError setAverageDepth(int depth) {
            // TODO Auto-generated method stub
            return REVLibError.kNotImplemented;
        }

        @Override
        public int getAverageDepth() {
            // TODO Auto-generated method stub
            return 0;
        }

        @Override
        public REVLibError setZeroOffset(double offset) {
            return ErrorCode.OK == encoder.configMagnetOffset(offset) ? REVLibError.kOk : REVLibError.kError;
        }

        @Override
        public double getZeroOffset() {
            return encoder.configGetMagnetOffset();
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }




    @Override
    public double getDriveVelocity() {
        return mDriveController.getStateVelocity();
    }

    @Override
    public double getSteerAngle() {
        return mSteerController.getStateAngle();
    }

    @Override
    public void set(double driveVoltage, double steerAngle) {
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - getSteerAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        mDriveController.setReferenceVoltage(driveVoltage);
        mSteerController.setReferenceAngle(steerAngle);
    }    

    public double getAbsoluteAngle(){
        return mSteerController.getAbsoluteAngle();
    }
}
