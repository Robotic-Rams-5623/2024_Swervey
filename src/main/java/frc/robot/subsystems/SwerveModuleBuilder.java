package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * This swerve module code is based on the example code given by REV Robotics for their
 * MAXSwerveModule that uses a NEO and 550 motor with two Spark MAXs. The encoders used
 * in the module is a Through Bore Encoder. The code was adapted to use the Vortex motors
 * mounted to the Spark FLEX controller and using the built in Vortex encoders.
 * 
 * If the internal 3 Phrase Hall Effect Sensors in the Vortex motors may not work for the
 * angle encoder which should be an encoder capable of an Absolute position reading.
 */
public class SwerveModuleBuilder {
    private final CANSparkFlex m_driveMotor;
    private final CANSparkFlex m_turnMotor; // Could be a smaller less complex motor if needed

    private final RelativeEncoder m_driveEncoder;
    private final AbsoluteEncoder m_turnEncoder;
    //private final CANcoder m_turnCANcoder;

    private final SparkPIDController m_drivePIDControl;
    private final SparkPIDController m_turnPIDControl;

    public Rotation2d lastAngle;
    
    /*
     * Used to set the actual angle of the wheel relative to the absolute position of the encoder.
     */
    private double m_angleOffset;

    /*
     * Used to control and get the state of the swerve module.
     */
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /*
     * SwerveModule function that all four modules will be configured with.
     */
    public SwerveModuleBuilder(int driveCANid, int turnCANid, int coderCANid, double angleOffset, String Name) {
        /* Create the drive motor object and reset settings */
        m_driveMotor = new CANSparkFlex(driveCANid, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        
        /* Create the angle motor object and reset settings */
        m_turnMotor = new CANSparkFlex(turnCANid, MotorType.kBrushless);
        m_turnMotor.restoreFactoryDefaults();

        /* 
         * Set up the encoders to be the built-in 3 phase hall effect sensors.
         * It is very important that the angle encoder is set up to be an
         * absolute position so that at startup it won't matter what position
         * the wheel are in and the software will just know what direction the
         * wheels are facing.
        */
        m_driveEncoder = m_driveMotor.getEncoder(com.revrobotics.SparkRelativeEncoder.Type.kHallSensor, 42);
        m_turnEncoder = m_turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        //m_turnCANcoder = new CANcoder(turnCANid);

        /* 
         * Confgiure the PID Controllers from the motor PID controllers using the
         * encoders as the feedback sensors
         */
        m_drivePIDControl = m_driveMotor.getPIDController();
        m_drivePIDControl.setFeedbackDevice(m_driveEncoder);
        m_turnPIDControl = m_turnMotor.getPIDController();
        m_turnPIDControl.setFeedbackDevice(m_turnEncoder);

        /*
         * PID Controller Settings
         * Maybe put this in its own function to call?
         */
        
        /* Conversion factor on the drive encoder. The values for position and velocity
         * should be in meters and meters/second. The native units of the encoder are
         * in rotations and RPM.
         */
        m_driveEncoder.setPositionConversionFactor(Constants.SwerveModule.kDriveEncoderPositionFactor);
        m_driveEncoder.setVelocityConversionFactor(Constants.SwerveModule.kDriveEncoderVelocityFactor);

        /* Conversion factor on the turn enocder. The values for position and velocity
         * should be in radians and radians/second. The native WPIlib swerve API uses
         * these units of measure for kinematic calculations.
         */
        m_turnEncoder.setPositionConversionFactor(Constants.SwerveModule.kTurnEncoderPositionFactor);
        m_turnEncoder.setVelocityConversionFactor(Constants.SwerveModule.kTurnEncoderVelocityFactor);

        /* The turning motor shaft will rotate in the oposite direction of the drive
         * motor shaft
         */
        m_turnEncoder.setInverted(Constants.SwerveModule.kTurnEncoderInverted);

        /* Enable PID wrap around for the turning motor. This iwll allow the PID
         * controller to go past zero to get to a setpoint on the otherside of it.
         * i.e. going from 10° to 350° it can pass through 0° instead of going all
         * the way around the opposite way. 
         */
        m_turnPIDControl.setPositionPIDWrappingEnabled(true);
        m_turnPIDControl.setPositionPIDWrappingMinInput(Constants.SwerveModule.kTurnEncoderPositionPIDMinInput);
        m_turnPIDControl.setPositionPIDWrappingMaxInput(Constants.SwerveModule.kTurnEncoderPositionPIDMaxInput);

        /* Set the PID gains for the controller. They will definetly need to be adjusted
         * to avoid any unwanted behavior of the driving motors.
         */
        m_drivePIDControl.setP(Constants.SwerveModule.kDriveP);
        m_drivePIDControl.setI(Constants.SwerveModule.kDriveI);
        m_drivePIDControl.setD(Constants.SwerveModule.kDriveD);
        m_drivePIDControl.setFF(Constants.SwerveModule.kDriveFF);
        m_drivePIDControl.setOutputRange(
            Constants.SwerveModule.kDriveMinOutput,
            Constants.SwerveModule.kDriveMaxOutput
        );

        m_turnPIDControl.setP(Constants.SwerveModule.kTurnP);
        m_turnPIDControl.setI(Constants.SwerveModule.kTurnI);
        m_turnPIDControl.setD(Constants.SwerveModule.kTurnD);
        m_turnPIDControl.setFF(Constants.SwerveModule.kTurnFF);
        m_turnPIDControl.setOutputRange(
            Constants.SwerveModule.kTurnMinOutput,
            Constants.SwerveModule.kTurnMaxOutput
        );

        m_driveMotor.setIdleMode(Constants.SwerveModule.kDriveMotorIdleMode);
        m_driveMotor.setSmartCurrentLimit(Constants.SwerveModule.kDriveMotorCurrentLimit);
        m_turnMotor.setIdleMode(Constants.SwerveModule.kTurnMotorIdleMode);
        m_turnMotor.setSmartCurrentLimit(Constants.SwerveModule.kTurnMotorCurrentLimit);

        /* Burn the configurations to the flash memory of the motor */
        m_driveMotor.burnFlash();
        m_turnMotor.burnFlash();

        m_angleOffset = angleOffset;
        m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
        //m_desiredState.angle = new Rotation2d(m_turnCANcoder.getPosition());
        
        /* COMMENT THIS OUT ONCE CONSTATNS ARE SET */
        //SmartDashboard.putNumber("Angle Offset " + Name, m_turnEncoder.getPosition());

        resetEncoders();
    }


    /**
     * Optimize Turning
     * 
     * @return SwerveModuleState
     */
    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = orientTo2Pi(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if(Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * 
     * 
     * @return
     */
    public double orientTo2Pi(double curAngle, double newAngle){
        double lowerBound;
        double upperBound;
        double lowerOffset = curAngle % 360;

        //Whether the angle is above a full rotation
        if(lowerOffset >= 0){
            lowerBound = curAngle - lowerOffset;
            upperBound = curAngle + (360 - lowerOffset);
        } else {
            upperBound = curAngle - lowerOffset;
            lowerBound = curAngle + (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
            newAngle += 360;
        }
        while(newAngle > upperBound){
            newAngle -= 360;
        }
        if(newAngle - curAngle > 180){
            newAngle += 360;
        }
        return newAngle;
    }

    /**
     * Returns the current state of the module. Make sure to apply
     * the angular offset to the encoder position to get the position
     * relative to the chassis.
     * 
     * @return The current state of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            getAbsoluteAngle()
            //new Rotation2d(m_turnCANcoder.getPosition() - m_angleOffset)
        );
    }

    /**
     * Set the desired state for the swerve module. Make sure to apply
     * the angular offset to the desired state or it will be off by
     * that amount of offset.
     * 
     * @param desiredState = desired state with the speed and angle
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctDesiredState = new SwerveModuleState();
        correctDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_angleOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctDesiredState,
            new Rotation2d(m_turnEncoder.getPosition()));
            //new Rotation2d(m_turnCANcoder.getPosition()));

        // Command driving and turning motor controller towards their respective setpoints.
        m_drivePIDControl.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turnPIDControl.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /**
     * 
     */
    // private void setAngle (SwerveModuleState desiredState) {
    //     Rotation2d angle = 
    //         (Math.abs(desiredState.speedMetersPerSecond) 
    //             <= (Constants.Swerve.kMaxDriveMeterPerSec * 0.01)) 
    //             ? lastAngle : desiredState.angle;
    //     m_turnPIDControl.setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);
    // }

    /**
     * Set all the encoder in the swerve module to zero.
     */
    public void resetEncoders() {
        m_driveEncoder.setPosition(0);
    }

    public double getTurningVelocity() {
        return m_turnEncoder.getVelocity();
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRadians(m_turnEncoder.getPosition());
    }

    public double getAngle() {
        return m_turnEncoder.getPosition();
    }

    /**
     * Returns the current position of the module.Make sure to apply
     * the angular offset of the encoder angle to get the position
     * relative to the chassis.
     * 
     * @return The current position of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(), getAbsoluteAngle());
    }

    /**
     * Stop Motors
     */
    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
    }

    /**
     * Called once every scheduler run
     */
    public void periodic() {
        SmartDashboard.putNumber("Drive Velocity", getDriveVelocity());
        //SmartDashboard.putNumber("Turn Angle", getAngle());
    }
}
