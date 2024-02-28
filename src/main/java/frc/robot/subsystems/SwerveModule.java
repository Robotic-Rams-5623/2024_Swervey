package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
public class SwerveModule {
    private CANSparkFlex m_driveMotor;
    private CANSparkFlex m_turnMotor;
    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turnEncoder;
    private CANcoder m_absEncoder;
    
    private SparkPIDController m_drivePIDController;
    private SparkPIDController m_turnPIDController;
    private SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(
        Constants.SwerveModule.kDriveS, 
        Constants.SwerveModule.kDriveV
    );

    private double offset;
    private double m_referenceAngleRadians = 0;
    public String modName;
    // private double drivePIDControllerOutput = 0;
    // private double driveFFControllerOutput = 0;
    // private double driveMotorVel = 0;
    // private double driveMotorVelCF = 0;

    
    
    /**
     * Initializes a new SwerveModule
     * 
     * @param name                  the name of the module for logging/dashboard purposes
     * @param driveCANid            the CAN ID of the drive motor
     * @param turnCANid             the CAN ID of the turn motor
     * @param coderCANid            the CAN ID of the CTRE CANcoder
     * @param angleOffset           the offset angle in radians from the zero point
     * @param driveMotorInvert
     * @param driveEncInvert
     * @param turnMotorInvert
     * @param turnEncInvert
     */
    public SwerveModule(
        String Name,
        int driveCANid, 
        int turnCANid, 
        int coderCANid, 
        double angleOffset,
        boolean driveMotorInvert,
        boolean driveEncInvert,
        boolean turnMotorInvert,
        boolean turnEncInvert) {

        this.modName = Name;

        /* DRIVE MOTOR */
        m_driveMotor = new CANSparkFlex(driveCANid, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults(); // Reset All Settings
        m_driveMotor.setInverted(driveMotorInvert);
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setSmartCurrentLimit(Constants.SwerveModule.kDriveMotorCurrentLimit);
        m_driveMotor.enableVoltageCompensation(11.5);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 0);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 0);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 0);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 0);

        /** DRIVE ENCODER */
        m_driveEncoder = m_driveMotor.getEncoder();
        // m_driveEncoder.setInverted(driveEncInvert);
        m_driveEncoder.setPositionConversionFactor(Constants.SwerveModule.kDriveEncoderPositionFactor);
        m_driveEncoder.setVelocityConversionFactor(Constants.SwerveModule.kDriveEncoderVelocityFactor);
        // m_driveEncoder.setAverageDepth(4);
        // m_driveEncoder.setMeasurementPeriod(16);
        m_driveEncoder.setPosition(0.0);

        /** DRIVE PID CONTROLLER */
        m_drivePIDController = m_driveMotor.getPIDController();
        m_drivePIDController.setP(Constants.SwerveModule.kDriveP);
        m_drivePIDController.setFeedbackDevice(m_driveEncoder);
        // m_drivePIDController.setI(Constants.SwerveModule.kDriveI); Don't Really Need These
        // m_drivePIDController.setD(Constants.SwerveModule.kDriveD); They Make things Too Difficult Anyways
        // m_drivePIDController.setFF(Constants.SwerveModule.kDriveFF);


        
        /** TURN MOTOR */
        m_turnMotor = new CANSparkFlex(turnCANid, MotorType.kBrushless);
        m_turnMotor.restoreFactoryDefaults();
        m_turnMotor.setInverted(turnMotorInvert);
        m_turnMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setSmartCurrentLimit(Constants.SwerveModule.kTurnMotorCurrentLimit);
        m_turnMotor.enableVoltageCompensation(11.5);
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 0);
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 0);       
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 0);       
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 0);       

        /** TURN ENCODER (RELATIVE) */
        m_turnEncoder = m_turnMotor.getEncoder();
        m_turnEncoder.setPositionConversionFactor(Constants.SwerveModule.kDriveEncoderPositionFactor);
        m_turnEncoder.setVelocityConversionFactor(Constants.SwerveModule.kDriveEncoderVelocityFactor);
        //m_turnEncoder.setInverted(turnEncInvert);
        // m_turnEncoder.setMeasurementPeriod(16);
        // m_turnEncoder.setAverageDepth(4);

        /** TURN PID CONTORLLER */
        m_turnPIDController = m_turnMotor.getPIDController();
        m_turnPIDController.setP(Constants.SwerveModule.kTurnP);
        m_turnPIDController.setFeedbackDevice(m_turnEncoder);
        m_turnPIDController.setPositionPIDWrappingEnabled(true);
        m_turnPIDController.setPositionPIDWrappingMinInput(0);
        m_turnPIDController.setPositionPIDWrappingMaxInput(Math.PI * 2);

        /** TURN ENCODER (ABSOLUTE) */
        m_absEncoder = new CANcoder(coderCANid);
        m_absEncoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoderConfigurator m_absEncoderConfig = m_absEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();
        m_absEncoderConfig.refresh(magnetSensorConfiguration);
        m_absEncoderConfig.apply(magnetSensorConfiguration.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1).withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        // m_absEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // m_absEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // m_absEncoder.getPosition().setUpdateFrequency(10);
        // m_absEncoder.getPosition().setUpdateFrequency(10);
        // m_absEncoder.getConfigurator().apply(m_absEncoderConfig);

        /** OTHER THINGS TO SETUP */
        offset = angleOffset;
        m_turnEncoder.setPosition(getAbsEncoder());
        
        /** SAVE MOTOR CONFIGURATIONS */
        m_driveMotor.burnFlash();
        m_turnMotor.burnFlash();  
    }

    /**
     * GET STATE OF MODULE
     *
     * @return the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getStateAngle()));
    }

    /**
     * GET POSITION OF MODULE
     *
     * @return the current position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(getStateAngle()));
    }

    /**
     * SET THE DESIRED STATE OF THE MODULE
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the provided state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));

        // Calculate the desired FeedForward motor % from the current desired velocity and the static feedforward gains
        final double driveFF = m_driveFF.calculate(state.speedMetersPerSecond);

        // Set the reference state of the module
        m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity, 0, driveFF * 11.5);
        setReferenceAngle(state.angle.getRadians());
    }

    /**
     * SET REFERENCE ANGLE FOR TURN MOTOR
     *
     * @param referenceAngleRadians Desired reference angle.
     */
    public void setReferenceAngle(double referenceAngleRadians) {
        double currentAngleRadians = m_turnEncoder.getPosition();

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
        // that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        m_referenceAngleRadians = referenceAngleRadians;
        m_turnPIDController.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);
    }

    /**
     * GET REFERENCE ANGLE FOR TURN
     *
     * @return Reference angle
     */
    public double getReferenceAngle() {
        return m_referenceAngleRadians;
    }

    /**
     * GET STATE ANGLE FOR TURN
     *
     * @return state angle (radians)
     */
    public double getStateAngle() {
        double motorAngleRadians = m_turnEncoder.getPosition();
        
        // motorAngleRadians %= 2.0 * Math.PI;

        // // If the angle is less than zero than correct value to be an angle between 0 and 2pi
        // if (motorAngleRadians < 0.0) {
        //     motorAngleRadians += 2.0 * Math.PI;
        // }
        
        return motorAngleRadians;
    }

    /**
     * GET ABSOLUTE ENCODER ANGLE
     *
     * @return absolute angle (radians)
     */
    public double getAbsEncoder() {
        double absEnc = m_absEncoder.getAbsolutePosition().refresh().getValue() * 2 * Math.PI + offset;
        return absEnc;
    }

    /**
     * CHANGE DRIVE MOTOR IDLE MODE STATE
     *
     * @param brake mode enabled (boolean)
     */
    public void enableBrake(boolean brake) {
        if (brake) {
            m_driveMotor.setIdleMode(IdleMode.kBrake);
        } else {
            m_driveMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    /**
     * STOP MOTOR MOVEMENT
     */
    public void stop() {
        m_driveMotor.set(0.0);
        m_turnMotor.set(0.0);
    }

    /**
     * RESET DRIVE ENCODER POSITION
     */
    public void resetDriveEncoder() {
        m_driveEncoder.setPosition(0);
    }

    public double getTurnPosition() {
        return m_turnEncoder.getPosition();
    }
}
