package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    /* The motor used for driving the wheel on the module */
    private CANSparkFlex m_driveMotor;

    /* The motor used for turn the wheel on the module */
    private CANSparkFlex m_turnMotor;

    /* The encoder that keeps track of driving distance */
    private RelativeEncoder m_driveEncoder;

    /* The encoder that keeps track of the angle of the wheel on the module */
    private RelativeEncoder m_turnEncoder;
    private CANcoder m_turnCANcoder; // Perferably use this one

    private CANcoderConfiguration m_turnCANcoderConfig;

    /**
     * Spark Library PID Controller is great if we were using a Rev Robotics
     * absolute encoder for measuring the wheel angle. It is very difficult
     * to use a cancoder with the Spark PID Controller because it doesnt
     * directly take the cancoder object as a feedback device parameter
     * you haver to do a work around were you set the position of the turn
     * angle to the position of the cancoder constantly.
     */
    private SparkPIDController m_drivePIDController;
    private SparkPIDController m_turnPIDController;

    /* Feedforward Controller for the Drive Motor on the module */
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        Constants.SwerveModule.kDriveS, 
        Constants.SwerveModule.kDriveV
    );

    /* Set the PID output to zero for the drive on startup */
    private double drivePIDControllerOutput = 0;
    private double driveFFControllerOutput = 0;
    private double driveMotorVel = 0;
    private double driveMotorVelCF = 0;

    /**
     * The offset from the zero point of the CANcoder at startup needs
     * to be added to the measurement. This value is in radians. The
     * zero point is the when the wheel is facing parallel to the side
     * frame with the black bolt facing the outside of the robot.
     */
    private double turnCANcoderOffset;

    public String modName;


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

        /* Create and configure the drive motor and related objects */
        m_driveMotor = new CANSparkFlex(driveCANid, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults(); // Reset All Settings
        m_driveMotor.setInverted(driveMotorInvert);
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setSmartCurrentLimit(Constants.SwerveModule.kDriveMotorCurrentLimit);
        // m_driveMotor.enableVoltageCompensation()
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 500);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);
        
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setInverted(driveEncInvert);
        /* Conversion factor on the drive encoder. The values for position and velocity
         * should be in meters and meters/second. The native units of the encoder are
         * in rotations and RPM. */
        m_driveEncoder.setPositionConversionFactor(Constants.SwerveModule.kDriveEncoderPositionFactor);
        m_driveEncoder.setVelocityConversionFactor(Constants.SwerveModule.kDriveEncoderVelocityFactor);
        m_driveEncoder.setPosition(0.0);
        
        m_drivePIDController = m_driveMotor.getPIDController();
        m_drivePIDController.setP(Constants.SwerveModule.kDriveP);
        // m_drivePIDController.setI(Constants.SwerveModule.kDriveI);
        // m_drivePIDController.setD(Constants.SwerveModule.kDriveD);
        // m_drivePIDController.setFF(Constants.SwerveModule.kDriveFF);
        
        /* Burn the configurations to the flash memory of the motor */
        m_driveMotor.burnFlash();



        /* Create and configure the turn motor and related objects */
        m_turnMotor = new CANSparkFlex(turnCANid, MotorType.kBrushless);
        m_turnMotor.restoreFactoryDefaults();
        m_turnMotor.setInverted(turnMotorInvert);
        m_turnMotor.setIdleMode(IdleMode.kBrake);
        m_turnMotor.setSmartCurrentLimit(Constants.SwerveModule.kTurnMotorCurrentLimit);
        // m_turnMotor.enableVoltageCompensation();
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 500);
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        m_turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);       



        /* Setting up the CANcoder is a bit messy. The way that the Phoenix library is set up
         * doesn't really make sense. Why would I need to call a get function to set something? */
        m_turnCANcoder = new CANcoder(coderCANid);
        m_turnCANcoder.getConfigurator().apply(new CANcoderConfiguration());
        m_turnCANcoderConfig = new CANcoderConfiguration();
        m_turnCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        m_turnCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        m_turnCANcoder.getPosition().setUpdateFrequency(10);
        m_turnCANcoder.getPosition().setUpdateFrequency(10);
        m_turnCANcoder.getConfigurator().apply(m_turnCANcoderConfig);

        /* 
         * Set up the encoders to be the built-in 3 phase hall effect sensors.
         * It is very important that the angle encoder is set up to be an
         * absolute position so that at startup it won't matter what position
         * the wheel are in and the software will just know what direction the
         * wheels are facing.
        */
        m_turnEncoder = m_turnMotor.getEncoder();
        m_turnEncoder.setPositionConversionFactor(Constants.SwerveModule.kDriveEncoderPositionFactor);
        m_turnEncoder.setVelocityConversionFactor(Constants.SwerveModule.kDriveEncoderVelocityFactor);
        m_turnEncoder.setMeasurementPeriod(16);
        m_turnEncoder.setAverageDepth(4);
        
        m_turnPIDController = m_turnMotor.getPIDController();
        m_turnPIDController.setP(Constants.SwerveModule.kTurnP);

        m_turnMotor.burnFlash();
    }



    /**
     * Gets the positonal state of the swerve module using the position
     * and velocity.
     * 
     * @return the positional state of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), getWheelAngle());
    }

    /**
     * Get the wheel angle as read by the integrated motor encoder. The integrated encoder
     * is set up to be relative and can have it's position set based on the absolute
     * reading of the CANcoder.
     * 
     * @return the wheel angle (relative encoder)
     */
    public Rotation2d getWheelAngle() {
        return Rotation2d.fromRadians(m_turnEncoder.getPosition());
    }

    /**
     * Get the wheel angle as read by the CANcoder mounted on the center axis of the
     * swerve module. The mounted encoder is configured to read the Absolute reading
     * which is prefered for measuring wheel angle since it will range between 0 and
     * 360 degrees based on the magnetic north pole of the magnet relative to the
     * sensor.
     * 
     * @return the absolute encoder reading of the CANcoder
     */
    public Rotation2d getCANcoderAngle() {
        return Rotation2d.fromRadians(m_turnCANcoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Get the current angle of the wheel from the CANcoder in terms of degrees.
     * 
     * @return angle of wheel in degrees from [0 to 360)
     */
    public double getAngleDegrees() {
        double angle = getCANcoderAngle();
        return Math.toDegrees(*2*Math.PI());
    }

    /**
     * Get the State of the swerve module.
     * 
     * @return the state of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), getWheelAngle());
    }

    /**
     * Get the drive encoder position.
     * 
     * @return position of the drive encoder
     */
    public double getDriveEncoderPosition() {
        return m_driveEncoder.getPosition();
    }

    /**
     * Set the individual rotational position of the swerve module using the simple
     * PID controller to an angle in radians.
     * 
     * @param angleRad
     */
    public void setRotation(double angleRad) {
        m_turnMotor.set(
            m_turnPIDControllerSimple.calculate(
                getWheelAngle().getRadians(),
                angleRad
        ));
    }

    /**
     * Sets the PID controller setpoints to the desired state. Defaults to an open
     * loop control that is optimized.
     * 
     * @param state the desired state of the module
     */
    public void setState(SwerveModuleState state) {
        setState(state, true, true);
    }

    /**
     * Sets the PID controller setpoints to the desired state.
     * 
     * @param state the desired state of the module
     * @param openLoop run open or closed loop
     * @param optimize optimize the state of the module
     */
    public void setState(SwerveModuleState state, boolean openLoop, boolean optimize) {
        /* If the state of the module is less than the speed threshold then force the
        motors to stop so there is no subtle drifting. Exit the function afterwards. */
        if (state.speedMetersPerSecond < Constants.SwerveModule.kSpeedDeadband) {
            stop();
            return;
        }

        if (optimize) {
            state = SwerveModuleState.optimize(
                state, 
                getWheelAngle()
            );
        }

        if (openLoop) {
            m_driveMotor.setVoltage(state.speedMetersPerSecond / Constants.Swerve.kMaxDriveMeterPerSec);
            double v = m_turnPIDControllerSimple.calculate(getWheelAngle().getRadians(), state.angle.getRadians()) * 12;
            if (v > 8) v = 8.0;
            m_turnMotor.setVoltage(v);
        } else {
            this.drivePIDControllerOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
            this.driveFFControllerOutput = driveFeedforward.calculate(state.speedMetersPerSecond);
            this.driveMotorVel = m_driveEncoder.getVelocity();
            this.driveMotorVelCF = m_driveEncoder.getVelocityConversionFactor();

            double turnOutput = m_turnPIDController.calculate(getWheelAngle().getRadians(), state.angle.getRadians());

            m_driveMotor.setVoltage(drivePIDControllerOutput + driveFFControllerOutput);
            m_turnMotor.set(turnOutput);
        }
    }

    /**
     * Set the drive motor speed independently of everything else
     * for troubleshooting purposes.
     * 
     * @param speed of drive motor in terms of percent output
     */
    public void setDriveSpeed(double speed) {
        m_driveMotor.set(speed);
    }

    public double getTurningVelocity() {
        return m_turnEncoder.getVelocity();
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity();
    }

    /**
     * Resets the integrated turn motor encoder to the positon of the CANcoder absolute
     * reading.
     */
    public void resetToAbsolute() {
        double absolutePosition = getCANcoderAngle().getRadians() - turnCANcoderOffset;
        m_turnEncoder.setPosition(absolutePosition);
    }

    /**
     * Reset the positon of the drive encoder to zero.
     */
    public void resetDriveEncoder() {
        m_driveEncoder.setPosition(0);
    }

    /**
     * Stop Motors to essentially force stop the whole module from moving
     */
    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
    }

    public double getRawEncoder() {
        return m_turnCANcoder.getAbsolutePosition().getValueAsDouble();
    }
}
