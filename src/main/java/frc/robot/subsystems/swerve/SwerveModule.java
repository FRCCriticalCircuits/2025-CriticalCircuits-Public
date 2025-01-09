package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.utils.Conversions.WheelConversions;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.TunedConstants;


public class SwerveModule {
    public CANcoder canCoder;

    public TalonFX driveMotor;
    private TalonFXConfiguration driveConfig;

    public SparkMax turnMotor;
    private RelativeEncoder turnEncoder;
    private SparkMaxConfig turnConfig;
    private EncoderConfig encoderConfig;

    private PIDController turnPID;

    /*
     * Uses for control driveMotor with .setControl() method
     */
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);   // OpenLoop
    private VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0); // CloseLoop
    private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0, 0); // Feedforward for CloseLoop

    /**
     * @apiNote Constructor for SwerveModule used in {@link SwerveSubsystem}
     */
    public SwerveModule(int canCoderID, double CANCoderOffset, int driveID, boolean driveInverted, int turnID, boolean turnInverted){
        /* Configure CANCoder */
        canCoder = new CANcoder(canCoderID);

        canCoder.getConfigurator().apply(
            new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(
                    SensorDirectionValue.CounterClockwise_Positive
                ).withMagnetOffset(CANCoderOffset)
        );

        // BaseStatusSignal.setUpdateFrequencyForAll(100, canCoder.getAbsolutePosition());
        
        /* Configure Drive Motor */
        driveMotor = new TalonFX(driveID);

        /* Restore Factor Defualts */
        driveConfig = new TalonFXConfiguration();

        /* Current Limits */
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        driveConfig.CurrentLimits.StatorCurrentLimit = PhysicalConstants.DriveBase.CurrentLimits.DRIVE_CURRENT_LIMIT;

        /* Gear Ratio */
        driveConfig.Feedback.SensorToMechanismRatio = PhysicalConstants.DriveBase.GearRatios.DRIVE_GEAR_RATIO;

        /* OpenLoop Control Ramp Period*/
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = PhysicalConstants.DriveBase.CurrentLimits.DRIVE_LOOP_RAMP_RATE; // use with DutyCycleOut Control
        
        /* CloseLoop Control Ramp Period */
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0; // use with VelocityDutyCycle Control

        /* CloseLoop Constants, haven't tuned */
        driveConfig.Slot0 = new Slot0Configs()
            .withKP(TunedConstants.DriveBase.DRIVE_PID_P)
            .withKI(TunedConstants.DriveBase.DRIVE_PID_I);

        /* Motor Outputs */
        driveConfig.MotorOutput.Inverted = (driveInverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Feedforward used for VelocityDutyCycle Output */
        driveFeedForward = new SimpleMotorFeedforward(
            TunedConstants.DriveBase.DRIVE_FEED_FORWARD_KS,
            TunedConstants.DriveBase.DRIVE_FEED_FORWARD_KV,
            TunedConstants.DriveBase.DRIVE_FEED_FORWARD_KA
            // No need for kG because it's not arm/elevator
        );

        // BaseStatusSignal.setUpdateFrequencyForAll(60, driveMotor.getVelocity(), driveMotor.getPosition());

        /* Apply Settings */
        driveMotor.getConfigurator().apply(driveConfig);

        /* Configure Turn Motor */
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        turnEncoder = turnMotor.getEncoder();

        /* Create Configuration Objects */
        turnConfig = new SparkMaxConfig();
        encoderConfig = new EncoderConfig();

        /* Current Limits */
        turnConfig.voltageCompensation(PhysicalConstants.NOMINAL_VOLTAGE);
        turnConfig.smartCurrentLimit(PhysicalConstants.DriveBase.CurrentLimits.TURN_CURRENT_LIMIT);
        
        /* Gear Ratio */
        encoderConfig.positionConversionFactor((1.0 / PhysicalConstants.DriveBase.GearRatios.TURN_GEAR_RATIO) * Math.PI * 2);
        encoderConfig.velocityConversionFactor((60.0 / PhysicalConstants.DriveBase.GearRatios.TURN_GEAR_RATIO) * Math.PI * 2);

        /* PID Constructor */
        turnPID = new PIDController(
            TunedConstants.DriveBase.TURN_PID_P,
            TunedConstants.DriveBase.TURN_PID_I,
            TunedConstants.DriveBase.TURN_PID_D    
        );

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        /* Motor Outputs */
        turnConfig.idleMode(IdleMode.kBrake);
        turnConfig.inverted(turnInverted);

        /* Apply Settings */
        turnConfig.apply(encoderConfig);
        turnMotor.configure(
            turnConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * get the absolute angle of CANCoder, with an delay of 200ms
     * @return the angle in Radians
     */
    private double getAbsoluteAngleRad() {
        return canCoder.getAbsolutePosition().waitForUpdate(0.25).getValueAsDouble() * Math.PI * 2; 
    }

    /**
     * get the position of Wheel
     * @return distance in meters
     */
    private double getDriveDistance() {
        return WheelConversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(),  PhysicalConstants.DriveBase.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
    }

    /**
     * get the velocity of Wheel
     * @return velocity in meters per second
     */
    private double getDriveVelocity() {
        return WheelConversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), PhysicalConstants.DriveBase.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
    }

    /**
     * get the postion returns from turn
     * @return angle in radians
     */
    private double getTurnAngleRad(){
        return turnEncoder.getPosition();
    }

    /**
     * get the angle returns from turn's encoder
     * @return angle in {@link Rotation2d}
     */
    private Rotation2d getTurnRotation2D() {
        return Rotation2d.fromRadians(getTurnAngleRad());
    }

    /**
     * get the position(distane and angle) of Swerve Module
     * @return position in {@link SwerveModulePosition} 
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            getDriveDistance(),
            getTurnRotation2D()
        );
    }

    /**
     * get the state(velocity and angle) of Swerve Module
     * @return state in {@link SwerveModulePosition} 
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            getTurnRotation2D()
        );
    }

    /**
     * Uses for SysID tuning
     * @param voltage voltage output
     */
    public void setDriveVoltage(double voltage){
        driveMotor.setControl(new VoltageOut(voltage));
    }

    /**
     * set the speed for perpotion
     * @param speed the optimized {@link SwerveModuleState}'s speed
     * @param isDutyCycle true for openloop {@link DutyCycleOut} Control, false for {@link VelocityDutyCycle}
     */
    public void setDrive(double speed, boolean openLoop) {
        if(openLoop){
            dutyCycle.Output = speed / PhysicalConstants.DriveBase.MAX_SPEED_METERS;
            driveMotor.setControl(dutyCycle);
        }else{
            velocityDutyCycle.Velocity = WheelConversions.MPSToRPS(speed, PhysicalConstants.DriveBase.LENGTHS.DRIVE_WHEEL_CIRCUMFERENCE);
            velocityDutyCycle.FeedForward = driveFeedForward.calculateWithVelocities(getDriveVelocity(), speed);

            driveMotor.setControl(velocityDutyCycle);
        }
    }

    /**
     * set the angle for turn
     * @param desireAngle the optimized {@link SwerveModuleState}'s angle in radians
     */
    public void setTurn(double desireAngle){
        turnMotor.set(turnPID.calculate(getTurnAngleRad(), desireAngle));
    }

    /**
     * set the state of {@link SwerveModule}
     * @param desireState the desired {@link SwerveModuleState} 
     * @param isDutyCycle control mode, duty cycle is true, velocity is false
     */
    public void setState(SwerveModuleState desireState, boolean openLoop){
        /* Stop Module if speed < 1% */
        if(Math.abs(desireState.speedMetersPerSecond) < 0.05){
            stopModule();
            return;
        }

        /*Optimization and Cosine compensation for drive */
        Rotation2d currnetAngle = getModuleState().angle;
        desireState.optimize(currnetAngle);
        desireState.speedMetersPerSecond *= desireState.angle.minus(currnetAngle).getCos();

        setDrive(desireState.speedMetersPerSecond, openLoop);
        setTurn(desireState.angle.getRadians());
    }

    /**
     * reset position of encoders to default value
     * @apiNote drive to 0, turn to CANCoder's position
     */
    public void resetEncoders(){
        driveMotor.getConfigurator().setPosition(0.0); // not used in openloop
        turnEncoder.setPosition(getAbsoluteAngleRad());
    }

    /**
     * Stop motors of SwerveModule
     */
    public void stopModule(){
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
