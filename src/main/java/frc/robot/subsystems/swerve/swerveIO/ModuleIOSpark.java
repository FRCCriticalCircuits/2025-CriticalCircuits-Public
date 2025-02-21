package frc.robot.subsystems.swerve.swerveIO;

import static frc.robot.utils.MotorUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DeviceID;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.TunedConstants;

import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
    // Hardwares
    private CANcoder canCoder;
    private TalonFX driveMotor;
    private TalonFXConfiguration driveConfig;

    private SparkMax turnMotor;
    private RelativeEncoder turnEncoder;

    private SparkMaxConfig turnConfig;
    private EncoderConfig encoderConfig;

    private PIDController turnPID;

    /*
     * Uses for control driveMotor with .setControl() method
     */
    private VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0); // CloseLoop
    private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0, 0); // CloseLoop Feedforward

    // Queue inputs from odometry thread
    private Queue<Double> timestampQueue;
    private Queue<Double> drivePositionQueue;
    private Queue<Double> turnPositionQueue;

    // Connection debouncers
    private Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private Debouncer turnConnectedDebounce = new Debouncer(0.5);

    // StatusSignal from drive motor
    private StatusSignal<Angle> drivePosition;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<Voltage> driveAppliedVolts;
    private StatusSignal<Current> driveCurrent;

    // Constants
    private double zeroRotation;
    private int canCoderID, driveID, turnID;
    private boolean driveInverted;

    public ModuleIOSpark(int module) {
        zeroRotation = switch (module) {
            case 0 -> PhysicalConstants.DriveBase.CANCoder.FRONT_LEFT_OFFSET;
            case 1 -> PhysicalConstants.DriveBase.CANCoder.FRONT_RIGHT_OFFSET;
            case 2 -> PhysicalConstants.DriveBase.CANCoder.REAR_LEFT_OFFSET;
            case 3 -> PhysicalConstants.DriveBase.CANCoder.REAR_RIGHT_OFFSET;
            default -> 0;
        };

        canCoderID = switch (module) {
            case 0 -> DeviceID.DriveBase.FRONT_LEFT_CANCODER_ID;
            case 1 -> DeviceID.DriveBase.FRONT_RIGHT_CANCODER_ID;
            case 2 -> DeviceID.DriveBase.REAR_LEFT_CANCODER_ID;
            case 3 -> DeviceID.DriveBase.REAR_RIGHT_CANCODER_ID;
            default -> 0;
        };

        driveID = switch (module) {
            case 0 -> DeviceID.DriveBase.FRONT_LEFT_DRIVE_ID;
            case 1 -> DeviceID.DriveBase.FRONT_RIGHT_DRIVE_ID;
            case 2 -> DeviceID.DriveBase.REAR_LEFT_DRIVE_ID;
            case 3 -> DeviceID.DriveBase.REAR_RIGHT_DRIVE_ID;
            default -> 0;
        };

        driveInverted = switch (module) {
            case 0 -> true;
            case 1 -> false;
            case 2 -> true;
            case 3 -> false;
            default -> false;
        };

        turnID = switch (module) {
            case 0 -> DeviceID.DriveBase.FRONT_LEFT_TURN_ID;
            case 1 -> DeviceID.DriveBase.FRONT_RIGHT_TURN_ID;
            case 2 -> DeviceID.DriveBase.REAR_LEFT_TURN_ID;
            case 3 -> DeviceID.DriveBase.REAR_RIGHT_TURN_ID;
            default -> 0;
        };

        /* Configure CANCoder */
        canCoder = new CANcoder(canCoderID);

        canCoder.getConfigurator().apply(
            new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(
                    SensorDirectionValue.CounterClockwise_Positive
                ).withMagnetOffset(zeroRotation)
        );

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
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = PhysicalConstants.DriveBase.CurrentLimits.DRIVE_LOOP_RAMP_RATE;
        
        /* CloseLoop Control Ramp Period */
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

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
        turnConfig.inverted(true);

        /* Apply Settings */
        turnConfig.apply(encoderConfig);
        turnMotor.configure(
            turnConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Create drive status signals
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(PhysicalConstants.ODOMETRY_FREQUENCY, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor);

        driveMotor.setPosition(0);
        turnEncoder.setPosition(canCoder.getAbsolutePosition().waitForUpdate(0.25).getValueAsDouble() * Math.PI * 2);

        // Create odometry queues
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(() -> Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble()));
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnMotor,
                turnEncoder::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value));
        ifOk(turnMotor, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
            turnMotor,
                new DoubleSupplier[] {turnMotor::getAppliedOutput, turnMotor::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault); 

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value))
                .toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setControl(new VoltageOut(output));
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnMotor.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        velocityDutyCycle.Velocity = Units.radiansToRotations(velocityRadPerSec);
        velocityDutyCycle.FeedForward = driveFeedForward.calculateWithVelocities(driveVelocity.getValueAsDouble(), velocityDutyCycle.Velocity);
        driveMotor.setControl(velocityDutyCycle);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnMotor.set(turnPID.calculate(turnEncoder.getPosition(), rotation.getRadians()));
    }
}