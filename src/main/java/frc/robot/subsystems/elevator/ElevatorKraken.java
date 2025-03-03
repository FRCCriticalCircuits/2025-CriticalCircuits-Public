package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.DeviceID;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.TunedConstants;

public class ElevatorKraken implements ElevatorIO {
    private TalonFX m_leftMotor;
    private TalonFX m_rightMotor;
    private TalonFXConfiguration elevatorConfig;

    private double targetRotation = 0.0;

    private final MotionMagicVoltage m_MotionMagic = new MotionMagicVoltage(0).withSlot(0);

    public ElevatorKraken(){
        m_leftMotor = new TalonFX(DeviceID.Elevator.ELEVATOR_LEFT_ID);
        m_rightMotor = new TalonFX(DeviceID.Elevator.ELEVATOR_RIGHT_ID);

        elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = PhysicalConstants.Elevator.CurrentLimits.ELEVATOR_CURRENT_LIMIT;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        elevatorConfig.Feedback.SensorToMechanismRatio = 15.0;

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        elevatorConfig.withSlot0(
            new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKS(TunedConstants.Elevator.ELEVATOR_FEED_FORWARD_KS)
            .withKV(TunedConstants.Elevator.ELEVATOR_FEED_FORWARD_KV)
            .withKA(TunedConstants.Elevator.ELEVATOR_FEED_FORWARD_KA)
            .withKP(TunedConstants.Elevator.ELEVATOR_PID_P)             // Error Gain
            .withKI(TunedConstants.Elevator.ELEVATOR_PID_I)             // Error Intergral Gain
            .withKD(TunedConstants.Elevator.ELEVATOR_PID_D)             // Error Derivative Gain
        );

        elevatorConfig.Voltage.PeakForwardVoltage = 8;
        elevatorConfig.Voltage.PeakReverseVoltage = -8;

        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = TunedConstants.Elevator.ELEVATOR_MAX_VELOCITY;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = TunedConstants.Elevator.ELEVATOR_MAX_ACCELERATION; 
        m_rightMotor.getConfigurator().apply(elevatorConfig);

        elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftMotor.setControl(new Follower(m_rightMotor.getDeviceID(), true));
        m_leftMotor.getConfigurator().apply(elevatorConfig);

        m_rightMotor.setPosition(0);
        m_leftMotor.setPosition(0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = m_rightMotor.getPosition().getValueAsDouble();
        inputs.targetPosition = targetRotation;

        m_rightMotor.setControl(m_MotionMagic.withPosition(targetRotation));
    }

    @Override
    public void setPosition(double rotation) {
        this.targetRotation = rotation;
    }
}
