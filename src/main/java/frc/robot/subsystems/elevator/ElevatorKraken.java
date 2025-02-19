package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorKraken implements ElevatorIO {
    private TalonFX m_leftMotor;
    private TalonFX m_rightMotor;
    private TalonFXConfiguration elevatorConfig;

    private final MotionMagicVoltage m_MotionMagic = new MotionMagicVoltage(0).withSlot(0);

    public ElevatorKraken(){
        m_leftMotor = new TalonFX(21);
        m_rightMotor = new TalonFX(22);

        elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = 20;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        elevatorConfig.Feedback.SensorToMechanismRatio = 15.0;

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        elevatorConfig.withSlot0(
            new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKS(0.16)
            .withKV(1.6)
            .withKA(0.12)
            .withKP(20.0)           // Error Gain
            .withKI(0.0)            // Error Intergral Gain
            .withKD(0.0)            // Error Derivative Gain
        );

        elevatorConfig.Voltage.PeakForwardVoltage = 8;
        elevatorConfig.Voltage.PeakReverseVoltage = -8;

        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 6.0; // 4.5  r/s
        elevatorConfig.MotionMagic.MotionMagicAcceleration = 32.0;  // 10 r/s^2
        m_rightMotor.getConfigurator().apply(elevatorConfig);

        elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftMotor.setControl(new Follower(m_rightMotor.getDeviceID(), true));
        m_leftMotor.getConfigurator().apply(elevatorConfig);

        m_rightMotor.setPosition(0);
        m_leftMotor.setPosition(0);
    }

    @Override
    public void setPosition(double rotation) {
        m_rightMotor.setControl(m_MotionMagic.withPosition(rotation));
    }
}
