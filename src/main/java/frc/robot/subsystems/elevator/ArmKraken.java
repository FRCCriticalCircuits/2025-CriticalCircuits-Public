package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.DeviceID;
import frc.robot.Constants.PhysicalConstants.DriveBase.CurrentLimits;
import frc.robot.Constants.TunedConstants;

public class ArmKraken implements ArmIO {
    private TalonFX m_anglerMotor;
    private TalonFXConfiguration anglerConfig;
    private DutyCycleEncoder m_anglerEncoder;

    private final MotionMagicVoltage m_MotionMagic = new MotionMagicVoltage(0).withSlot(0);

    public ArmKraken(){
        m_anglerMotor = new TalonFX(DeviceID.Angler.ANGLER_ID);
        anglerConfig = new TalonFXConfiguration();

        m_anglerEncoder = new DutyCycleEncoder(DeviceID.Sensor.ANGLER_ENCODER, 1.0, 0);

        anglerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        anglerConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.ANGLER_CURRENT_LIMIT;
        anglerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        anglerConfig.CurrentLimits.SupplyCurrentLimit = 40;

        anglerConfig.Feedback.SensorToMechanismRatio = 35.0;

        anglerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        anglerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        anglerConfig.withSlot0(
            new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKS(TunedConstants.Arm.ARM_FEED_FORWARD_KS)
            .withKV(TunedConstants.Arm.ARM_FEED_FORWARD_KV)
            .withKA(TunedConstants.Arm.ARM_FEED_FORWARD_KA)
            .withKP(TunedConstants.Arm.ARM_PID_P)             // Error Gain
            .withKI(TunedConstants.Arm.ARM_PID_I)             // Error Intergral Gain
            .withKD(TunedConstants.Arm.ARM_PID_D)             // Error Derivative Gain
        );

        anglerConfig.Voltage.PeakForwardVoltage = 8;
        anglerConfig.Voltage.PeakReverseVoltage = -8;

        anglerConfig.MotionMagic.MotionMagicCruiseVelocity = TunedConstants.Arm.ARM_MAX_VELOCITY;
        anglerConfig.MotionMagic.MotionMagicAcceleration = TunedConstants.Arm.ARM_MAX_ACCELERATION;  

        m_anglerMotor.getConfigurator().apply(anglerConfig);

        m_anglerEncoder.setAssumedFrequency(975.609756); // 1s / 1025μs
        m_anglerEncoder.setDutyCycleRange(0.0010, 0.9990); // 1μs & 1024μs out of 1025μs

        m_anglerMotor.setPosition(m_anglerEncoder.get());

        m_MotionMagic.withPosition(m_anglerMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setPosition(double rotation) {
        m_anglerMotor.setControl(m_MotionMagic.withPosition(rotation));
    }
}
