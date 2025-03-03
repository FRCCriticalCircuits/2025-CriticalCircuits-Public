package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.DeviceID;
import frc.robot.Constants.Physical;
import frc.robot.Constants.TunedConstants;

public class ArmKraken implements ArmIO {
    private TalonFX m_anglerMotor;
    private TalonFXConfiguration anglerConfig;
    private DutyCycleEncoder m_anglerEncoder;

    private Rotation2d targetIORotation = Rotation2d.fromDegrees(55);

    private final PositionVoltage m_PositionVoltage_0 = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage m_PositionVoltage_1 = new PositionVoltage(0).withSlot(1);
    private final PositionVoltage m_PositionVoltage_2 = new PositionVoltage(0).withSlot(2);

    public ArmKraken(){
        m_anglerMotor = new TalonFX(DeviceID.Angler.ANGLER_ID);
        anglerConfig = new TalonFXConfiguration();

        m_anglerEncoder = new DutyCycleEncoder(DeviceID.Sensor.ANGLER_ENCODER, 1.0, Physical.Arm.ENCODER_ZERO_OFFSET);

        anglerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        anglerConfig.CurrentLimits.StatorCurrentLimit = Physical.Elevator.CurrentLimits.ANGLER_CURRENT_LIMIT;
        anglerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        anglerConfig.CurrentLimits.SupplyCurrentLimit = 40;

        anglerConfig.Feedback.SensorToMechanismRatio = 35.0;

        anglerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        anglerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        anglerConfig.withSlot0(
            new Slot0Configs()
            .withKP(TunedConstants.Arm.ARM_PID_P)             // Error Gain
            .withKI(TunedConstants.Arm.ARM_PID_I)             // Error Intergral Gain
            .withKD(TunedConstants.Arm.ARM_PID_D)             // Error Derivative Gain
        );

        anglerConfig.withSlot1(
            new Slot1Configs()
            .withKP(TunedConstants.Arm.ARM_PID_P_CORAL)             // Error Gain
            .withKI(TunedConstants.Arm.ARM_PID_I_CORAL)             // Error Intergral Gain
            .withKD(TunedConstants.Arm.ARM_PID_D_CORAL)             // Error Derivative Gain
        );

        anglerConfig.withSlot2(
            new Slot2Configs()
            .withKP(TunedConstants.Arm.ARM_PID_P_ALGAE)             // Error Gain
            .withKI(TunedConstants.Arm.ARM_PID_I_ALGAE)             // Error Intergral Gain
            .withKD(TunedConstants.Arm.ARM_PID_D_ALGAE)             // Error Derivative Gain
        );

        anglerConfig.Voltage.PeakForwardVoltage = 8;
        anglerConfig.Voltage.PeakReverseVoltage = -8;

        anglerConfig.MotionMagic.MotionMagicCruiseVelocity = TunedConstants.Arm.ARM_MAX_VELOCITY;
        anglerConfig.MotionMagic.MotionMagicAcceleration = TunedConstants.Arm.ARM_MAX_ACCELERATION;  

        m_anglerMotor.getConfigurator().apply(anglerConfig);

        m_anglerEncoder.setAssumedFrequency(975.609756); // 1s / 1025μs
        m_anglerEncoder.setDutyCycleRange(0.0010, 0.9990); // 1μs & 1024μs out of 1025μs
        m_anglerEncoder.setInverted(true);

        m_anglerMotor.setPosition(encoderConversion(m_anglerEncoder.get()));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs, boolean coralDetected, boolean algaeDetected) {
        inputs.ioRotation = Rotation2d.fromRotations(m_anglerMotor.getPosition().getValueAsDouble());
        inputs.targetRotation = this.targetIORotation;

        if(coralDetected) m_anglerMotor.setControl(m_PositionVoltage_1.withPosition(this.targetIORotation.getRotations()));
        else if (algaeDetected) m_anglerMotor.setControl(m_PositionVoltage_2.withPosition(this.targetIORotation.getRotations()));
        else m_anglerMotor.setControl(m_PositionVoltage_0.withPosition(this.targetIORotation.getRotations()));
    }

    /**
     * Converts encoder rotation from [0, 1] to [-0.5, 0.5)
     * @param rotation encoder rotation
     * @return converted encoder rotation
     */
    private double encoderConversion(double rotation){
        return (rotation > 0.5) ? rotation - 1.0 : rotation;
    }

    @Override
    public void setRotation(Rotation2d ioRotation) {
        this.targetIORotation = ioRotation;
    }
}
