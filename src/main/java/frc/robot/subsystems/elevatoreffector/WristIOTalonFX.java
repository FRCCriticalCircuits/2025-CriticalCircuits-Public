package frc.robot.subsystems.elevatoreffector;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.HardwareMap.CAN;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Degrees;

public class WristIOTalonFX implements WristIO {

    private final TalonFX motor = new TalonFX(CAN.EFFECTOR_WRIST_ID);;
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private int slot = 0;
    private Angle targetAngle = Degrees.of(0);
    private StatusSignal<Angle> currentAngleSignal = motor.getPosition();
    public WristIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.CURRENT_LIMIT_STATOR;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.CURRENT_LIMIT_SUPPLY;
        config.Feedback.SensorToMechanismRatio = WristConstants.SENSOR_TO_MECHANISM_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = WristConstants.INVERTED;

        config.Slot0.kP = WristConstants.PID.Nothing.kP;
        config.Slot0.kI = WristConstants.PID.Nothing.kI;
        config.Slot0.kD = WristConstants.PID.Nothing.kD;

        config.Slot1.kP = WristConstants.PID.Coral.kP;
        config.Slot1.kI = WristConstants.PID.Coral.kI;
        config.Slot1.kD = WristConstants.PID.Coral.kD;

        config.Slot2.kP = WristConstants.PID.Algae.kP;
        config.Slot2.kI = WristConstants.PID.Algae.kI;
        config.Slot2.kD = WristConstants.PID.Algae.kD;

        StatusCode status = motor.getConfigurator().apply(config, 0.25);
        // Show error if failed to init
        if (status != StatusCode.OK) {
            new Alert("Failed to configure TalonFX: " + this.getClass().getSimpleName(),
                    Alert.AlertType.kError).set(true);
        }

        // Assume motor starts at zero position
        motor.setPosition(WristConstants.ZERO_POSITION);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.currentRotation = currentAngleSignal.getValue();
        inputs.targetRotation = targetAngle;
    }

    @Override
    public void setTargetAngle(Angle angle) {
        targetAngle = angle;

        motor.setControl(motionMagic.withSlot(slot).withPosition(targetAngle));
    }

    @Override
    public void setMode(WristIOState state) {
        switch (state) {
            case NOTHING -> slot = 0;
            case CORAL -> slot = 1;
            case ALGAE -> slot = 2;
        }
    }
}
