package frc.robot.subsystems.elevatoreffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.constants.HardwareMap;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leftLeader;
    private final TalonFX rightFollower;

    public ElevatorIOTalonFX() {
        leftLeader = new TalonFX(HardwareMap.CAN.ELEVATOR_LEFT_ID);
        rightFollower = new TalonFX(HardwareMap.CAN.ELEVATOR_RIGHT_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.CURRENT_LIMIT_STATOR;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.CURRENT_LIMIT_SUPPLY;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.SENSOR_TO_MECHANISM_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        ElevatorIO.super.updateInputs(inputs);

    }

    @Override
    public void setTargetPosition() {
        ElevatorIO.super.setTargetPosition();
    }
}
