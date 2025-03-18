package frc.robot.subsystems.elevatoreffector;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.HardwareMap.CAN;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOTalonFX implements WristIO {
    private TalonFX motor;
    public WristIOTalonFX() {
        motor = new TalonFX(CAN.EFFECTOR_WRIST_ID);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        WristIO.super.updateInputs(inputs);


    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        WristIO.super.setTargetAngle(angle);
    }
}
