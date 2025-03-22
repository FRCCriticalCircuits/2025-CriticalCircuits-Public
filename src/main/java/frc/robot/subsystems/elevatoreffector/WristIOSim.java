package frc.robot.subsystems.elevatoreffector;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.HardwareMap.CAN;

import static edu.wpi.first.units.Units.Degrees;

public class WristIOSim implements WristIO {

    private int slot = 0;
    private Angle targetAngle = Degrees.of(0);
    public WristIOSim() {

    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.currentRotation = Degrees.of(0);
        inputs.targetRotation = targetAngle;
    }

    @Override
    public void setTargetAngle(Angle angle) {
        targetAngle = angle;
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
