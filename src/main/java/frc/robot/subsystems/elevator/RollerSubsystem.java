package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerMode;
import frc.robot.utils.structures.DataStrcutures;

import java.util.function.Supplier;

public class RollerSubsystem extends SubsystemBase {
    public boolean lowVoltage = false;

    private RollerIO rollerIO;
    private RollerIOInputs inputs = new RollerIOInputs();

    public RollerSubsystem(Robot r) {
        if (Robot.isReal()) {
            this.rollerIO = new RollerKraken(r);
        } else {
            this.rollerIO = new RollerSim();
        }
    }

    public void overrideSimStates(RollerIOInputs desireInputs) {
        inputs = desireInputs;
        rollerIO.overrideStates(desireInputs);
    }

    public Boolean hasAlgae() {
        return inputs.algaeDetected;
    }

    public Boolean hasCoral() {
        return inputs.coralDetected;
    }

    public void setMode(RollerMode mode) {
        rollerIO.setMode(mode);
    }

    public Command intakeCoralCommand() {
        return startEnd(
                () -> setMode(RollerIO.RollerMode.CORAL_IN),
                () -> setMode(RollerIO.RollerMode.HOLD)
        );
    }

    public Command intakeAlgaeCommand() {
        return startEnd(
                () -> setMode(RollerIO.RollerMode.ALGAE_IN),
                () -> setMode(RollerIO.RollerMode.HOLD)
        );
    }

    public Command outtakeCoralCommand(Supplier<DataStrcutures.Level> l) {
        return startEnd(
                () -> {
                    if (l.get() == DataStrcutures.Level.L1) {
                        setMode(RollerMode.CORAL_OUT_LIGHT);
                    } else {
                        setMode(RollerMode.CORAL_OUT);
                    }
                },
                () -> setMode(RollerMode.HOLD)
        );
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(inputs, lowVoltage);

        // SmartDashboard.putBoolean("HasCoral", coralDetected());
    }
}
