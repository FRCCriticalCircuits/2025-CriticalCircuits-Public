package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.swerve.TempDriveOffsetCommand;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerMode;
import frc.robot.subsystems.elevatoreffector.ElevatorSubsystem2;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.utils.structures.DataStrcutures;

import java.util.function.Supplier;

public class RollerSubsystem extends SubsystemBase {
    public boolean lowVoltage = false;

    private final ElevatorSubsystem2 elevatorSubsystem;
    private boolean sameCoral = false;

    private RollerIO rollerIO;
    private RollerIOInputs inputs = new RollerIOInputs();

    public RollerSubsystem(Robot r, ElevatorSubsystem2 e) {
        if (Robot.isReal()) {
            this.rollerIO = new RollerKraken(r, e);
        } else {
            this.rollerIO = new RollerSim();
        }

        this.elevatorSubsystem = e;
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

    @Override
    public void periodic() {
        rollerIO.updateInputs(inputs, lowVoltage);

        if (inputs.coralDetected
                && elevatorSubsystem.getMode() == DataStrcutures.Mode.CORAL_INTAKE
                && DriverStation.isTeleop()) {
            if (!sameCoral) {
                sameCoral = true;
                // Move the robot back and change the elevator state automatically
                new TempDriveOffsetCommand(0, -1).withTimeout(0.25).andThen(
                        new InstantCommand(() -> {
                            // Change the mode back to coral placement
                            elevatorSubsystem.setMode(DataStrcutures.Mode.CORAL_PLACE);
                        })
                ).schedule();
            }
            LEDSubsystem.getInstance().setBlink(true);
        } else {
            sameCoral = false;
            LEDSubsystem.getInstance().setBlink(false);
        }
        // SmartDashboard.putBoolean("HasCoral", coralDetected());
    }

    public Command intakeCoralCommand() {
        return startEnd(
                () -> setMode(RollerIO.RollerMode.CORAL_IN),
                () -> setMode(RollerIO.RollerMode.HOLD_CORAL)
        );
    }

    public Command intakeAlgaeCommand() {
        return startEnd(
                () -> setMode(RollerIO.RollerMode.ALGAE_IN),
                () -> setMode(RollerIO.RollerMode.HOLD_CORAL)
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
                () -> setMode(RollerMode.HOLD_CORAL)
        );
    }
}
