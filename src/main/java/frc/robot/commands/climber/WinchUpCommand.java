package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.climber.WinchSubsystem;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.climber.WinchIO.WINCH_STATES;
import frc.robot.subsystems.elevatoreffector.ElevatorSubsystem2;
import frc.robot.utils.structures.DataStrcutures;
import frc.robot.utils.structures.DataStrcutures.Level;

public class WinchUpCommand extends Command {
    private final ElevatorSubsystem2 elevatorSubsystem;
    private WinchSubsystem winchSubsystem;
    private RollerSubsystem rollerSubsystem;

    public WinchUpCommand(WinchSubsystem winchSubsystem, RollerSubsystem r, ElevatorSubsystem2 e) {
        this.elevatorSubsystem = e;
        this.winchSubsystem = winchSubsystem;
        this.rollerSubsystem = r;
        addRequirements(winchSubsystem, rollerSubsystem);
    }

    @Override
    public void execute() {
        if (!rollerSubsystem.hasCoral()) {
            elevatorSubsystem.setMode(DataStrcutures.Mode.CLIMB);
            winchSubsystem.setState(WINCH_STATES.UP);

        }
    }

    @Override
    public boolean isFinished() {
        // Don't run command if coral present
        return rollerSubsystem.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        winchSubsystem.setState(WINCH_STATES.IDLE);
    }
}