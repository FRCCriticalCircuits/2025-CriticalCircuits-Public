package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.climber.WinchIO.WINCH_STATES;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.climber.WinchSubsystem;
import frc.robot.utils.structures.DataStrcutures.Level;

public class WinchDownCommand extends Command {
    private WinchSubsystem winchSubsystem;
    private RollerSubsystem rollerSubsystem;

    public WinchDownCommand(WinchSubsystem winchSubsystem, RollerSubsystem r) {
        this.winchSubsystem = winchSubsystem;
        this.rollerSubsystem = r;
        addRequirements(winchSubsystem, rollerSubsystem);
    }

    @Override
    public void execute() {
        if (!rollerSubsystem.hasCoral()) {
            AutoAimManager.getInstance().updateLevel(Level.LClimb);
            winchSubsystem.setState(WINCH_STATES.DOWN);
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