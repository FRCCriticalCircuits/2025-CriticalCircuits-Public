package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.climber.WinchSubsystem;
import frc.robot.subsystems.climber.WinchIO.WINCH_STATES;
import frc.robot.utils.structures.DataStrcutures.Level;

public class WinchUpCommand extends Command {
    WinchSubsystem winchSubsystem;

    public WinchUpCommand(WinchSubsystem winchSubsystem) {
        this.winchSubsystem = winchSubsystem;
        addRequirements(winchSubsystem);
    }

    @Override
    public void execute() {
        winchSubsystem.setState(WINCH_STATES.UP);
        AutoAimManager.getInstance().updateLevel(Level.LClimb);
    }

    @Override
    public void end(boolean interrupted) {
        winchSubsystem.setState(WINCH_STATES.IDLE);
    }
}