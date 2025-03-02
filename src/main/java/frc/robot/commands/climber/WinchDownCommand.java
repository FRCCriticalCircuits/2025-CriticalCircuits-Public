package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.climber.WinchSubsystem;
import frc.robot.utils.structures.DataStrcutures.Level;

public class WinchDownCommand extends Command {
    WinchSubsystem winchSubsystem;
    boolean dir;
    public WinchDownCommand(WinchSubsystem s) {
        winchSubsystem = s;
        addRequirements(s);
    }

    @Override
    public void execute() {
        winchSubsystem.winch(1);
        AutoAimManager.getInstance().updateLevel(Level.LClimb);
    }

    @Override
    public void end(boolean interrupted) {
        winchSubsystem.winch(0);
    }
}