package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerMode;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class Shoot extends Command{
    private RollerSubsystem rollerSubsystem;

    public Shoot(){
        rollerSubsystem = RollerSubsystem.getInstance();

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize(){
        rollerSubsystem.lowVoltage = (AutoAimManager.getInstance().getSetting().getLevel() == Level.L1) ? true : false;
        rollerSubsystem.set(RollerMode.OUT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.set(RollerMode.HOLD);

        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = false;
            newInputs.coralDetected = false;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }
}
