package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerMode;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class IntakeCoral extends Command{
    private RollerSubsystem rollerSubsystem;
    
    public IntakeCoral(RollerSubsystem r){
        rollerSubsystem = r;

        addRequirements(rollerSubsystem);
    }
    
    @Override
    public void initialize(){
        rollerSubsystem.set(RollerMode.IN);
    }

    @Override
    public boolean isFinished() {
        return rollerSubsystem.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.set(RollerMode.HOLD);

        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = false;
            newInputs.coralDetected = true;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }
}
