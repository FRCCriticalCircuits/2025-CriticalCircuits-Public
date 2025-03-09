package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerMode;

public class IntakeAlgae extends Command{
    private RollerSubsystem rollerSubsystem;
    
    public IntakeAlgae(RollerSubsystem r){
        this.rollerSubsystem = r;
        addRequirements(rollerSubsystem);
    }
    
    @Override
    public void initialize(){
        rollerSubsystem.set(RollerMode.IN);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if(rollerSubsystem.algaeDetected()) rollerSubsystem.set(RollerMode.HOLD);
        else rollerSubsystem.set(RollerMode.HOLD);

        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = true;
            newInputs.coralDetected = false;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }
}
