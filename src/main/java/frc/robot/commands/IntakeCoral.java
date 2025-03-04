package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class IntakeCoral extends Command{
    private RollerSubsystem rollerSubsystem;
    
    public IntakeCoral(){
        rollerSubsystem = RollerSubsystem.getInstance();

        addRequirements(rollerSubsystem);
    }

    public void initialize(){
        rollerSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted) {
        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = false;
            newInputs.coralDetected = true;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }
}
