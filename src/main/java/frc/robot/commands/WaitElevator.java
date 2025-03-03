package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class WaitElevator extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    
    public WaitElevator(){
        elevatorSubsystem = ElevatorSubsystem.getInstance();

        addRequirements(elevatorSubsystem);
        addRequirements(RollerSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        Timer.delay(0.1);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.curState == elevatorSubsystem.targetState;
    }
}
