package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class WaitElevator extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    
    public WaitElevator(ElevatorSubsystem elev){
        elevatorSubsystem = elev;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atGoal;
    }
}
