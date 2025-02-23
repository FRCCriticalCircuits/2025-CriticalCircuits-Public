package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class setElevator extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    private String finalState;
    
    public setElevator(String finalState){
        elevatorSubsystem = ElevatorSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        elevatorSubsystem.targetState = finalState;
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.curState == finalState;
    }
}
