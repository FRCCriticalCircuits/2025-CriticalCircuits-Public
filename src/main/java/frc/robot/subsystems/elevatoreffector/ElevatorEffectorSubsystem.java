package frc.robot.subsystems.elevatoreffector;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GraphMachine;
import org.littletonrobotics.junction.Logger;

public class ElevatorEffectorSubsystem extends SubsystemBase {
    public enum ElevatorState {
        IDLE, ZERO,
        L1_CORAL, L1_ALGAE, L2_CORAL, L2_ALGAE, L3_CORAL,
        INTAKE_CORAL, INTAKE_ALGAE,
        BARGE,
        T1, T2, T3 // Transition states
    }

    private final ElevatorIO elevatorIO;
    private final WristIO wristIO;

    private final GraphMachine graph;

    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    private ElevatorState currentState;
    private ElevatorState requestedState;

    public ElevatorEffectorSubsystem(ElevatorIO elevatorIO, WristIO wristIO) {
        this.elevatorIO = elevatorIO;
        this.wristIO = wristIO;

        // Default state
        currentState = ElevatorState.ZERO;
        requestedState = ElevatorState.IDLE;

        graph = new GraphMachine();

        graph.addNode();


        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        wristIO.updateInputs(wristInputs);

        Logger.processInputs("Elevator", elevatorInputs);
        Logger.processInputs("Wrist", wristInputs);

        // TODO: implement graph for elevator

    }
}

