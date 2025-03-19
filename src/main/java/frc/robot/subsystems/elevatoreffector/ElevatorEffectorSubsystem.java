package frc.robot.subsystems.elevatoreffector;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.utils.structures.DataStrcutures;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.elevatoreffector.ElevatorConstants.Physical;
import static frc.robot.utils.structures.DataStrcutures.*;

public class ElevatorEffectorSubsystem extends SubsystemBase {
    public enum ElevatorState {
        IDLE, ZERO,
        L1_CORAL, L1_ALGAE, L2_CORAL, L2_ALGAE, L3_CORAL, L4_CORAL,
        INTAKE_CORAL, INTAKE_ALGAE,
        BARGE_ALGAE,
        T1, T2, T3 // Transition states
    }

    private final ElevatorIO elevatorIO;
    private final WristIO wristIO;

    private final ElevatorStateMachine stateMachine;

    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    private ElevatorState currentState;
    private ElevatorState requestedState;

    private Mode mode;
    private Level level;

    public ElevatorEffectorSubsystem(ElevatorIO elevatorIO, WristIO wristIO) {
        LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();
        this.elevatorIO = elevatorIO;
        this.wristIO = wristIO;

        // Default state
        currentState = ElevatorState.ZERO;
        // Target IDLE state at startup
        requestedState = ElevatorState.IDLE;

        stateMachine = new ElevatorStateMachine();

        stateMachine.addNode(ElevatorState.ZERO, Physical.ZERO);
        stateMachine.addNode(ElevatorState.IDLE, Physical.IDLE);
        stateMachine.addNode(ElevatorState.L1_CORAL, Physical.L1_CORAL);
        stateMachine.addNode(ElevatorState.L2_CORAL, Physical.L2_CORAL);
        stateMachine.addNode(ElevatorState.L3_CORAL, Physical.L3_CORAL);
        stateMachine.addNode(ElevatorState.L4_CORAL, Physical.L4_CORAL);
        stateMachine.addNode(ElevatorState.INTAKE_CORAL, Physical.INTAKE_CORAL);

        stateMachine.addNode(ElevatorState.L1_ALGAE, Physical.L1_ALGAE);
        stateMachine.addNode(ElevatorState.L2_ALGAE, Physical.L2_ALGAE);
        stateMachine.addNode(ElevatorState.INTAKE_ALGAE, Physical.INTAKE_ALGAE);
        stateMachine.addNode(ElevatorState.BARGE_ALGAE, Physical.BARGE_ALGAE);

        stateMachine.addNode(ElevatorState.T1, Physical.T1);

        stateMachine.addEdge(ElevatorState.ZERO, ElevatorState.IDLE);

        // Block 1
        stateMachine.addFullyConnectedEdges(
                ElevatorState.IDLE,
                ElevatorState.L1_CORAL,
                ElevatorState.INTAKE_ALGAE,
                ElevatorState.T1
        );

        // Block 2
        stateMachine.addFullyConnectedEdges(
                ElevatorState.T1,
                ElevatorState.L1_ALGAE,
                ElevatorState.L2_ALGAE,
                ElevatorState.L2_CORAL,
                ElevatorState.L3_CORAL
        );

        // Block 3
        stateMachine.addFullyConnectedEdges(
                ElevatorState.L3_CORAL,
                ElevatorState.INTAKE_CORAL,
                ElevatorState.BARGE_ALGAE
        );


        // TODO: set heigh constnats properly

        switch (this.mode) {
            case CORAL_INTAKE -> {
                ledSubsystem.setColor(Color.kRed);
                this.requestedState = ElevatorState.INTAKE_CORAL;
            }
            case CORAL_PLACE -> {
                ledSubsystem.setColor(Color.kBlue);
                switch(this.level) {
                    case L1 -> {
                    }
                    case L2 -> {
                    }
                    case L3 -> {
                    }
                    case L4 -> {
                    }
                    case LClimb -> {
                    }
                }
            }
            case ALGAE_INTAKE -> {
                ledSubsystem.setColor(Color.kGreen);
                switch(this.level) {
                    case L1 -> {
                        // g intake
                    }
                    case L2 -> {
                        // processor
                    }
                    case L3 -> {
                        // l1 algae

                    }
                    case L4 -> {
                        // l2 algae
                    }
                }
            }
        }

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

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Mode getMode() {
        return this.mode;
    }

    public void setLevel(Level level) {
        this.level = level;
    }

    public Level getLevel() {
        return this.level;
    }
}

