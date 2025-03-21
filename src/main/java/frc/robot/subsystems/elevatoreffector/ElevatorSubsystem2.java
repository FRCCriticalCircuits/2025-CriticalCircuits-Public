package frc.robot.subsystems.elevatoreffector;


import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.RollerIO;
import frc.robot.subsystems.led.LEDSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.elevatoreffector.ElevatorConstants.Physical;
import static frc.robot.utils.structures.DataStrcutures.Level;
import static frc.robot.utils.structures.DataStrcutures.Mode;

public class ElevatorSubsystem2 extends SubsystemBase {
    public enum ElevatorState {
        IDLE, ZERO, CLIMB,
        L1_CORAL, L2_CORAL, L3_CORAL, L4_CORAL,
        L1_ALGAE, L1_ALGAE_IN, L2_ALGAE, L2_ALGAE_IN,
        STATION_INTAKE_CORAL, GROUND_INTAKE_ALGAE,
        BARGE_ALGAE, PROCESSOR_ALGAE,
        T1 // Transition states
    }

    private final LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

    private final ElevatorIO elevatorIO;
    private final WristIO wristIO;

    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    private final ElevatorStateMachine stateMachine;
    private ElevatorState currentState;
    private ElevatorState requestedState;

    private Mode mode;
    private Level level;
    private boolean algaeIntake = false;

    public ElevatorSubsystem2(ElevatorIO elevatorIO, WristIO wristIO) {
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
        stateMachine.addNode(ElevatorState.STATION_INTAKE_CORAL, Physical.STATION_INTAKE_CORAL);

        stateMachine.addNode(ElevatorState.L1_ALGAE, Physical.L1_ALGAE);
        stateMachine.addNode(ElevatorState.L2_ALGAE, Physical.L2_ALGAE);
        stateMachine.addNode(ElevatorState.GROUND_INTAKE_ALGAE, Physical.GROUND_INTAKE_ALGAE);
        stateMachine.addNode(ElevatorState.BARGE_ALGAE, Physical.BARGE_ALGAE);

        stateMachine.addNode(ElevatorState.T1, Physical.T1);

        stateMachine.addEdge(ElevatorState.ZERO, ElevatorState.IDLE);

        // Block 1
        stateMachine.addFullyConnectedEdges(
                ElevatorState.IDLE,
                ElevatorState.L1_CORAL,
                ElevatorState.GROUND_INTAKE_ALGAE,
                ElevatorState.PROCESSOR_ALGAE,
                ElevatorState.T1,
                ElevatorState.CLIMB
        );

        // Block 2
        stateMachine.addFullyConnectedEdges(
                ElevatorState.T1,
                ElevatorState.L1_ALGAE,
                ElevatorState.L2_ALGAE,
                ElevatorState.L2_CORAL,
                ElevatorState.L3_CORAL
        );

        stateMachine.addEdge(ElevatorState.L1_ALGAE, ElevatorState.L1_ALGAE_IN);
        stateMachine.addEdge(ElevatorState.L2_ALGAE, ElevatorState.L2_ALGAE_IN);

        // Block 3
        // The transition state T2 is actually L3_CORAL
        stateMachine.addFullyConnectedEdges(
                ElevatorState.L3_CORAL,
                ElevatorState.STATION_INTAKE_CORAL,
                ElevatorState.BARGE_ALGAE
        );

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
        // TODO: set heigh constnats properly

        // gigantic switch statement that turns the mode, level to a state
        this.requestedState = updateCurrentRequestedState(mode, level);

        // Don;t run state machine if we are already at the wanted state
        if (currentState == requestedState) {
            return;
        } else {
            // Check if we have reached our goal state yet
            // set the current state to be the requested state if true
            if (isAtTargetState()) currentState = requestedState;
        }

        // Find next node
        Pair<Distance, Angle> next = stateMachine.findPath(currentState, requestedState).getCommands();

        elevatorIO.setTargetPosition(next.getFirst());
        wristIO.setTargetAngle(next.getSecond());
    }

    private ElevatorState updateCurrentRequestedState(Mode mode, Level level) {
        switch (this.mode) {
            case CLIMB -> {
                return ElevatorState.CLIMB;
            }
            case CORAL_INTAKE -> {
                ledSubsystem.setColor(Color.kRed);
                // FIXME: temporary IDLE state for testing
                return ElevatorState.IDLE;
//                this.requestedState = ElevatorState.STATION_INTAKE_CORAL;
            }
            case CORAL_PLACE -> {
                ledSubsystem.setColor(Color.kBlue);
                switch (this.level) {
                    case L1 -> {
                        return ElevatorState.L1_CORAL;
                    }
                    case L2 -> { // TODO: for now, only do L1 to L2 to confirm T1 state works
                        return ElevatorState.L2_CORAL;
                    }
                    case L3 -> {
//                        this.requestedState = ElevatorState.L3_CORAL;
                    }
                    case L4 -> {
//                        this.requestedState = ElevatorState.L4_CORAL;
                    }
                    case LClimb -> {
                    }
                    default -> {
                        return ElevatorState.IDLE;
                    }
                }
            }
            case ALGAE_INTAKE -> {
                ledSubsystem.setColor(Color.kGreen);
                // FIXME: temporary IDLE state for testing
                return ElevatorState.IDLE;

                /*
                switch (this.level) {
                    case L1 -> {
                        return ElevatorState.GROUND_INTAKE_ALGAE;
                    }
                    case L2 -> {
                        return ElevatorState.PROCESSOR_ALGAE;
                    }
                    case L3 -> {
                        if (algaeIntake) {
                            return ElevatorState.L1_ALGAE_IN;
                        }
                        return ElevatorState.L1_ALGAE;
                        // l1 algae

                    }
                    case L4 -> {
                        if (algaeIntake) {
                            return ElevatorState.L2_ALGAE_IN;
                        }
                        return ElevatorState.L2_ALGAE;
                    }
                }
                */
            }
        }
        return null;
    }

    /**
     * Check if the elevator + wrist are at the goal state yet
     *
     * @return true if yes
     */
    public boolean isAtTargetState() {
        double dElev = Math.abs(elevatorInputs.targetPosition.minus(elevatorInputs.currentPosition).in(Inches));
        double dWrist = Math.abs(wristInputs.targetRotation.minus(wristInputs.currentRotation).in(Degrees));
        // 0.5in and 5 degrees of tolerance for "at position"
        return dElev < 0.5 && dWrist < 5;
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

    public Command fetchAlgaeCommand() {
        return startEnd(() -> algaeIntake = true, () -> algaeIntake = false);
    }

    public Command setLevelCommand(Level l) {
        return runOnce(() -> setLevel(l));
    }

    public Command setModeCommand(Mode m) {
        return runOnce(() -> setMode(m));
    }
}

