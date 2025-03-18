package frc.robot.subsystems.elevatoreffector;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorEffectorSubsystem extends SubsystemBase {
    public enum ElevatorState {
        IDLE, ZERO,
        L1_CORAL, L1_ALGAE, L2_CORAL, L2_ALGAE, L3_CORAL,
        INTAKE_CORAL, INTAKE_ALGAE,
        BARGE
    }

    private WristIO wristIO;

    private ElevatorState currentState;
    private ElevatorState requestedState;

    public ElevatorEffectorSubsystem() {
        // Default state
        currentState = ElevatorState.ZERO;
        requestedState = ElevatorState.IDLE;
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

    }

    @Override
    public void periodic() {
        switch (currentState) {
            case ZERO:

                break;
            case IDLE:
                break;
            case L1_CORAL:
                break;
            case L2_CORAL:
                break;
            case L3_CORAL:
                break;
            case L1_ALGAE:
                break;
            case L2_ALGAE:
                break;
            case INTAKE_CORAL:
                break;
            case INTAKE_ALGAE:
                break;
            case BARGE:
                break;
        }
        super.periodic();
    }
}

