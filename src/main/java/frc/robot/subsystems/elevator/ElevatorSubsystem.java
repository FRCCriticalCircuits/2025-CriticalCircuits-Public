package frc.robot.subsystems.elevator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ArmIO.ArmIOInputs;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.utils.GraphMachine;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;

    private ElevatorIO elevatorIO = new ElevatorKraken();
    private final ElevatorIOInputs elevatorInputs = new ElevatorIOInputs();

    private ArmIO armIO = new ArmKraken();
    private final ArmIOInputs armInputs = new ArmIOInputs();

    public String curState = "preMatch", targetState = "preMatch";

    Debouncer armAtGoal = new Debouncer(0.2);
    Debouncer elevatorAtGoal = new Debouncer(0.2);

    Debouncer coralDebouncer = new Debouncer(0.1);
    Debouncer algaeDebouncer = new Debouncer(0.1);

    boolean atGoal, coralDetected, algaeDetected;

    GraphMachine graphMachine = new GraphMachine();
    Pair<String, Pair<Double, Double>> nextState;
    
    public ElevatorSubsystem() {
        // State Nodes
        graphMachine.addNode("preMatch", new Pair<Double, Double>(Units.degreesToRotations(20), 0.0));
        graphMachine.addNode("L1coral", new Pair<Double, Double>(Units.degreesToRotations(10), 1.0));
        graphMachine.addNode("L4coral", new Pair<Double, Double>(Units.degreesToRotations(10), 5.0));

        // Transition Nodes
        graphMachine.addNode("tn-1", new Pair<Double, Double>(Units.degreesToRotations(10), 0.0));
        graphMachine.addNode("tn-2", new Pair<Double, Double>(0.0, 0.0));
        graphMachine.addNode("tn-3", new Pair<Double, Double>(0.0, 5.0));

        graphMachine.addEdge("preMatch", "tn-1");
        graphMachine.addEdge("preMatch", "tn-2");
        graphMachine.addEdge("tn-1", "L1coral");
        graphMachine.addEdge("tn-1", "tn-2");
        graphMachine.addEdge("tn-2", "tn-3");
        graphMachine.addEdge("tn-3", "L4coral");
    }

    public static ElevatorSubsystem getInstance(){
        if(instance == null) instance = new ElevatorSubsystem();
        return instance;
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);
        armIO.updateInputs(armInputs);

        // false if osilating
        atGoal =    elevatorAtGoal.calculate(Math.abs(elevatorInputs.position - elevatorInputs.targetPosition) < 0.1) &&
                    armAtGoal.calculate(Math.abs(armInputs.rotation - armInputs.targetRotation) < 0.1);
        
        // false if error is too big
        atGoal =    (
                        (Math.abs(elevatorInputs.position - elevatorInputs.targetPosition) > 0.1) &&
                        (Math.abs(armInputs.rotation - armInputs.targetRotation) > 0.1)
                    ) ? false 
                      : atGoal;

        algaeDetected = algaeDebouncer.calculate(armInputs.algaeDetected);
        coralDetected = coralDebouncer.calculate(armInputs.coralDetected);

        nextState = graphMachine.findPath(curState, targetState);
        armIO.setRotation(nextState.getSecond().getFirst());
        elevatorIO.setPosition(nextState.getSecond().getSecond());

        if(atGoal) curState = nextState.getFirst();
    }
}