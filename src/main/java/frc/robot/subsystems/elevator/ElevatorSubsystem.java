package frc.robot.subsystems.elevator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.elevator.ArmIO.ArmIOInputs;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.utils.GraphMachine;
import frc.robot.utils.structures.AutoAimSetting;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;

    private ElevatorIO elevatorIO = new ElevatorKraken();
    private final ElevatorIOInputs elevatorInputs = new ElevatorIOInputs();

    private ArmIO armIO = new ArmKraken();
    private final ArmIOInputs armInputs = new ArmIOInputs();

    private RollerIO rollerIO = new RollerKraken();
    private final RollerIOInputs rollerInputs = new RollerIOInputs();

    public String curState = "preMatch", targetState = "preMatch";

    Debouncer armAtGoal = new Debouncer(0.2);
    Debouncer elevatorAtGoal = new Debouncer(0.2);

    boolean atGoal;

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

        // Edges
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
        AutoAimSetting currentSettings = AutoAimManager.getInstance().getSetting();

        switch(currentSettings.getMode()){
            case CORAL_PLACE:
                switch (currentSettings.getLevel()) {
                    case L1:
                        this.targetState = "L1coral"; // Mid
                        break;
                    case L2:
                        this.targetState = "L2coral"; // L/R
                        break;
                    case L3:
                        this.targetState = "L3coral"; // L/R
                        break;
                    default:
                        this.targetState = "L4coral"; // L/R
                        break;
                }
                break;
            case CORAL_INTAKE:
                this.targetState = "coralIntake";
                break;
            default:
                switch (currentSettings.getLevel()) {
                    case L1:
                        this.targetState = "groundAlgae"; // No AutoAim
                        break;
                    case L2:
                        this.targetState = "processorAlgae"; // No AutoAim
                        break;
                    case L3:
                        this.targetState = "algaeL1"; // Mid
                        break;
                    default:
                        this.targetState = "algaeL2"; // Mid
                        break;
                }
        }

        elevatorIO.updateInputs(elevatorInputs);
        armIO.updateInputs(armInputs);
        rollerIO.updateInputs(rollerInputs);

        // false if osilating
        atGoal =    elevatorAtGoal.calculate(Math.abs(elevatorInputs.position - elevatorInputs.targetPosition) < 0.02) &&
                    armAtGoal.calculate(Math.abs(armInputs.ioRotation.getRotations() - armInputs.targetRotation.getRotations()) < 0.02);
        
        // false if error is too big
        atGoal =    (
                        (Math.abs(elevatorInputs.position - elevatorInputs.targetPosition) > 0.02) ||
                        (Math.abs(armInputs.ioRotation.getRotations() - armInputs.targetRotation.getRotations()) > 0.02)
                    ) ? false 
                      : atGoal;

        nextState = graphMachine.findPath(curState, targetState);
        armIO.setRotation(Rotation2d.fromRotations(nextState.getSecond().getFirst()));
        elevatorIO.setPosition(nextState.getSecond().getSecond());

        // debug
        SmartDashboard.putString("targetState", nextState.getFirst() + "(arm, elev): " + nextState.getSecond().getFirst() + ", " + nextState.getSecond().getSecond());
        SmartDashboard.putBoolean("atGoal", atGoal);

        if(atGoal) curState = nextState.getFirst();
    }
}