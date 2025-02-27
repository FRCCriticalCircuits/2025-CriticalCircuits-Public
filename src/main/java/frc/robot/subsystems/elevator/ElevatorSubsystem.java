package frc.robot.subsystems.elevator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.elevator.ArmIO.ArmIOInputs;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.utils.GraphMachine;
import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.Constants.PhysicalConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance;

    private ElevatorIO elevatorIO;
    private final ElevatorIOInputs elevatorInputs = new ElevatorIOInputs();

    private ArmIO armIO;
    private final ArmIOInputs armInputs = new ArmIOInputs();

    public String curState = "preMatch", targetState = "preMatch";

    private Debouncer armAtGoal = new Debouncer(0.2);
    private Debouncer elevatorAtGoal = new Debouncer(0.2);
 
    private boolean atGoal = false, reachAlgae = false;

    GraphMachine graphMachine = new GraphMachine();
    Pair<String, Pair<Double, Double>> nextState;

    private StructPublisher<Pose3d> wristPose;
    private Pose3d wristPose3d = new Pose3d();
    
    public ElevatorSubsystem() {
        if(Robot.isSimulation()){
            elevatorIO = new ElevatorSim();
            armIO = new ArmSim();
        }else{
            elevatorIO = new ElevatorKraken();
            armIO = new ArmKraken();
        }

        // State Nodes
        graphMachine.addNode("preMatch", new Pair<Double, Double>(Units.degreesToRotations(56), 0.0));      // 56.0 deg,    0 cm
        graphMachine.addNode("L1coral", new Pair<Double, Double>(Units.degreesToRotations(40), 0.15));      // 40.0 deg,    2 cm
        graphMachine.addNode("coralIntake", new Pair<Double, Double>(Units.degreesToRotations(30), 3.55));  // 30.0 deg,    50.5cm

        // Nodes with 0 deg
        graphMachine.addNode("L2coral", new Pair<Double, Double>(Units.degreesToRotations(0), 2.0));        // 00.0 deg,    28.2cm
        graphMachine.addNode("L3coral", new Pair<Double, Double>(Units.degreesToRotations(0), 5.0));        // 00.0 deg,    70.5cm

        // Transition Nodes
        graphMachine.addNode("0n-1", new Pair<Double, Double>(0.0, 0.0));    // 00.0 deg,    0    cm
        graphMachine.addNode("0n-2", new Pair<Double, Double>(0.0, 0.15));   // 00.0 deg,    2.0  cm
        graphMachine.addNode("0n-3", new Pair<Double, Double>(0.0, 3.55));   // 00.0 deg,    50.5 cm
        graphMachine.addNode("0n-4", new Pair<Double, Double>(0.0, 5.0));    // 00.0 deg,    70.5 cm

        // Edges
        graphMachine.addEdge("0n-1", "preMatch");
        graphMachine.addEdge("0n-2", "L1coral");
        graphMachine.addEdge("0n-3", "coralIntake");

        // Edges (Nodes with 0 deg)
        graphMachine.addEdge("0n-1", "L3coral");
        graphMachine.addEdge("0n-2", "L3coral");
        graphMachine.addEdge("0n-3", "L3coral");
        graphMachine.addEdge("0n-4", "L3coral");

        graphMachine.addEdge("0n-1", "L2coral");
        graphMachine.addEdge("0n-2", "L2coral");
        graphMachine.addEdge("0n-3", "L2coral");
        graphMachine.addEdge("0n-4", "L2coral");

        // Transition Edges with angle 0
        graphMachine.addEdge("0n-1", "0n-2");
        graphMachine.addEdge("0n-1", "0n-3");
        graphMachine.addEdge("0n-1", "0n-4");
        graphMachine.addEdge("0n-2", "0n-3");
        graphMachine.addEdge("0n-2", "0n-4");
        graphMachine.addEdge("0n-3", "0n-4");

        wristPose = NetworkTableInstance.getDefault().getStructTopic("/Arm/wristRelativePos", Pose3d.struct).publish();
    }

    public static ElevatorSubsystem getInstance(){
        if(instance == null) instance = new ElevatorSubsystem();
        return instance;
    }

    public void visualize(){
         wristPose.set(
            this.wristPose3d
        );
    }

    private void updateRollerTranslation(double elevatorRotation, Rotation2d armRotation2d){
        this.wristPose3d = new Pose3d(
                0.2383,
                0,
                0.4704 + (elevatorRotation * PhysicalConstants.Elevator.GEAR_CIRCUMFERENCE_METERS),
                new Rotation3d(
                    0,
                    -armRotation2d.getRadians(),
                    0
                )
        );
    }

    public Pose3d getRollerTransltaion(){
        return this.wristPose3d;
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
                        this.targetState = "L3coral"; // L/R
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
                        if(reachAlgae) this.targetState = "algaeL1-in"; // Mid
                        else this.targetState = "algaeL1"; // Mid
                        break;
                    default:
                        if(reachAlgae) this.targetState = "algaeL2-in"; // Mid
                        else this.targetState = "algaeL2"; // Mid
                        break;
                }
        }

        nextState = graphMachine.findPath(curState, targetState);
        
        armIO.setRotation(Rotation2d.fromRotations(nextState.getSecond().getFirst()));
        elevatorIO.setPosition(nextState.getSecond().getSecond());

        elevatorIO.updateInputs(elevatorInputs);
        armIO.updateInputs(armInputs);

        // false if osilating
        atGoal =    elevatorAtGoal.calculate(Math.abs(elevatorInputs.position - elevatorInputs.targetPosition) < 0.1) &&
                    armAtGoal.calculate(Math.abs(armInputs.ioRotation.getRotations() - armInputs.targetRotation.getRotations()) < 0.03);
        
        // false if error is too big
        atGoal =    (
                        (Math.abs(elevatorInputs.position - elevatorInputs.targetPosition) > 0.1) ||
                        (Math.abs(armInputs.ioRotation.getRotations() - armInputs.targetRotation.getRotations()) > 0.03)
                    ) ? false 
                      : atGoal;

        updateRollerTranslation(elevatorInputs.position, armInputs.ioRotation);
        visualize();

        // debug
        SmartDashboard.putString("targetState", nextState.getFirst() + "(arm, elev): " + nextState.getSecond().getFirst() + ", " + nextState.getSecond().getSecond());
        SmartDashboard.putString("curState", curState + "(arm, elev): " + armInputs.ioRotation + ", " + elevatorInputs.position);
        SmartDashboard.putBoolean("atGoal", atGoal);

        if(atGoal) curState = nextState.getFirst();
    }
}