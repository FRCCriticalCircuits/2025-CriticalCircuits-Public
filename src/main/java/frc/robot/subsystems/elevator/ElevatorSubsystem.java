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
import frc.robot.Constants.Physical;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance;

  private RollerSubsystem rollerSubsystem;

  private ElevatorIO elevatorIO;
  private final ElevatorIOInputs elevatorInputs = new ElevatorIOInputs();

  private ArmIO armIO;
  private final ArmIOInputs armInputs = new ArmIOInputs();

  public String curState = "preMatch", targetState = "preMatch";

  private Debouncer armAtGoal = new Debouncer(0.2);
  private Debouncer elevatorAtGoal = new Debouncer(0.2);

  public boolean atGoal = false, fetchAlgae = false;

  GraphMachine graphMachine = new GraphMachine();
  Pair<String, Pair<Double, Double>> nextState;

  private StructPublisher<Pose3d> wristPose;
  private Pose3d wristPose3d = new Pose3d();

  private boolean enabled;

  public ElevatorSubsystem(RollerSubsystem r) {
    enabled = true;
    rollerSubsystem = r;
    if (Robot.isSimulation()) {
      elevatorIO = new ElevatorSim();
      armIO = new ArmSim();
    } else {
      elevatorIO = new ElevatorKraken();
      armIO = new ArmKraken();
    }

    // State Nodes
    graphMachine.addNode("preMatch", new Pair<Double, Double>(Units.degreesToRotations(60) + 0.305, 0.0)); // 56.0 deg,
                                                                                                           // 0 cm
    graphMachine.addNode("groundAlgae", new Pair<Double, Double>(Units.degreesToRotations(-95) + 0.305, 0.1));
    graphMachine.addNode("L1coral", new Pair<Double, Double>(Units.degreesToRotations(40) + 0.305, 0.15)); // 40.0 deg,
                                                                                                           // 2 cm
    graphMachine.addNode("coralIntake", new Pair<Double, Double>(Units.degreesToRotations(30) + 0.305, 3.090)); // 30.0
                                                                                                                // deg,
                                                                                                                // 50.5cm

    // State Nodes with angle 0
    graphMachine.addNode("L2coral", new Pair<Double, Double>(0.0 + 0.305, 2.25));
    graphMachine.addNode("L3coral", new Pair<Double, Double>(Units.degreesToRotations(-8) + 0.305, 5.25));

    graphMachine.addNode("processorAlgae", new Pair<Double, Double>(Units.degreesToRotations(-40) + 0.305, 0.1));

    graphMachine.addNode("algaeL1", new Pair<Double, Double>(0.0 + 0.305, 2.25));
    graphMachine.addNode("algaeL2", new Pair<Double, Double>(0.0 + 0.305, 5.2));

    // Algae-Fetch Nodes
    graphMachine.addNode("algaeL1-in", new Pair<Double, Double>(Units.degreesToRotations(-40) + 0.305, 2.25));
    graphMachine.addNode("algaeL2-in", new Pair<Double, Double>(Units.degreesToRotations(-40) + 0.305, 5.2));

    // Transition Nodes
    graphMachine.addNode("0n-1", new Pair<Double, Double>(0.305, 0.1)); // 00.0 deg, 0 cm
    graphMachine.addNode("0n-2", new Pair<Double, Double>(0.305, 0.15)); // 00.0 deg, 2.0 cm
    graphMachine.addNode("0n-3", new Pair<Double, Double>(0.305, 3.090)); // 00.0 deg, 50.5 cm

    // Edges for Regular Nodes
    graphMachine.addEdge("0n-1", "preMatch");
    graphMachine.addEdge("0n-1", "groundAlgae");
    graphMachine.addEdge("0n-2", "L1coral");
    graphMachine.addEdge("0n-3", "coralIntake");

    // Edges for Algae-Fetch Nodes
    graphMachine.addEdge("algaeL1", "algaeL1-in");
    graphMachine.addEdge("algaeL2", "algaeL2-in");

    // Edges for State Nodes with angle 0
    graphMachine.addEdge("0n-1", "L2coral");
    graphMachine.addEdge("0n-2", "L2coral");
    graphMachine.addEdge("0n-3", "L2coral");

    graphMachine.addEdge("0n-1", "L3coral");
    graphMachine.addEdge("0n-2", "L3coral");
    graphMachine.addEdge("0n-3", "L3coral");

    graphMachine.addEdge("0n-1", "processorAlgae");
    graphMachine.addEdge("0n-2", "processorAlgae");
    graphMachine.addEdge("0n-3", "processorAlgae");

    graphMachine.addEdge("0n-1", "algaeL1");
    graphMachine.addEdge("0n-2", "algaeL1");
    graphMachine.addEdge("0n-3", "algaeL1");

    graphMachine.addEdge("0n-1", "algaeL2");
    graphMachine.addEdge("0n-2", "algaeL2");
    graphMachine.addEdge("0n-3", "algaeL2");

    graphMachine.addEdge("L2coral", "L3coral");
    graphMachine.addEdge("L2coral", "processorAlgae");
    graphMachine.addEdge("L2coral", "algaeL1");
    graphMachine.addEdge("L2coral", "algaeL2");

    graphMachine.addEdge("L3coral", "processorAlgae");
    graphMachine.addEdge("L3coral", "algaeL1");
    graphMachine.addEdge("L3coral", "algaeL2");

    graphMachine.addEdge("processorAlgae", "algaeL1");
    graphMachine.addEdge("processorAlgae", "algaeL2");

    graphMachine.addEdge("algaeL1", "algaeL2");

    // Edges for Transition Nodes
    graphMachine.addEdge("0n-1", "0n-2");
    graphMachine.addEdge("0n-1", "0n-3");
    graphMachine.addEdge("0n-2", "0n-3");

    wristPose = NetworkTableInstance.getDefault().getStructTopic("/Arm/wristRelativePos", Pose3d.struct).publish();
  }

  public void visualize() {
    wristPose.set(
        this.wristPose3d);
  }

  private void updateRollerTranslation(double elevatorRotation, Rotation2d armRotation2d) {
    this.wristPose3d = new Pose3d(
        0.2383,
        0,
        0.4704 + (elevatorRotation * Physical.Elevator.GEAR_CIRCUMFERENCE_METERS),
        new Rotation3d(
            0,
            -armRotation2d.getRadians(),
            0));
  }

  public synchronized Pose3d getRollerTransltaion() {
    return this.wristPose3d;
  }

  @Override
  public void periodic() {
    AutoAimSetting currentSettings = AutoAimManager.getInstance().getSetting();

    switch (currentSettings.getMode()) {
      case CORAL_PLACE:
        switch (currentSettings.getLevel()) {
          case L1:
            this.targetState = "L1coral";
            break;
          case L2:
            this.targetState = "L2coral";
            break;
          case L3:
            this.targetState = "L3coral";
            break;
          case L4:
            this.targetState = "L3coral";
            break;
          case LClimb:
            this.targetState = "groundAlgae";
        }
        break;
      case CORAL_INTAKE:
        this.targetState = "coralIntake";
        break;
      case ALGAE_INTAKE:
        switch (currentSettings.getLevel()) {
          case L1:
            this.targetState = "groundAlgae"; // No AutoAim
            break;
          case L2:
            this.targetState = "processorAlgae"; // No AutoAim
            break;
          case L3:
            if (fetchAlgae)
              this.targetState = "algaeL1-in"; // Mid
            else
              this.targetState = "algaeL1"; // Mid
            break;
          case L4:
            if (fetchAlgae)
              this.targetState = "algaeL2-in"; // Mid
            else
              this.targetState = "algaeL2"; // Mid
            break;
          case LClimb:
            break;
        }
        break;
    }

    nextState = graphMachine.findPath(curState, targetState);

    armIO.setRotation(Rotation2d.fromRotations(nextState.getSecond().getFirst()));
    armIO.updateInputs(armInputs, rollerSubsystem.hasCoral(), rollerSubsystem.hasAlgae());

    elevatorIO.setPosition(nextState.getSecond().getSecond());
    elevatorIO.updateInputs(elevatorInputs);

    // false if osilating
    atGoal = elevatorAtGoal.calculate(Math.abs(elevatorInputs.position - elevatorInputs.targetPosition) < 0.1) &&
        armAtGoal
            .calculate(Math.abs(armInputs.ioRotation.getRotations() - armInputs.targetRotation.getRotations()) < 0.05);

    // false if error is too big
    atGoal = ((Math.abs(elevatorInputs.position - elevatorInputs.targetPosition) > 0.1) ||
        (Math.abs(armInputs.ioRotation.getRotations() - armInputs.targetRotation.getRotations()) > 0.05)) ? false
            : atGoal;

    updateRollerTranslation(elevatorInputs.position, armInputs.ioRotation);
    visualize();

    SmartDashboard.putString("targetState", nextState.getFirst() + "(arm, elev): " + nextState.getSecond().getFirst()
        + ", " + nextState.getSecond().getSecond());
    SmartDashboard.putString("curState",
        curState + "(arm, elev): " + armInputs.ioRotation + ", " + elevatorInputs.position);
    SmartDashboard.putBoolean("atGoal", atGoal);

    if (atGoal)
      curState = nextState.getFirst();
  }

  public ArmIO getWrist() {
    return this.armIO;
  }
}