package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerMode;

public class RollerSubsystem extends SubsystemBase{
    private static RollerSubsystem instance;

    private RollerIO rollerIO;
    private RollerIOInputs inputs = new RollerIOInputs();

    public RollerSubsystem(){
        if(Robot.isReal()){
            this.rollerIO = new RollerKraken();
        }else{
            this.rollerIO = new RollerSim();
        }
    }

    public static RollerSubsystem getInstance() {
        if (instance == null) {
            instance = new RollerSubsystem();
        }
        return instance;
    }

    public void overrideSimStates(RollerIOInputs desireInputs){
        inputs = desireInputs;
        rollerIO.overrideStates(desireInputs);
    }

    public Boolean algaeDetected(){
        return inputs.algaeDetected;
    }

    public Boolean coralDetected(){
        return inputs.coralDetected;
    }

    public void setIntakeCoral(){
        rollerIO.setMode(RollerMode.IN);
        rollerIO.setHatcher(true);
        rollerIO.setIntake(false);
    }

    public void setIntakeAlgae(){
        rollerIO.setMode(RollerMode.IN);
        rollerIO.setHatcher(false);
        rollerIO.setIntake(true);
    }

    public void outTake(){
        rollerIO.setMode(RollerMode.OUT);
        rollerIO.setHatcher(true);
        rollerIO.setIntake(true);
    }

    public void idle(){
        rollerIO.setMode(RollerMode.IN);
        rollerIO.setHatcher(false);
        rollerIO.setIntake(false);
    }

    @Override
    public void periodic() {
        rollerIO.updateInputs(inputs);
    }
}
