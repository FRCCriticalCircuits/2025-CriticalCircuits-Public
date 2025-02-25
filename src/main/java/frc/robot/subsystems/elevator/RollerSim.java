package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RollerSim implements RollerIO{
    RollerIOInputs inputsSim = new RollerIOInputs();

    public RollerSim(){

    }

    @Override
    public void overrideStates(RollerIOInputs inputs) {
        this.inputsSim = inputs;
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs = this.inputsSim;

        SmartDashboard.putBoolean("algae", inputs.algaeDetected);
        SmartDashboard.putBoolean("coral", inputs.coralDetected);
    }
}
