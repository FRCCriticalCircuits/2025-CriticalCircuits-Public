package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSim implements ElevatorIO{
    private DCMotorSim motorSim;
    private PIDController pidController = new PIDController(2, 0, 0);
    
    private double targetRotation = 0.0;
    
    public ElevatorSim(){
        this.motorSim = new DCMotorSim(LinearSystemId.identifyPositionSystem(0.11, 0.01), DCMotor.getKrakenX60(2));
    }

    @Override
    public void setPosition(double rotation) {
        this.targetRotation = rotation;
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        motorSim.update(0.02);

        motorSim.setInputVoltage(pidController.calculate(inputs.position, targetRotation));
        inputs.position = motorSim.getAngularPositionRotations();
        inputs.targetPosition = this.targetRotation;

        SmartDashboard.putNumber("ElevSim/position", inputs.position);
        SmartDashboard.putNumber("ElevSim/targetPosition", inputs.targetPosition);
    }
}
