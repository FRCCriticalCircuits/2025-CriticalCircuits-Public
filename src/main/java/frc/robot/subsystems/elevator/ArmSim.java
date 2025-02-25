package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhysicalConstants;

public class ArmSim implements ArmIO{
    private DCMotorSim motorSim;
    private PIDController pidController = new PIDController(150, 0, 10);

    private Rotation2d targetIORotation = Rotation2d.fromDegrees(83.8);

    public ArmSim(){
        this.motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(1.6, 0.1), DCMotor.getKrakenX60(1));

        motorSim.setAngle(PhysicalConstants.Arm.OFFSET_CENTER_GRAVITY.getRadians());
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        this.targetIORotation = rotation;
    }

    private Rotation2d toTalonRotation(Rotation2d ioRotation){
        return ioRotation.plus(PhysicalConstants.Arm.OFFSET_CENTER_GRAVITY);
    }
    
    private Rotation2d toIORotation(Rotation2d talonRotation){
        return talonRotation.minus(PhysicalConstants.Arm.OFFSET_CENTER_GRAVITY);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        motorSim.update(0.02);

        motorSim.setInputVoltage(pidController.calculate(motorSim.getAngularPositionRotations(), toTalonRotation(targetIORotation).getRotations()));

        inputs.ioRotation = toIORotation(Rotation2d.fromRotations(motorSim.getAngularPositionRotations()));
        inputs.targetRotation = this.targetIORotation;

        SmartDashboard.putNumber("ArmSim/position", inputs.ioRotation.getDegrees());
        SmartDashboard.putNumber("ArmSim/targetPosition", inputs.targetRotation.getDegrees());
    }
}
