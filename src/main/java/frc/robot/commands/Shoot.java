package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.RollerIO.RollerIOInputs;
import frc.robot.subsystems.elevator.RollerIO.RollerMode;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.subsystems.AutoAimManager;
import frc.robot.subsystems.elevator.RollerSubsystem;

public class Shoot extends Command{
    private RollerSubsystem rollerSubsystem;

    public Shoot(RollerSubsystem r){
        rollerSubsystem = r;

        addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize(){
        if (AutoAimManager.getInstance().getSetting().getLevel() == Level.L1) {
            rollerSubsystem.setMode(RollerMode.CORAL_OUT_LIGHT);
        } else {
            rollerSubsystem.setMode(RollerMode.OUT);
        }
        
        /*
         * if(Robot.isSimulation()){
            Pose3d wristTranslation = ElevatorSubsystem.getInstance().getRollerTransltaion();

            if(rollerSubsystem.coralDetected()){
                SimulatedArena.getInstance().addGamePieceProjectile(
                    new ReefscapeCoralOnFly(
                        SwerveSubsystem.driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        wristTranslation.getTranslation().toTranslation2d(),
                        SwerveSubsystem.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        SwerveSubsystem.driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        wristTranslation.getMeasureZ(),
                        MetersPerSecond.of(3),
                        wristTranslation.getRotation().getMeasureY().times(-1)
                    ).enableBecomesGamePieceOnFieldAfterTouchGround()
                );
            } 
            
            if(rollerSubsystem.algaeDetected()){
                SimulatedArena.getInstance().addGamePieceProjectile(
                    new ReefscapeAlgaeOnFly(
                        SwerveSubsystem.driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        wristTranslation.getTranslation().toTranslation2d(),
                        SwerveSubsystem.driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        SwerveSubsystem.driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        wristTranslation.getMeasureZ(),
                        MetersPerSecond.of(3),
                        wristTranslation.getRotation().getMeasureY().times(-1)
                    ).enableBecomesGamePieceOnFieldAfterTouchGround()
                );
            }
        }
         */
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.setMode(RollerMode.HOLD);

        if(Robot.isSimulation()){
            RollerIOInputs newInputs = new RollerIOInputs();
            newInputs.algaeDetected = false;
            newInputs.coralDetected = false;
            rollerSubsystem.overrideSimStates(newInputs);
        }
    }
}
