    package frc.robot.subsystems;

    import java.util.function.Supplier;

    import com.pathplanner.lib.auto.AutoBuilder;
    import com.pathplanner.lib.path.PathConstraints;

    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.networktables.StructPublisher;
    import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
    import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
    import frc.robot.utils.DriveStationIO.DriveStationIO;
    import frc.robot.utils.Math.AdvancedPose2D;
    import frc.robot.utils.structures.AutoAimSetting;
    import frc.robot.utils.structures.DataStrcutures.Level;
    import frc.robot.utils.structures.DataStrcutures.Mode;
    import frc.robot.utils.structures.DataStrcutures.Spot;
    import frc.robot.utils.web.WebServer;

    public class AutoAimManager{
        private static AutoAimManager instance;
        
        private WebServer server = WebServer.getInstance();
        private Notifier notifier = new Notifier(this::updateValues);

        private Command command;

        private AutoAimSetting settingTemp;
        private Supplier<Double> LTSupplier, RTSupplier;

        private Pose2d targetPose = new Pose2d();

        private StructPublisher<Pose2d> autoAimPositionPublisher;
        
        private PathConstraints constraints = new PathConstraints(
            .9,
            3,
            Math.PI / 4,
            Math.PI / 2
        );

        private AutoAimManager(Supplier<Double> LTSupplier, Supplier<Double> RTSupplier){
            this.LTSupplier = LTSupplier;
            this.RTSupplier = RTSupplier;

            notifier.setName("Autoaim Thread");
            notifier.startPeriodic(0.05);

            autoAimPositionPublisher = NetworkTableInstance.getDefault().getStructTopic("/AutoAim/estimatedPosition", Pose2d.struct).publish();
        }
        
        public synchronized static AutoAimManager getInstance(){
            if(instance == null) throw new NullPointerException("AutoAimManager is not initialized");
            return instance;
        }

        public synchronized static AutoAimManager getInstance(Supplier<Double> LTSupplier, Supplier<Double> RTSupplier){
            if(instance == null) instance = new AutoAimManager(LTSupplier, RTSupplier);
            return instance;
        }

        /**
         * Find the cloest coral station based on current Pos
         * @param currentPos current {@link Translation2d} for the robot
         * @return the pose for target coral station
         */
        private AdvancedPose2D cloestCoralStation(Translation2d currentPos){
            if(DriveStationIO.getAlliance() == Alliance.Blue){
                if (
                    currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_A.getTranslation()) > 
                    currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_B.getTranslation())
                ) return FieldConstants.AutoAim.CORAL_STATION_B;
                else return FieldConstants.AutoAim.CORAL_STATION_A;
            }else{
                if (
                    currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_C.getTranslation()) > 
                    currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_D.getTranslation())
                ) return FieldConstants.AutoAim.CORAL_STATION_D;
                else return FieldConstants.AutoAim.CORAL_STATION_C;
            }
        }

        private AdvancedPose2D nearest(Pose2d currentPose) {
            AdvancedPose2D targetPose = null;
            double minDist = Double.MAX_VALUE;

            for(AdvancedPose2D pose: FieldConstants.AutoAim.LIST_REEF_POS){
                double tmpDist = pose.getTranslation().getDistance(currentPose.getTranslation());
                if(tmpDist < minDist){
                    targetPose = pose;
                    minDist = tmpDist;
                }
            }

            return targetPose;
        }

        /**
         * estimate the station based on current heading
         * @param setting Autoaim Setting Includes {@link Spot}, {@link Level}, {@link Mode}
         * @param manualTranslation manual translation distence from the origin aim pos
         * @return {@link AdvancedPose2D} for pathplanning
         */
        private AdvancedPose2D estimateAimPos(AutoAimSetting setting, Translation2d manualTranslation){
            Pose2d currentPos = SwerveSubsystem.getInstance().getPoseEstimate();

            if(setting.getMode() == Mode.CORAL_INTAKE){
                return cloestCoralStation(currentPos.getTranslation());
            }else if (setting.getMode() == Mode.CORAL_PLACE){
                AdvancedPose2D aimPose = nearest(currentPos);

                if(setting.getSpot() == Spot.L){
                    return aimPose.withRobotRelativeTransformation(new Translation2d(-FieldConstants.AutoAim.AUTO_TRANSLATION + FieldConstants.AutoAim.AUTO_TRANSLATION_OFFSET, 0));
                }else if(setting.getSpot() == Spot.R){
                    return aimPose.withRobotRelativeTransformation(new Translation2d(FieldConstants.AutoAim.AUTO_TRANSLATION + FieldConstants.AutoAim.AUTO_TRANSLATION_OFFSET, 0));
                }else{
                    return aimPose.withRobotRelativeTransformation(manualTranslation);
                }
            }else{
                AdvancedPose2D aimPose = nearest(currentPos);
                return aimPose.withRobotRelativeTransformation(manualTranslation);
            }
        }

        /**
         * a perodic funtion (0.05s) updates values from WebSocket, save/send Estimate Target Pose
         */
        private void updateValues() {
            settingTemp = server.getAutoAimSettings();

            targetPose = estimateAimPos(
                settingTemp,
                new Translation2d
                (
                    FieldConstants.AutoAim.MANUAL_TRANSLATION_RANGE * (LTSupplier.get() - RTSupplier.get()),
                    0
                )
            );
            
            autoAimPositionPublisher.set(targetPose);
        }

        /**
         * get the PathFinding Command based on current settings
         * @return the {@link Command} to execute
         */
        public synchronized Command getCommand(){
            updateValues();

            SwerveSubsystem.getInstance().m_field.getObject("Target").setPose(targetPose);

            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            this.command = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
            );

            return new PrintCommand("autoaim");
        }

        public synchronized boolean isFinished(){
            return this.command.isFinished();
        }

        public synchronized void cancle(){
            if (this.command != null) this.command.cancel();
        }

        public synchronized AutoAimSetting getSetting(){
            return server.getAutoAimSettings();
        }

        public synchronized void updateSetting(AutoAimSetting desireSetting){
            this.settingTemp = desireSetting;
            server.updateSetting(desireSetting);
        }

        public synchronized void updateSpot(Spot spot){
            updateSetting(server.getAutoAimSettings().withSpot(spot));
        }

        public synchronized void updateMode(Mode mode){
            updateSetting(server.getAutoAimSettings().withMode(mode));

            LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

            switch (mode.value) {
                case 0:
                    ledSubsystem.setColor(Color.kRed);
                    break;
                case 1:
                    ledSubsystem.setColor(Color.kBlue);
                    break;
                case 2:
                    ledSubsystem.setColor(Color.kGreen);
                    break;
          }
        }

        public Mode getMode(){
            return server.getAutoAimSettings().getMode();
        }        

        public synchronized void updateLevel(Level level){
            updateSetting(server.getAutoAimSettings().withLevel(level));
        }
    }
