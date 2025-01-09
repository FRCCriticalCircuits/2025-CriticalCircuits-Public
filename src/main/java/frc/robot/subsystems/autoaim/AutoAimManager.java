package frc.robot.subsystems.autoaim;

import frc.robot.utils.DataStrcutures.AutoAimSetting;

public class AutoAimManager {
    public static AutoAimSetting setting;

    public static void configure(AutoAimSetting setting){
        AutoAimManager.setting = setting;
    }

    public static void runSwerveAutoAim(){
        /*
         * get pose
         * estimateStationAdvancedPose2D(currentPose, aimSetting[uses Spot / maybe mode for y translation], CONTROLLER_INPUT * RANGE)
         * 
         */
    }
}
