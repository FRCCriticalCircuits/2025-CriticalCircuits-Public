package frc.robot.utils.structures;

import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;

public class AutoAimSetting{
    public Spot spot;
    public Level level;
    public Mode mode;

    public AutoAimSetting(Spot spot, Level level, Mode mode){
        this.spot = spot;
        this.level = level;
        this.mode = mode;
    }
}