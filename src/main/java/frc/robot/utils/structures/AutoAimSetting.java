package frc.robot.utils.structures;

import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;

public class AutoAimSetting{
    private Spot spot;
    private Level level;
    private Mode mode;

    public AutoAimSetting(Spot spot, Level level, Mode mode){
        this.spot = spot;
        this.level = level;
        this.mode = mode;
    }

    public Spot getSpot(){
        return this.spot;
    }

    public AutoAimSetting withSpot(Spot spot){
        this.spot = spot;
        return this;
    }

    public Level getLevel(){
        return this.level;
    }

    public AutoAimSetting withLevel(Level level){
        this.level = level;
        return this;
    }

    public Mode getMode(){
        return this.mode;
    }

    public AutoAimSetting withMode(Mode mode){
        this.mode = mode;
        return this;
    }
}