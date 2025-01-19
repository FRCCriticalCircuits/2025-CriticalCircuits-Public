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

    public void setSpot(Spot spot){
        this.spot = spot;
    }

    public Level getLevel(){
        return this.level;
    }

    public void setLevel(Level level){
        this.level = level;
    }

    public Mode getMode(){
        return this.mode;
    }

    public void setMode(Mode mode){
        this.mode = mode;
    }
}