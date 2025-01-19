package frc.robot.utils.structures;

import frc.robot.Constants;

public class SolverResult {
    private double[][] values = new double[Constants.SAMPLE_NUM][2];

    public SolverResult(double[][] values){
        this.values = values;
    }

    public double[][] asArray(){
        return this.values;
    }
}
