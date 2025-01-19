package frc.robot.utils.structures;

public class SolverRequest {
    private double[] values = new double[4];
    private int m_id;

    public SolverRequest(double thetaInit, double thetaFinal, double heightInit, double heightFinal){
        this.values[0] = thetaInit;
        this.values[1] = thetaFinal;
        this.values[2] = heightInit;
        this.values[3] = heightFinal;

        m_id = values.hashCode();
    }

    public int getId(){
        return m_id;
    }

    public double[] asArray(){
        return this.values;
    }
}
