package frc.robot;

public class PID {
    public double P;
    public double I;
    public double D;
    public double S;

    public PID (double p, double i, double d) {
        P = p;
        I = i;
        D = d;
    }

    public PID (double p, double i, double d, double s) {
        P = p;
        I = i;
        D = d;
        S = s;
    }
}
