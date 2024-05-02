package frc.lib.util;

import frc.robot.Robot;

public class PIDClass {
    public final double kP;
    public final double kI;
    public final double kD;

    public PIDClass(double realkP, double realkI, double realkD, double simkP, double simkI, double simkD) {
        if(Robot.isReal()) {
            this.kP = realkP;
            this.kI = realkI;
            this.kD = realkD;
        } else {
            this.kP = simkP;
            this.kI = simkI;
            this.kD = simkD;
        }
    }
}
