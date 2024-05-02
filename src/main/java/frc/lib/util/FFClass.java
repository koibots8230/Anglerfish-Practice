package frc.lib.util;

import frc.robot.Robot;

public class FFClass {
    public final double kS;
    public final double kV;
    public final double kA;
    public final double kG;

    public FFClass(double realkS, double realkV, double realkA, double realkG, double simkS, double simkV, double simkA, double simkG) {
        if(Robot.isReal()) {
            this.kS = realkS;
            this.kV = realkV;
            this.kA = realkA;
            this.kG = realkG;
        } else {
            this.kS = simkS;
            this.kV = simkV;
            this.kA = simkA;
            this.kG = simkG;
        }
    }
}
