package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import monologue.Annotations.*;
import monologue.Logged;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.RobotConstants;

public class Shooter extends SubsystemBase implements Logged{
    @Log private double topTarget = 0;
    @Log private double bottomTarget = 0;

    @Log private double topVelocity = 0;
    @Log private double bottomVelocity = 0;

    @Log private double topAppliedVoltage = 0;  
    @Log private double bottomAppliedVoltage = 0;

    @Log private double topCurrent = 0;
    @Log private double bottomCurrent = 0;

    private SparkPIDController topController;
    private SparkPIDController bottomController;

    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;

    private DCMotorSim topMotorSim;
    private DCMotorSim bottomMotorSim;

    private PIDController topSimPID;
    private PIDController bottomSimPID;
    private SimpleMotorFeedforward topSimFF;
    private SimpleMotorFeedforward bottomSimFF;

    private boolean isReal;

    public Shooter(boolean isReal) {
        this.isReal = isReal;
        if(this.isReal) {
            topMotor = new CANSparkMax(ShooterConstants.TOP_ID, MotorType.kBrushless);
            bottomMotor = new CANSparkMax(ShooterConstants.BOTTOM_ID, MotorType.kBrushless);

            topController = topMotor.getPIDController();
            bottomController = bottomMotor.getPIDController();
            topEncoder = topMotor.getEncoder();
            bottomEncoder = bottomMotor.getEncoder();

            topController.setP(ShooterConstants.TOP_PID.kP);
            bottomController.setP(ShooterConstants.BOTTOM_PID.kP);
            topController.setFF(ShooterConstants.TOP_FF.kV);
            bottomController.setFF(ShooterConstants.BOTTOM_FF.kV);
        } else {
            topMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
            bottomMotorSim = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

            topSimPID = new PIDController(ShooterConstants.TOP_PID.kP, ShooterConstants.TOP_PID.kI, ShooterConstants.TOP_PID.kD);
            bottomSimPID = new PIDController(ShooterConstants.BOTTOM_PID.kP, ShooterConstants.BOTTOM_PID.kI, ShooterConstants.BOTTOM_PID.kD);

            topSimFF = new SimpleMotorFeedforward(ShooterConstants.TOP_FF.kS, ShooterConstants.TOP_FF.kV, ShooterConstants.TOP_FF.kA);
            bottomSimFF = new SimpleMotorFeedforward(ShooterConstants.BOTTOM_FF.kS, ShooterConstants.BOTTOM_FF.kV, ShooterConstants.BOTTOM_FF.kA);
        }
    }

    @Override
    public void periodic() {
        topController.setReference(topTarget, ControlType.kVelocity);
        bottomController.setReference(bottomTarget, ControlType.kVelocity);

        topVelocity = topEncoder.getVelocity();
        bottomVelocity = bottomEncoder.getVelocity();

        topAppliedVoltage = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
        bottomAppliedVoltage = bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage();

        topCurrent = topMotor.getOutputCurrent();
        bottomCurrent = bottomMotor.getOutputCurrent();
    }

    @Override
    public void simulationPeriodic() {
        topMotorSim.update(RobotConstants.PERIODIC_LOOP.in(Seconds));
        bottomMotorSim.update(RobotConstants.PERIODIC_LOOP.in(Seconds));

        topCurrent = topMotorSim.getCurrentDrawAmps();
        bottomCurrent = bottomMotorSim.getCurrentDrawAmps();

        topVelocity = topMotorSim.getAngularVelocityRPM();
        bottomVelocity = bottomMotorSim.getAngularVelocityRPM();

        topAppliedVoltage = topSimPID.calculate(topVelocity, topTarget) + topSimFF.calculate(topTarget);
        bottomAppliedVoltage = bottomSimPID.calculate(bottomVelocity, bottomTarget) + bottomSimFF.calculate(bottomTarget);

        topMotorSim.setInputVoltage(topAppliedVoltage);
        bottomMotorSim.setInputVoltage(bottomAppliedVoltage);
    }

    public void setVelocity(Measure<Velocity<Angle>> topTarget, Measure<Velocity<Angle>> bottomTarget){
        this.topTarget = topTarget.in(RPM);
        this.bottomTarget = bottomTarget.in(RPM);
    }
}