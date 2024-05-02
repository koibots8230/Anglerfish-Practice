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
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RobotConstants;

public class Indexer extends SubsystemBase implements Logged{
    @Log private double target = 0;
    @Log private double velocity = 0;
    @Log private double appliedVoltage = 0;
    @Log private double current = 0;

    private SparkPIDController realController;
    private PIDController simPID;
    private SimpleMotorFeedforward simFF;

    private CANSparkMax motor;
    private DCMotorSim simMotor;
    private RelativeEncoder encoder;

    private boolean isReal;

    public Indexer(boolean isReal) {
        this.isReal = isReal;
        if(this.isReal) {
            motor = new CANSparkMax(IndexerConstants.ID, MotorType.kBrushless);
            encoder = motor.getEncoder();

            realController = motor.getPIDController();
            realController.setP(IndexerConstants.PID.kP);
            realController.setFF(IndexerConstants.FF.kV);
        } else {
            simMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
            simPID = new PIDController(IndexerConstants.PID.kP, IndexerConstants.PID.kI, IndexerConstants.PID.kD);
            simFF = new SimpleMotorFeedforward(IndexerConstants.FF.kS, IndexerConstants.FF.kV, IndexerConstants.FF.kA);
        }
    }

    @Override
    public void periodic() {
        realController.setReference(target, ControlType.kVelocity);

        velocity = encoder.getVelocity();
        appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        current = motor.getOutputCurrent();
    }

    @Override
    public void simulationPeriodic() {
        simMotor.update(RobotConstants.PERIODIC_LOOP.in(Seconds));

        current = simMotor.getCurrentDrawAmps();
        velocity = simMotor.getAngularVelocityRPM();
        appliedVoltage = simPID.calculate(velocity, target) + simFF.calculate(target);

        simMotor.setInputVoltage(appliedVoltage);
    }

    public void setVelocity(Measure<Velocity<Angle>> target) {
        this.target = target.in(RPM);
    }
}
