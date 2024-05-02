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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;

public class Intake extends SubsystemBase implements Logged{
    @Log private double setpoint = 0;
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

    public Intake(boolean isReal) {
        this.isReal = isReal;
        if(this.isReal) {
            motor = new CANSparkMax(IntakeConstants.ID, MotorType.kBrushless);
            encoder = motor.getEncoder();

            realController = motor.getPIDController();
            realController.setP(IntakeConstants.PID.kP);
            realController.setFF(IntakeConstants.FF.kV);
        } else {
            simMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);
            simPID = new PIDController(IntakeConstants.PID.kP, IntakeConstants.PID.kI, IntakeConstants.PID.kD);
            simFF = new SimpleMotorFeedforward(IntakeConstants.FF.kS, IntakeConstants.FF.kV, IntakeConstants.FF.kA);
        }
    }

    @Override
    public void periodic() {
        realController.setReference(setpoint, ControlType.kVelocity);

        velocity = encoder.getVelocity();
        appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        current = motor.getOutputCurrent();
    }

    @Override
    public void simulationPeriodic() {
        simMotor.update(RobotConstants.PERIODIC_LOOP.in(Seconds));

        current = simMotor.getCurrentDrawAmps();
        velocity = simMotor.getAngularVelocityRPM();
        appliedVoltage = simPID.calculate(velocity, setpoint) + simFF.calculate(setpoint);

        simMotor.setInputVoltage(appliedVoltage);
    }

    public void setVelocity(Measure<Velocity<Angle>> setpoint) {
        this.setpoint = setpoint.in(RPM);
    }
}
