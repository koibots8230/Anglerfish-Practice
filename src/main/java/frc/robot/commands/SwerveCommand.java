package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Swerve;

public class SwerveCommand extends Command {

    public final Swerve swerve;
    public final DoubleSupplier leftX;
    public final DoubleSupplier leftY;
    public final DoubleSupplier rightX;
    
    public SwerveCommand(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, Swerve swerve){
        this.swerve = swerve;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        addRequirements(swerve);

    }

    @Override
    public void execute() {
        double xInput = leftX.getAsDouble();
        double yInput = leftY.getAsDouble();

        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xInput, yInput), 0.025);

        Rotation2d linearDirection = new Rotation2d(xInput, yInput);

        linearMagnitude *= linearMagnitude * Math.signum(linearMagnitude);

        double thetaInput = rightX.getAsDouble();

        thetaInput *= thetaInput * thetaInput;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            linearMagnitude * linearDirection.getCos() * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond),
            linearMagnitude * linearDirection.getSin() * RobotConstants.MAX_LINEAR_SPEED.in(MetersPerSecond), 
            thetaInput * RobotConstants.MAX_ANGULAR_SPPEd.in(RadiansPerSecond), 
            swerve.getGyroAngle()
        );

        swerve.driveRobotRelative(speeds);
    }
}
