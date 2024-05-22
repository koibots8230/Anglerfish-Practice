package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.controller;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import monologue.Monologue;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

        private final Shooter shooterSubsystem;
        private final Intake intakeSubsystem;
        private final Indexer indexerSubsystem;

        private final IntakeCommand intakeCommand;

        Trigger speakerShooterTrigger;

        public RobotContainer(boolean isReal) {
                shooterSubsystem = new Shooter(isReal);
                intakeSubsystem = new Intake(isReal);
                indexerSubsystem = new Indexer(isReal);

                intakeCommand = new IntakeCommand(indexerSubsystem, intakeSubsystem);

                Monologue.setupMonologue(
                                this, "Robot", Constants.LoggerConstants.FILEONLY,
                                Constants.LoggerConstants.LAZYLOGGING);
                configureBindings();
        }

        private void configureBindings() {

                Trigger intakeTrigger = new Trigger(() -> controller.CONTROLLER.getRightTriggerAxis() > 0.15);
                intakeTrigger.onTrue(intakeCommand);
                intakeTrigger.onFalse(new ParallelCommandGroup(
                                new InstantCommand(() -> indexerSubsystem.setIndexerVelocity(0.0), indexerSubsystem),
                                new InstantCommand(() -> intakeSubsystem.setIntakeVelocity(0.0), intakeSubsystem)));

                Trigger reverse = new Trigger(controller.CONTROLLER::getRightBumper);
                reverse.onTrue(new ParallelCommandGroup(
                                new InstantCommand(
                                                () -> intakeSubsystem
                                                                .setIntakeVelocity(-(PIDConstants.INTAKE_SETPOINT)),
                                                intakeSubsystem),
                                new InstantCommand(
                                                () -> indexerSubsystem
                                                                .setIndexerVelocity(-(PIDConstants.INDEXER_SETPOINT)),
                                                indexerSubsystem)));
                reverse.onFalse(new ParallelCommandGroup(
                                new InstantCommand(() -> intakeSubsystem.setIntakeVelocity(0.0), intakeSubsystem),
                                new InstantCommand(() -> indexerSubsystem.setIndexerVelocity(0.0), indexerSubsystem)));

                speakerShooterTrigger = new Trigger(() -> controller.CONTROLLER.getRawButton(1));
                speakerShooterTrigger.onTrue(new ParallelCommandGroup(
                                new InstantCommand(
                                                () -> shooterSubsystem.setVelocity(
                                                                PIDConstants.TOP_SHOOTER_SPEAKER_SETPOINT,
                                                                PIDConstants.BOTTOM_SHOOTER_SPEAKER_SETPOINT),
                                                shooterSubsystem)));

                speakerShooterTrigger.onFalse(new ParallelCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setVelocity(0.0, 0.0), shooterSubsystem)));

                Trigger ampShooterTrigger = new Trigger(() -> controller.OPERATOR_CONTROLLER.getRawButton(5));
                ampShooterTrigger.onTrue(new ParallelCommandGroup(
                                new InstantCommand(
                                                () -> shooterSubsystem.setVelocity(
                                                                PIDConstants.TOP_SHOOTER_AMP_SETPOINT,
                                                                PIDConstants.BOTTOM_SHOOTER_AMP_SETPOINT),
                                                shooterSubsystem)));
                ampShooterTrigger.onFalse(new ParallelCommandGroup(
                                new InstantCommand(() -> shooterSubsystem.setVelocity(0.0,
                                                0.0), shooterSubsystem)));

                Trigger sendToShooterTrigger = new Trigger(() -> controller.CONTROLLER.getRawButton(2) &&
                                shooterSubsystem.checkVelocity());
                sendToShooterTrigger.onTrue(new ParallelCommandGroup(
                                new InstantCommand(
                                                () -> indexerSubsystem.setIndexerVelocity(
                                                                PIDConstants.SEND_TO_SHOOTER_SETPOINT),
                                                indexerSubsystem)));
                sendToShooterTrigger.onFalse(new InstantCommand(() -> indexerSubsystem.setIndexerVelocity(0.0)));
        }

}
