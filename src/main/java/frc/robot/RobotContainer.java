package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

public class RobotContainer {

  private final XboxController driverController = new XboxController(0);
  private final GenericHID opperatorPad = new GenericHID(1);

  private final Indexer Indexer;
  private final Intake Intake;
  private final Shooter Shooter;

  public RobotContainer(boolean isReal) {
    Indexer = new Indexer(isReal);
    Intake = new Intake(isReal);
    Shooter = new Shooter(isReal);

    configureBindings();
  }

  private void configureBindings() {
    Trigger intake = new Trigger(() -> driverController.getRightTriggerAxis() > 0.15);
    intake.onTrue();
  }
}
