// Language: Java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.outtake.OuttakeBeamBreakCommand;
import frc.robot.sensors.DIO.LEDController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.tusks.Tusks;
import frc.robot.subsystems.utils.tusks.tuskPositions;

public class ConditionalTuskBasedIntakeOuttakeCommand extends Command {
  private final Intake m_intake;
  private final Outtake m_outtake;
  private final Tusks m_tusks;
  private final LEDController ledController;

  public ConditionalTuskBasedIntakeOuttakeCommand(Intake intake, Outtake outtake, LEDController ledcontroller, Tusks tusks) {
    this.m_intake = intake;
    this.m_outtake = outtake;
    this.m_tusks = tusks;
    this.ledController = ledcontroller;
    addRequirements(intake, outtake, tusks);
  }

  @Override
  public void initialize() {
    if(m_tusks.getCurrentState() == tuskPositions.IN) {
      new ParallelCommandGroup(
          new InstantCommand(() -> m_intake.setIntakePower(0.4), m_intake)
            .andThen(new OuttakeBeamBreakCommand(m_outtake, ledController, -0.4)),
          new InstantCommand(() -> m_tusks.setRollerPower(0.3), m_tusks)
      ).schedule();
    } else {
      new ParallelCommandGroup(
          new InstantCommand(() -> m_outtake.setOuttakeSpeed(0.3), m_outtake),
          new InstantCommand(() -> m_tusks.setRollerPower(0.3), m_tusks)
      ).schedule();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}