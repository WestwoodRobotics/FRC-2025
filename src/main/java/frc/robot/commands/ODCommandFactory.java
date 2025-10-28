

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.outtake.OuttakeBeamBreakCommand;
import frc.robot.commands.tusks.tuskSetPositionCommand;
import frc.robot.commands.tusks.tusksHoldCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.tusks.Tusks;
import frc.robot.subsystems.utils.tusks.tuskPositions;
import frc.robot.sensors.DIO.LEDController;
import frc.robot.subsystems.elevator.Elevator;

public class ODCommandFactory {

    private final Intake m_intake;
    private final Outtake m_outtake;
    private final Elevator m_elevator;
    private final Tusks m_tusks;
    private final LEDController ledController;
    private XboxController controller;


    public ODCommandFactory(Intake m_intake, Outtake m_outtake, Elevator m_elevator, Tusks m_tusks, LEDController ledController, XboxController controller){
        this.m_intake = m_intake;
        this.m_outtake = m_outtake;
        this.m_elevator = m_elevator;
        this.m_tusks = m_tusks;
        this.ledController = ledController;
        this.controller = controller;
    }

    public Command IntakeToOuttakeBeamBreakCommand(){

        return new InstantCommand(() -> m_intake.setBothPowers(0.5, 0.75), m_intake)
        .andThen(new OuttakeBeamBreakCommand(m_outtake, ledController, controller, 1, -0.4)
        );

    }

    public Command scoreCoral(){
        return new InstantCommand(() -> m_outtake.setOuttakeSpeed(-0.85));
    }

    public Command intake(tuskPositions position){
        
        //tuskPositions current_position = this.m_tusks.getCurrentState();
        tuskPositions current_position = position;

        return new ParallelCommandGroup(
            new InstantCommand(() -> m_outtake.setOuttakeSpeed(0.3), m_outtake),
            new InstantCommand(() -> m_tusks.setRollerPower(0.3), m_tusks)
        );
        

    }

    public Command  stopIntake(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.setIntakePower(0), m_intake),
            new InstantCommand(() -> m_tusks.stopRoller(), m_tusks),
            new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)
        );
    }

    public Command holdTuskPivot () {
        return new tuskSetPositionCommand(m_tusks, tuskPositions.HOME);
    }
    

    


}

