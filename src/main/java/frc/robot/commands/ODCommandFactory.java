

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.commands.outtake.OuttakeBeamBreakCommand;
import frc.robot.subsystems.elevator.Elevator;

public class ODCommandFactory {

    private final Intake m_intake;
    private final Outtake m_outtake;
    private final Elevator m_elevator;


    public ODCommandFactory(Intake m_intake, Outtake m_outtake, Elevator m_elevator){
        this.m_intake = m_intake;
        this.m_outtake = m_outtake;
        this.m_elevator = m_elevator;
    }

    public Command IntakeToOuttakeBeamBreakCommand(){

        return new SequentialCommandGroup(
            new InstantCommand(() -> m_intake.setIntakePower(0.8), m_intake),
            new OuttakeBeamBreakCommand(m_outtake, -0.3)
        );  
    }

    public InstantCommand scoreCoral(){
       return new InstantCommand(() -> m_outtake.setOuttakeSpeed(-0.5));
    }


}

