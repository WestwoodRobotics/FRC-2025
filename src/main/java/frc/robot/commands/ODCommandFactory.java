

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
import frc.robot.subsystems.tusks.Tusks;
import frc.robot.subsystems.utils.tusks.tuskPositions;
import frc.robot.commands.outtake.OuttakeBeamBreakCommand;
import frc.robot.subsystems.elevator.Elevator;

public class ODCommandFactory {

    private final Intake m_intake;
    private final Outtake m_outtake;
    private final Elevator m_elevator;
    private final Tusks m_tusks;


    public ODCommandFactory(Intake m_intake, Outtake m_outtake, Elevator m_elevator, Tusks m_tusks){
        this.m_intake = m_intake;
        this.m_outtake = m_outtake;
        this.m_elevator = m_elevator;
        this.m_tusks = m_tusks;
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

    public Command intake(tuskPositions position){
        
        //tuskPositions current_position = this.m_tusks.getCurrentState();
        tuskPositions current_position = position;

        if (current_position == tuskPositions.IN){
            
        } 

        return new ParallelCommandGroup(
            new InstantCommand(() -> m_outtake.setOuttakeSpeed(0.3), m_outtake),
            new InstantCommand(() -> m_tusks.setRollerPower(0.3), m_tusks)
        );
        

    }

    public Command  stopIntake(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> m_intake.setIntakePower(0), m_intake),
            new InstantCommand(() -> m_tusks.setRollerPower(0), m_tusks),
            new InstantCommand(() -> m_outtake.setOuttakeSpeed(0), m_outtake)
        );
    }


}

