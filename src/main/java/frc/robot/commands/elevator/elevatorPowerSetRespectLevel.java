package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.utils.elevator.elevatorPositions;

public class elevatorPowerSetRespectLevel extends Command{
    private double l2Power;
    private double l3Power;
    private double l4Power;
    private double homePower;
    private double interruptedPower;
    private Elevator elevator;
    private Outtake outtake;
    private elevatorPositions currentPosition;

    public elevatorPowerSetRespectLevel(Elevator elevator, Outtake outtake){
        this.elevator = elevator;
        this.outtake = outtake;
        addRequirements(elevator, outtake);
    }

    @Override
    public void initialize(){
        currentPosition = elevator.getElevatorPositionEnum();
        l2Power = -0.20;
        l3Power = -0.20;
        l4Power = -0.35;
        homePower = -0.20;
        interruptedPower = -0.35;

        if (currentPosition == elevatorPositions.L2){
            outtake.setOuttakeSpeed(l2Power);
        }
        else if (currentPosition == elevatorPositions.L3){
            outtake.setOuttakeSpeed(l3Power);
        }
        else if (currentPosition == elevatorPositions.INTERRUPTED){
            outtake.setOuttakeSpeed(interruptedPower);
        } else if (currentPosition == elevatorPositions.L4){
            outtake.setOuttakeSpeed(l4Power);
        }
        else if (currentPosition == elevatorPositions.HOME) {
            outtake.setOuttakeSpeed(homePower);
        }




    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        outtake.setOuttakeSpeed(0);
    }



}
