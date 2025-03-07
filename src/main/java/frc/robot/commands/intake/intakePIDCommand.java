package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class intakePIDCommand extends Command { 

    private Intake intake;
    private PIDController intakeSidePIDController;
    private PIDController intakeTopPIDController;
    private double sideRollerTargetRPM;
    private double topRollerTargetRPM;

    public intakePIDCommand(Intake intake, double sideRollerTargetRPM, double topRollerTargetRPM){ 
        this.intake = intake;

        this.intakeTopPIDController = intake.getTopPIDController();
        this.sideRollerTargetRPM = sideRollerTargetRPM;
        this.topRollerTargetRPM = topRollerTargetRPM;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intakeSidePIDController.setSetpoint(sideRollerTargetRPM);
        intakeTopPIDController.setSetpoint(topRollerTargetRPM);
    }

    @Override
    public void execute() {

        intake.setTopIntakePower(intakeTopPIDController.calculate(intake.getTopMotorRPM()));
    }

    @Override
    public void end(boolean interrupted){

    }
    
}
