package frc.robot.commands.swerve;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveMonitor;
import edu.wpi.first.wpilibj.Timer;



public class speedAndTimerTerminateCommand extends Command {
    private final SwerveDriveMonitor swerveDriveMonitor;
    private double counter;
    

    public speedAndTimerTerminateCommand(SwerveDriveMonitor swerveDriveMonitor) {
        this.swerveDriveMonitor = swerveDriveMonitor;
        this.counter = 0;
        addRequirements(swerveDriveMonitor);
    }

    @Override
    public void initialize() {
        this.counter = 0;
    }

    @Override
    public void execute() {
        
        if (!swerveDriveMonitor.isSwerveDriveMoving()) {
            this.counter++;
        }
    }



    @Override
    public boolean isFinished() {
        return this.counter > 5;
    }

    public void end(boolean interrupted) {
        
    }

}
