package frc.robot.commands.swerve;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveMonitor;
import edu.wpi.first.wpilibj.Timer;



public class speedAndTimerTerminateCommand extends Command {
    private final SwerveDriveMonitor swerveDriveMonitor;
    private final Timer timer;
    private boolean isStopped;
    private boolean commandFinished;
    

    public speedAndTimerTerminateCommand(SwerveDriveMonitor swerveDriveMonitor) {
        this.swerveDriveMonitor = swerveDriveMonitor;
        this.timer = new Timer();
        this.isStopped = false;
        this.commandFinished = false;
        addRequirements(swerveDriveMonitor);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if (!swerveDriveMonitor.isSwerveDriveMoving()) {
            if (isStopped){
                if (timer.get() > 0.3){
                    commandFinished = true;
                }
            }
            else{
                timer.start();
                isStopped = true;
            }
        } else {
            timer.start();
            isStopped = false;
        }
    }



    @Override
    public boolean isFinished() {
        return commandFinished;
    }

    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

}
