package frc.robot.commands.swerve;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveMonitor;
import edu.wpi.first.wpilibj.Timer;



public class speedAndTimerTerminateCommand extends Command {
    private final SwerveDriveMonitor swerveDriveMonitor;
    private final Timer isStoppedTimer;
    private final Timer initTimer;
    private boolean isStopped;
    private boolean commandFinished;
    

    public speedAndTimerTerminateCommand(SwerveDriveMonitor swerveDriveMonitor) {
        this.swerveDriveMonitor = swerveDriveMonitor;
        this.isStoppedTimer = new Timer();
        this.initTimer = new Timer();
        this.isStopped = false;
        this.commandFinished = false;
        addRequirements(swerveDriveMonitor);
    }

    @Override
    public void initialize() {
        initTimer.start();
    }

    @Override
    public void execute() {
        if (initTimer.get() < 0.3) {
            return;
        }
        if (!swerveDriveMonitor.isSwerveDriveMoving()) {
            if (isStopped){
                if (isStoppedTimer.get() > 0.3){
                    commandFinished = true;
                }
            }
            else{
                isStoppedTimer.start();
                isStopped = true;
            }
        } else {
            isStoppedTimer.start();
            isStopped = false;
        }
    }



    @Override
    public boolean isFinished() {
        return commandFinished;
    }

    public void end(boolean interrupted) {
        isStoppedTimer.stop();
        isStoppedTimer.reset();
        initTimer.stop();
        initTimer.reset();
        commandFinished = false;
        isStopped = false;
        
    }

}
