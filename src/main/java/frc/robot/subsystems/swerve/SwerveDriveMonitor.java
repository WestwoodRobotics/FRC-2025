package frc.robot.subsystems.swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveMonitor extends SubsystemBase{
    private SwerveDrive swerveDrive;

    public SwerveDriveMonitor(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public boolean isSwerveDriveMoving(){
        return Math.abs(swerveDrive.getOdometry().getXVel()) > 0.1 &&
               Math.abs(swerveDrive.getOdometry().getYVel()) > 0.1 && 
               Math.abs(swerveDrive.getOdometry().getThetaVel()) > Math.toRadians(3);
    }
    
}
