package frc.robot.commands.swerve;

import javax.sound.midi.Track;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.utils.LimelightHelpers;

enum TrackingState5 {
    SEARCHING,
    LOCKED_ON,
    TARGET_LOST
}

public class SeekAndTrackRotOnly extends Command {
    private final SwerveDrive swerve;
    private final String limelightName;
    private final PIDController rotationPID;
    private final double SEARCH_ROTATION_SPEED = 0.2; // Adjust this for search speed
    private TrackingState5 state;
    private final double lostTimeOut = 1;
    private Timer timer;
    public SeekAndTrackRotOnly(SwerveDrive swerve, String limelightName) {
        this.swerve = swerve;
        this.limelightName = limelightName;
        this.state = TrackingState5.SEARCHING;
        this.timer = new Timer();
        
        // Tune PID values for your robot
        rotationPID = new PIDController(0.01, 0.0, 0.00);
        rotationPID.setTolerance(1.0); // 1 degree tolerance
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        timer.reset();
        
        this.state = TrackingState5.SEARCHING;
    }

    @Override
    public void execute() {
        switch(this.state) {
            case SEARCHING:
                swerve.drive(0, 0, SEARCH_ROTATION_SPEED, true);
                if (LimelightHelpers.getTV(limelightName)) {
                    this.state = TrackingState5.LOCKED_ON;

                }
            break;
            case TARGET_LOST:
                if (timer.hasElapsed(lostTimeOut)){
                    this.state = TrackingState5.SEARCHING;
                } 
                
                else if (LimelightHelpers.getTV(limelightName)) {
                    this.state = TrackingState5.LOCKED_ON;
                }
                swerve.drive(0,0,0,true);
            break;
            case LOCKED_ON:
                if (!LimelightHelpers.getTV(limelightName)) {
                    this.state = TrackingState5.TARGET_LOST;
                    this.timer.reset();
                    this.timer.start();
                }
                    // Get horizontal offset from target
                double tx = LimelightHelpers.getTX(limelightName);
                
                // Calculate rotation to track target
                double rotationOutput = rotationPID.calculate(tx, 0.0);
                
                // Apply rotation only - no translation
                swerve.drive(0, 0, 
                rotationOutput, true);
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own - must be interrupted
        return false;
    }
}