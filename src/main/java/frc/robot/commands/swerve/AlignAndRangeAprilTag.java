package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.utils.LimelightHelpers;

public class AlignAndRangeAprilTag extends Command {
    private final SwerveDrive swerve;
    private final PIDController rotationPID;
    private final PIDController rangePID;
    private final String limelightName;
    private static final double TARGET_RANGE = 1.0; // 1 meter target distance
    
    public AlignAndRangeAprilTag(SwerveDrive swerve, String limelightName) {
        this.swerve = swerve;
        this.limelightName = limelightName;
        
        // Tune these PID values for your robot
        rotationPID = new PIDController(0.015, 0.0, 0.001);
        rangePID = new PIDController(0.7, 0.0, 0.01);
        
        // Set tolerance for both controllers
        rotationPID.setTolerance(1.0); // 1 degree tolerance
        rangePID.setTolerance(0.05); // 5cm tolerance
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        rangePID.reset();
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(limelightName)) {
            // No valid target
            swerve.drive(0, 0, 0, true);
            return;
        }

        // Get horizontal offset from target
        double tx = LimelightHelpers.getTX(limelightName);
        
        // Get distance from target using 3D pose data
        double currentRange = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getZ();

        // Calculate control outputs
        double rotationOutput = rotationPID.calculate(tx, 0.0);
        double rangeOutput = rangePID.calculate(currentRange, TARGET_RANGE);

        // Apply control outputs to robot
        // Forward/backward movement for range control
        // Left/right movement is zero
        // Rotation is for horizontal alignment
        swerve.drive(-rangeOutput, 0, -rotationOutput, true);
    }

    @Override
    public boolean isFinished() {
        // Command finishes when we're at the target distance and aligned
        // return LimelightHelpers.getTV(limelightName) && 
        //        rotationPID.atSetpoint() && 
        //        rangePID.atSetpoint();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, true);
    }

    
}