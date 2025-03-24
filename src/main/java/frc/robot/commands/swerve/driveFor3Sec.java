package frc.robot.commands.swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class driveFor3Sec extends Command {

    private final SwerveDrive m_swerveDrive;
    private XboxController controller;
    private boolean slowMode;
    private boolean isYuMode;
    private PIDController rotationPIDController;
    private double targetHeading;
    private boolean isRotInput;
    private Timer timer;
  
    /**
     * Constructs a new driveCommand.
     * 
     * @param swerveDrive The swerve drive subsystem used by this command.
     * @param controller The Xbox controller used to control the swerve drive.
     */
    public driveFor3Sec(SwerveDrive swerveDrive, XboxController controller) {
      m_swerveDrive = swerveDrive;
      this.controller = controller;
      timer = new Timer();
      
      addRequirements(swerveDrive);
  
    }

    @Override
    public void execute(){
        m_swerveDrive.drive(1, 0, 0, false);
    }

    @Override
    public void end( boolean isTerminated){
        m_swerveDrive.drive(0, 0, 0, false);
    }
}