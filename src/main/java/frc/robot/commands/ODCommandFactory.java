// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.RepeatCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;

// import frc.robot.commands.shooter.shooterPIDCommand;
// import frc.robot.subsystems.Shooter.Shooter;
// import frc.robot.subsystems.Shooter.preRoller;
// import frc.robot.subsystems.intake.Intake;

// /**
//  * The ODCommandFactory class is responsible for creating various commands for the robot's subsystems.
//  * It provides methods to create commands for the intake, preRoller, and shooter subsystems.
//  */
// public class ODCommandFactory {
//     private final Intake m_intake;
//     private final preRoller m_preRoller;
//     private final Shooter m_shooter;
//     private static final Timer m_timer = new Timer();

//     /**
//      * Constructs a new ODCommandFactory.
//      * 
//      * @param m_intake The intake subsystem used by this factory.
//      * @param m_preRoller The preRoller subsystem used by this factory.
//      * @param m_shooter The shooter subsystem used by this factory.
//      */
//     public ODCommandFactory(Intake m_intake, preRoller m_preRoller, Shooter m_shooter){
//         this.m_intake = m_intake;
//         this.m_preRoller = m_preRoller;
//         this.m_shooter = m_shooter;
//     }

//     /**
//      * Creates a command to sense the intake and set the intake power.
//      * 
//      * @return The command to sense the intake and set the intake power.
//      */
//     public Command intakeSenseCommand(){
//         return new preRollerSenseCommand(m_preRoller, 6180, 0.1, 10, 30).alongWith(new InstantCommand(()->m_intake.setIntakePower(0.8)));
//     }

//     /**
//      * Creates a command to stop the intake and preRoller.
//      * 
//      * @return The command to stop the intake and preRoller.
//      */
//     public Command stopIntakeSenseCommand(){
//         return new InstantCommand(() -> m_intake.stopIntake(), m_intake).alongWith(new InstantCommand(() -> m_preRoller.stopPreRoller(), m_preRoller));
//     }
    
//     /**
//      * Creates a command to rev up the shooter and shoot.
//      * 
//      * @param power The power to set for the shooter.
//      * @param targetRPM The target RPM for the shooter.
//      * @return The command to rev up the shooter and shoot.
//      */
//     public Command revUpAndShootCommand(double power, double targetRPM){
//         return new shooterPIDCommand(m_shooter, power, targetRPM)
//         .andThen(new InstantCommand(() -> m_preRoller.setPreRollerPower(0.7)))
//         .andThen(new WaitCommand(0.6))
//         .andThen(new InstantCommand(()-> m_shooter.setShooterPower(1)).alongWith(new InstantCommand(() -> m_preRoller.stopPreRoller())));
//     }

//     public Command revUpAndShootCommandAuton(double power, double targetRPMUpper, double targetRPMLower){
//         return new shooterPIDCommand(m_shooter, power, power, targetRPMUpper, targetRPMLower)
//         .andThen(new InstantCommand(() -> m_preRoller.setPreRollerPower(0.7)))
//         .andThen(new WaitCommand(0.6))
//         .andThen(new InstantCommand(()-> m_shooter.setShooterPower(1)).alongWith(new InstantCommand(() -> m_preRoller.stopPreRoller())));
//     }

//     /**
//      * Creates a command to stop the shooter and preRoller.
//      * 
//      * @return The command to stop the shooter and preRoller.
//      */
//     public Command stopShooterCommand(){
//         //return new InstantCommand(() -> m_shooter.stopShooter()).andThen(new InstantCommand(() -> m_preRoller.stopPreRoller()));
//         return new InstantCommand(() -> m_shooter.stopShooter(), m_shooter).andThen(new InstantCommand(() -> m_preRoller.stopPreRoller(), m_preRoller)); 
//     }

//     /**
//      * Creates a command to stop the preRoller.
//      * 
//      * @return The command to stop the preRoller.
//      */
//     public Command stopPreRollerCommand(){
//         return new InstantCommand(() -> m_preRoller.stopPreRoller(), m_preRoller);
//     }

//     /**
//      * Creates a command to stop the intake.
//      * 
//      * @return The command to stop the intake.
//      */
//     public Command stopIntakeCommand(){
//         return new InstantCommand(() -> m_intake.stopIntake(), m_intake);
//     }

//     /**
//      * Creates a command to rev up the shooter.
//      * 
//      * @return The command to rev up the shooter.
//      */
//     public Command revUpShooter(){
//         return new InstantCommand(() -> m_shooter.setShooterPower(0.8));
//     }

//     /**
//      * Creates a command to fire a note using the preRoller.
//      * 
//      * @return The command to fire a note using the preRoller.
//      */
//     public Command fireNote(){
//         return new InstantCommand(() -> m_preRoller.setPreRollerPower(0.5)).andThen(new WaitCommand(0.75)).andThen(new InstantCommand(() -> m_preRoller.stopPreRoller(), m_preRoller));
//     }

//     /**
//      * Creates a command to stop all subsystems (shooter, intake, and preRoller).
//      * 
//      * @return The command to stop all subsystems.
//      */
//     public Command stopAllCommand(){
//         return new InstantCommand(() -> m_shooter.stopShooter(), m_shooter).alongWith(this.stopIntakeSenseCommand());
//     }

//     public void startTimer(){
//         m_timer.start();
//     }

//     public void stopTimer(){
//         m_timer.stop();
//     }

//     public void resetTimer(){
//         m_timer.reset();
//     }

//     public double getTimer(){
//         return m_timer.get();
//     }


//     public Command checkAutoAndShoot(){
//         if (m_timer.get() > 14.5){
//             return stopAllCommand();
//         }
//         return revUpAndShootCommand(0.75, 4000);
//     }

// }
