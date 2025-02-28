package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.DIO.LEDController;
import frc.robot.subsystems.outtake.Outtake;

public class OuttakeBeamBreakCommand extends Command {
  
  private enum CoralState {
    WAITING_FOR_FRONT_CORAL,            // Haven't seen the coral yet; spin forward
    WAITING_FOR_NO_CORAL_AFTER_FRONT, // Coral’s front has passed beam, but not the rest
    WAITING_FOR_BACK_CORAL,             // Coral’s back not yet detected; spin reverse
    DONE                          // We’ve seen the back of the coral; stop
  }

  private final Outtake outtake;

  private double power;
  private double enumStartVal;
  private CoralState state;
  private LEDController leds;
  private Timer timer;
  private double debounceCounter;
  private boolean isElevator;

  public OuttakeBeamBreakCommand(Outtake outtake, LEDController leds, double eumStartVal, double power) {
    this.outtake = outtake;
    this.power = power;
    this.enumStartVal = enumStartVal;

    this.leds = leds;
    addRequirements(outtake);
  }

  public OuttakeBeamBreakCommand(Outtake outtake,  LEDController leds, double power) {
    this.outtake = outtake;
    this.power = power;
    this.leds = leds;
    this.enumStartVal=-1;
    debounceCounter = 0;

    addRequirements(outtake);
  }

  public OuttakeBeamBreakCommand(Outtake outtake, LEDController leds, double power, boolean isElevator) {
    this.outtake = outtake;
    this.power = power;
    this.leds = leds;
    this.enumStartVal=-1;
    this.isElevator = isElevator;
    debounceCounter = 0;

    addRequirements(outtake);
  }

  @Override
  public void initialize() {
    this.timer = new Timer();

    if(isElevator && !outtake.isCoralDetected()){
      state = CoralState.DONE;
      return;
    }

    if(enumStartVal > 0){
      state = CoralState.WAITING_FOR_NO_CORAL_AFTER_FRONT;
    }
    state = CoralState.WAITING_FOR_FRONT_CORAL;
    debounceCounter = 0;

  }

  @Override
  public void execute() {
    
    switch (state) {
      case WAITING_FOR_FRONT_CORAL:
        // Spin forward until the beam breaks (we detect coral)
        outtake.setOuttakeSpeed(power);
        leds.blinkin.set(0.91);//violet
        if (outtake.isCoralDetected()) {
          debounceCounter++;
        }
        if (debounceCounter >= 2) {
          // The front of the coral just passed
          debounceCounter = 0;
          state = CoralState.WAITING_FOR_NO_CORAL_AFTER_FRONT;
        }
        break;

      case WAITING_FOR_NO_CORAL_AFTER_FRONT:
        // Still spin forward until coral has fully passed -> beam unbroken
        if(enumStartVal > 0 && !outtake.isCoralDetected()){
          state = CoralState.DONE;
          return;
        }
        outtake.setOuttakeSpeed(0.3*power);
        
        
        if (outtake.isCoralNotDetected()) {
          // Entire coral just passed the sensor; reverse direction
          state = CoralState.WAITING_FOR_BACK_CORAL;
        }
        break;

      case WAITING_FOR_BACK_CORAL:
        // Spin in reverse until the beam breaks again (back of coral)
        outtake.setOuttakeSpeed(-0.4*power);
        leds.blinkin.set(0.87); //blue
        if (outtake.isCoralDetected()) {
          // We just detected the back of the coral
          outtake.setOuttakeSpeed(0);
          state = CoralState.DONE;
        }
        break;

      case DONE:
        // No more action needed
        outtake.setOuttakeSpeed(0);
        leds.blinkin.set(0.77); //green
        if (!timer.isRunning()){
          timer.restart();
        }
        break;
    }
  }

  @Override
  public boolean isFinished(){
    if (enumStartVal<0){
      return state == CoralState.DONE;
    }
    return state == CoralState.DONE && timer.get() > 0.5;

  }
  @Override
  public void end(boolean interrupted) {
    outtake.setOuttakeSpeed(0);
  }
}

