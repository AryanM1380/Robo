package app;
import java.util.concurrent.atomic.AtomicBoolean;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class MotorController extends Thread {

  private RegulatedMotor motorA = new EV3LargeRegulatedMotor(MotorPort.A);
  private RegulatedMotor motorD = new EV3LargeRegulatedMotor(MotorPort.D);
  private AtomicBoolean  robotStop;
  private AtomicBoolean  robotRotate;

  public MotorController(AtomicBoolean robotStop, AtomicBoolean robotRotate) {
    this.robotStop = robotStop;
    this.robotRotate = robotRotate;
  }

  @Override
  public void run() {
    while (true) {
      if (robotRotate.get()) {
        // Rotate the robot
        motorA.setSpeed(100);
        motorD.setSpeed(100);
        motorA.forward();
        motorD.backward();
        robotRotate.set(false);
      }
      if (robotStop.get()) {
        // Stop the robot
        motorA.stop();
        motorD.stop();
        robotStop.set(false);
      }
      Delay.msDelay(10);
      if (Button.getButtons() != 0) {
        break;
      }
    }
  }

  public void robotRotate() {
    robotRotate.set(true);
  }

  public void robotStop() {
    robotStop.set(true);
  }
}
