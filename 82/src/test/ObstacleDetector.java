package test;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import java.util.concurrent.atomic.AtomicBoolean;
import lejos.hardware.Sound;

public class ObstacleDetector extends Thread {

    AtomicBoolean robotStop;
    AtomicBoolean robotrotate;
    RegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
    RegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
    SampleProvider sp = new EV3UltrasonicSensor(SensorPort.S1).getDistanceMode();
    int distanceValue = 0;

    public ObstacleDetector(AtomicBoolean robotStop, AtomicBoolean robotrotate) {
        this.robotStop = robotStop;
        this.robotrotate = robotrotate;
    }

    @Override
    public void run() {

        while (true) {
            System.out.println("Distance: " + distanceValue);
            float[] sample = new float[sp.sampleSize()];
            sp.fetchSample(sample, 0);
            distanceValue = (int) (sample[0] * 100);
            if (distanceValue <= 20) {
                robotStop.set(true);
                Sound.buzz();
                int[] frequencies = { 440, 494, 523, 587, 659, 698, 784 }; // Frequencies in Hz
                int[] durations = { 500, 500, 500, 500, 500, 500, 1000 }; // Durations in milliseconds

                for (int i = 0; i < frequencies.length; i++) {
                    Sound.playTone(frequencies[i], durations[i]);
                    try {
                        Thread.sleep(durations[i]);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                robotrotate.set(true);

                // Move forward for a short distance to clear the obstacle
                leftMotor.setSpeed(200);
                rightMotor.setSpeed(200);
                leftMotor.forward();
                rightMotor.forward();
                Delay.msDelay(1000);

                // Turn left to avoid the obstacle
                leftMotor.setSpeed(200);
                rightMotor.setSpeed(200);
                leftMotor.backward();
                rightMotor.forward();
                Delay.msDelay(1000);

                // Move forward again to continue along the path
                leftMotor.setSpeed(200);
                rightMotor.setSpeed(200);
                leftMotor.forward();
                rightMotor.forward();
                Delay.msDelay(2000);

                // Stop the motors and reset the robotStop and robotrotate flags
                leftMotor.stop();
                rightMotor.stop();
                robotStop.set(false);
                robotrotate.set(false);
            }
            Delay.msDelay(10);
            if (Button.getButtons() != 0) {
                break;
            }
        }
    }
}
}