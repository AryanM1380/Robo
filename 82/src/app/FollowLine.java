package app;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor; 
import lejos.robotics.SampleProvider;

public class FollowLine extends Thread {

    private static final float MAX_DISTANCE = 0.3f;
    private static final float COLOR_THRESHOLD = 0.15f;
    
    private EV3ColorSensor colorSensor;
    private EV3UltrasonicSensor ultrasonicSensor;
    private RegulatedMotor motorA;
    private RegulatedMotor motorD;
    
    private SampleProvider colorProvider;
    private SampleProvider distanceProvider;
    
    private float[] colorSample;
    private float[] distanceSample;
    
    public FollowLine() {
        colorSensor = new EV3ColorSensor(SensorPort.S4);
        ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        
        motorA = new EV3LargeRegulatedMotor(MotorPort.A);
        motorD = new EV3LargeRegulatedMotor(MotorPort.D);
        
        colorProvider = colorSensor.getRedMode();
        distanceProvider = ultrasonicSensor.getDistanceMode();
        
        colorSample = new float[colorProvider.sampleSize()];
        distanceSample = new float[distanceProvider.sampleSize()];
    }
    
    public void run() {
        while (true) {
            // Read the color sensor value
            colorProvider.fetchSample(colorSample, 0);
            float colorValue = colorSample[0];
            
            // Determine the motor speeds based on the color sensor value
            if (colorValue <= COLOR_THRESHOLD) {
                motorA.setSpeed(150);
                motorD.setSpeed(50);
                motorA.forward();
                motorD.forward();
            } else {
                motorD.setSpeed(150);
                motorA.setSpeed(50);
                motorD.forward();
                motorA.forward();
            }
            
            // Check for obstacle
            distanceProvider.fetchSample(distanceSample, 0);
            float distanceValue = distanceSample[0];
            if (distanceValue < MAX_DISTANCE) {
                System.out.println("Obstacle detected at distance: " + distanceValue);
                // Stop the robot
                motorA.stop();
                motorD.stop();
                // Back up and turn
                motorA.setSpeed(100);
                motorD.setSpeed(100);
                motorA.backward();
                motorD.backward();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                motorA.stop();
                motorD.stop();
                motorA.setSpeed(50);
                motorD.setSpeed(-50);
                motorA.forward();
                motorD.forward();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            
            if (Button.getButtons() != 0) {
                break;
            }
        }
    }
    
    public void close() {
        colorSensor.close();
        ultrasonicSensor.close();
        motorA.close();
        motorD.close();
    }
    
    public static void main(String[] args) {
        FollowLine robot = new FollowLine();
        Thread robotThread = new Thread(robot);
        robotThread.start();
        while (true) {
            if (Button.getButtons() != 0) {
                robot.close();
                break;
            }
        }
    }
}
