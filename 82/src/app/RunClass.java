package app;
public class RunClass {
    public static void main(String[] args) {
        // Initialize objects
        FollowLine followLine = new FollowLine();
        ObstacleDetector obstacleDetector = new ObstacleDetector(null, null);
        MotorController motorController = new MotorController(null, null);

        // Initialize threads
        Thread followLineThread = new Thread(followLine);
        Thread obstacleDetectorThread = new Thread(obstacleDetector);
        Thread motorControllerThread = new Thread(motorController);

        // Start threads
        followLineThread.start();
        obstacleDetectorThread.start();
        motorControllerThread.start();

        // Wait for threads to finish
        try {
            followLineThread.join();
            obstacleDetectorThread.join();
            motorControllerThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
