package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class globalOdometryPositioning implements Runnable{

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor leftEncoder;

    //Thead run condition
    private boolean isRunning = true;

    //Files to access the algorithm constants
    private File robotEncoderWheelDistanceFile = AppUtil.getInstance().getSettingsFile("robotEncoderWheelDistance.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    // Algorithm Constants
    private double robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(robotEncoderWheelDistanceFile).trim());
    private double horizontalTickOffsetRadians = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    private final int countsPerInch = 1875;

    // Position Variables
    private volatile double robotXCoordinate = 35.25; // LEFT STARTING VALUES
    private volatile double robotYCoordinate = 22031.25; // LEFT STARTING VALUES
    private volatile double robotOrientationRadians = 0;
    private double previousLeftEncoderPosition = 0;
    private double previousRightEncoderPosition = 0;
    private double previousNormalEncoderPosition = 0;
    private volatile double robotXCoordinateInches = 0;
    private volatile double robotYCoordinateInches = 0;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public globalOdometryPositioning(int threadSleepDelay) {
        sleepTime = threadSleepDelay;
    }
    /**
    * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
    */
    private void globalPositionUpdate() {
        // Save the change in each encoder position to a variable
        double leftChange = leftEncoder.getCurrentPosition() - previousLeftEncoderPosition;
        double rightChange = BL.getCurrentPosition() - previousRightEncoderPosition;
        // Calculate Angle
        double changeInRobotOrientation = (leftChange - rightChange) / robotEncoderWheelDistance;
        robotOrientationRadians += changeInRobotOrientation;
        // Calculate the horizontal change using the middle encoder
        double rawHorizontalChange = FR.getCurrentPosition() - previousNormalEncoderPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation * horizontalTickOffsetRadians);
        double verticalChange = (rightChange + leftChange) / 2;
        // Calculate Position
        robotXCoordinate += verticalChange * Math.sin(robotOrientationRadians) + horizontalChange * Math.cos(robotOrientationRadians);
        robotYCoordinate += verticalChange * Math.cos(robotOrientationRadians) - horizontalChange * Math.sin(robotOrientationRadians);
        // Converts the X and Y coordinates to inches
        robotXCoordinateInches = robotXCoordinate / countsPerInch;
        robotYCoordinateInches = robotYCoordinate / countsPerInch;
        // Save the position values to variables for use in the next loop
        previousLeftEncoderPosition = leftEncoder.getCurrentPosition();
        previousRightEncoderPosition = BL.getCurrentPosition();
        previousNormalEncoderPosition = FR.getCurrentPosition();
    }
    public double getXCoordinate() {
        return robotXCoordinate;
    }
    public double getYCoordinate() {
        return robotYCoordinate;
    }
    public double getXCoordinateInches() {
        return robotXCoordinateInches;
    }
    public double getYCoordinateInches() {
        return robotYCoordinateInches;
    }
    public double getOrientationRadians() {
        return robotOrientationRadians;
    }
    public double getOrientationDegrees() {
        return Math.toDegrees(robotOrientationRadians) % 360;
    }
    /**
     * Stops the position update thread
     */
    public void stop() {
        isRunning = false;
    }
    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalPositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
