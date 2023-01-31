package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

public class OdometryPipeline {

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor leftEncoder;
    private DcMotor extender;
    private DcMotor spinner;
    private DcMotor verticalArm;

    //Files to access the algorithm constants
    private File robotEncoderWheelDistanceFile = AppUtil.getInstance().getSettingsFile("robotEncoderWheelDistance.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int countsPerInch;
    private double robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(robotEncoderWheelDistanceFile).trim());
    private double horizontalTickOffsetRadians = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
    private double robotXCoordinate;
    private double robotYCoordinate;
    private double robotOrientationRadians;
    private double previousLeftEncoderPosition;
    private double previousRightEncoderPosition;
    private double previousNormalEncoderPosition;
    private double robotXCoordinateInches;
    private double robotYCoordinateInches;

    public OdometryPipeline(int CPI) {
        countsPerInch = CPI;
    }

    /**
     * This function calculates the x and y position of the robot, along with its orientation.
     */
    public void Odometry() {
        double leftChange;
        double rightChange;
        double changeInRobotOrientation;
        double rawHorizontalChange;
        double horizontalChange;
        double verticalChange;

        // Save the change in each encoder position to a variable
        leftChange = leftEncoder.getCurrentPosition() - previousLeftEncoderPosition;
        rightChange = BL.getCurrentPosition() - previousRightEncoderPosition;
        // Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / robotEncoderWheelDistance;
        robotOrientationRadians += changeInRobotOrientation;
        // Calculate the horizontal change using the middle encoder
        rawHorizontalChange = FR.getCurrentPosition() - previousNormalEncoderPosition;
        horizontalChange = rawHorizontalChange - (changeInRobotOrientation * horizontalTickOffsetRadians);
        verticalChange = (rightChange + leftChange) / 2;
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
}
