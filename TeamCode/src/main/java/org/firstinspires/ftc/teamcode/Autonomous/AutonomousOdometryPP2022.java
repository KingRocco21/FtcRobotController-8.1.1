package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@Autonomous(name = "AutonomousOdometryPP2022 Blocks to Java", preselectTeleOp = "TeleOp")
public class AutonomousOdometryPP2022 extends LinearOpMode {

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
    private BNO055IMU imu;
    private DcMotor leftEncoder;
    private DcMotor extender;
    private DcMotor spinner;
    private DcMotor verticalArm;
    private Servo clawLift;
    private ColorSensor color1;

    BNO055IMU.Parameters storedValues;
    int countsPerInch;
    float wheelPowerTlBr;
    int robotEncoderWheelDistance;
    int robotXCoordinate;
    double horizontalTickOffsetRadians;
    int robotYCoordinate;
    int previousLeftEncoderPosition;
    int robotOrientationRadians;
    int previousRightEncoderPosition;
    int robotXCoordinateInches;
    int previousNormalEncoderPosition;

    /**
     * Describe this function...
     */
    private void moveForward(double forwardSeconds) {
        FR.setPower(0.3);
        FL.setPower(0.3);
        BR.setPower(0.3);
        BL.setPower(0.3);
        sleep((long) (1000 * forwardSeconds));
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    /**
     * Describe this function...
     */
    private float getZAngle() {
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Describe this function...
     */
    private void Initialize_IMU() {
        BNO055IMU.Parameters IMUparameters;

        // Create a new IMU Parameters Object
        IMUparameters = new BNO055IMU.Parameters();
        // Set IMU mode to IMU so it callibrates itself
        IMUparameters.mode = BNO055IMU.SensorMode.IMU;
        // Use Degrees as angle unit
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Use meters/sec as unit of acceleration
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Warn driver this may take several seconds!
        telemetry.addData("Status", "init IMU, please wait");
        telemetry.update();
        // Init IMU with these parameters
        imu.initialize(IMUparameters);
        telemetry.addData("Status", "IMU initialized");
        telemetry.update();
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        extender = hardwareMap.get(DcMotor.class, "extender");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        verticalArm = hardwareMap.get(DcMotor.class, "verticalArm");
        clawLift = hardwareMap.get(Servo.class, "clawLift");
        color1 = hardwareMap.get(ColorSensor.class, "color1");

        // Initialize variables.
        countsPerInch = 1875;
        robotEncoderWheelDistance = 12026;
        horizontalTickOffsetRadians = 12194;
        previousLeftEncoderPosition = 0;
        previousRightEncoderPosition = 0;
        previousNormalEncoderPosition = 0;
        robotOrientationRadians = 0;
        robotXCoordinate = 0;
        robotYCoordinate = 0;
        // Initialize motors and encoders.
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        extender.setTargetPosition(0);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalArm.setTargetPosition(2200);
        verticalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalArm.setPower(1);
        extender.setPower(1);
        spinner.setPower(1);
        clawLift.setPosition(1);
        // Init the IMU (will reset Z orientation to 0)
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Odometry();
            }
            // end
        }
    }

    /**
     * Describe this function...
     */
    private void CreateStorageFile() {
        storedValues = new BNO055IMU.Parameters();
        storedValues.i2cAddr = I2cAddr.create7bit(robotXCoordinate);
        storedValues.i2cAddr = I2cAddr.create8bit(robotYCoordinate);
        // Inputs the parameters into a file
    }

    /**
     * Describe this function...
     */
    private void moveBackward(double backwardSeconds) {
        FR.setPower(-0.5);
        FL.setPower(-0.5);
        BR.setPower(-0.5);
        BL.setPower(-0.5);
        sleep((long) (1000 * backwardSeconds));
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void moveLeft(double leftSeconds) {
        FR.setPower(0.5);
        FL.setPower(-0.5);
        BR.setPower(-0.5);
        BL.setPower(0.5);
        sleep((long) (1000 * leftSeconds));
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void moveRight(double rightSeconds) {
        FR.setPower(-0.5);
        FL.setPower(0.5);
        BR.setPower(0.5);
        BL.setPower(-0.5);
        sleep((long) (1000 * rightSeconds));
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void calibrateOdometry() {
        int encoderDifference;
        float verticalEncoderTickOffsetPerDegree;
        double wheelBaseSeparation;

        BL.setPower(-0.2);
        BR.setPower(0.2);
        FL.setPower(0.2);
        FR.setPower(-0.2);
        while (getZAngle() < 90 && opModeIsActive()) {
        }
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        sleep(1000);
        encoderDifference = Math.abs(leftEncoder.getCurrentPosition()) + Math.abs(BL.getCurrentPosition());
        verticalEncoderTickOffsetPerDegree = encoderDifference / getZAngle();
        wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * countsPerInch);
        robotEncoderWheelDistance = (int) (wheelBaseSeparation * countsPerInch);
        horizontalTickOffsetRadians = FR.getCurrentPosition() / (getZAngle() * (Math.PI / 180));
        telemetry.addData("IMU Angle", getZAngle());
        telemetry.addData("Left Encoder", -leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder", BL.getCurrentPosition());
        telemetry.addData("Middle Encoder", FR.getCurrentPosition());
        telemetry.addData("WheelBaseSeparation", wheelBaseSeparation);
        telemetry.addData("horizontalTickOffsetRadians", horizontalTickOffsetRadians);
        telemetry.update();
        sleep(1000000);
    }

    /**
     * Describe this function...
     */
    /*private void TeleOpRead() {
        BNO055IMU.Parameters storedValues2;

        // Reads from the imu's file and puts our coordinates into imu parameters
        storedValues.calibrationDataFile = "storedValues.json";
        robotXCoordinate = storedValues.i2cAddr.get7Bit();
        robotYCoordinate = storedValues.i2cAddr.get8Bit();
        // Reads from the imu's second file and does the same thing
        storedValues2.calibrationDataFile = "storedValues2.json";
        robotOrientationRadians = storedValues2.i2cAddr.get7Bit();
    }
*/
    /**
     * Describe this function...
     */
    private void Odometry() {
        double leftChange;
        double rightChange;
        double changeInRobotOrientation;
        int rawHorizontalChange;
        double horizontalChange;
        double verticalChange;
        int robotYCoordinateInches;

        // Save the change in each encoder position to a variable
        leftChange = leftEncoder.getCurrentPosition() - previousLeftEncoderPosition;
        rightChange = BL.getCurrentPosition() - previousRightEncoderPosition;
        // Calculate Angle
        changeInRobotOrientation = (leftChange - rightChange) / robotEncoderWheelDistance;
        robotOrientationRadians += changeInRobotOrientation;
        // Calculate the horizontal change using the middle encoder
        rawHorizontalChange = FR.getCurrentPosition() - previousNormalEncoderPosition;
        horizontalChange = rawHorizontalChange - changeInRobotOrientation * horizontalTickOffsetRadians;
        verticalChange = (rightChange + leftChange) / 2;
        // Calculate Position
        robotXCoordinate += verticalChange * Math.sin(robotOrientationRadians / 180 * Math.PI) + horizontalChange * Math.cos(robotOrientationRadians / 180 * Math.PI);
        robotYCoordinate += verticalChange * Math.cos(robotOrientationRadians / 180 * Math.PI) - horizontalChange * Math.sin(robotOrientationRadians / 180 * Math.PI);
        // Converts the X and Y coordinates to inches
        robotXCoordinateInches = robotXCoordinate / countsPerInch;
        robotYCoordinateInches = robotYCoordinate / countsPerInch;
        // Save the position values to variables for use in the next loop
        previousLeftEncoderPosition = leftEncoder.getCurrentPosition();
        previousRightEncoderPosition = BL.getCurrentPosition();
        previousNormalEncoderPosition = FR.getCurrentPosition();
        telemetry.addData("robotXCoordinate", robotXCoordinate);
        telemetry.addData("robotYCoordinate", robotYCoordinate);
        telemetry.addData("robotXCoordinateInches", robotXCoordinateInches);
        telemetry.addData("robotYCoordinateInches", robotYCoordinateInches);
        telemetry.addData("robotOrientationRadians", robotOrientationRadians);
        telemetry.addData("robotOrientationDegrees", robotOrientationRadians * (180 / Math.PI));
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void goToPosition(int xInches, int yInches, int motorPower, int desiredOrientation) {
        direction_factor(getZAngle() - desiredOrientation);
        while (xInches != robotXCoordinateInches && yInches != robotXCoordinateInches) {
        }
        FL.setPower(wheelPowerTlBr);
    }

    /**
     * Describe this function...
     */
    private void direction_factor(float heading) {
        float wheelPowerTrBl;

        if (heading >= 0 && heading < 90) {
            wheelPowerTlBr = 1;
            wheelPowerTrBl = heading * -4 + 1;
        } else if (heading >= 90 && heading < 180) {
            wheelPowerTrBl = -1;
            wheelPowerTlBr = heading * -4 + 3;
        } else if (heading >= 180 && heading < 270) {
            wheelPowerTlBr = -1;
            wheelPowerTrBl = heading * 4 - 5;
        } else if (heading >= 270 && heading < 360) {
            wheelPowerTrBl = 1;
            wheelPowerTlBr = heading * 4 - 7;
        }
    }
}