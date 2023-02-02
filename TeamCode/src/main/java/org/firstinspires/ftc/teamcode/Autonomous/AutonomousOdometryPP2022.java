package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@Autonomous(name = "AutonomousOdometryPP2022 Java", preselectTeleOp = "TeleOp")
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
    private CRServo claw;
    private ColorSensor color1;

    float wheelPowerTlBr;
    double robotEncoderWheelDistance;
    double horizontalTickOffsetRadians;
    double robotXCoordinate;
    double robotYCoordinate;
    double robotOrientationRadians;
    double previousLeftEncoderPosition;
    double previousRightEncoderPosition;
    double previousNormalEncoderPosition;
    double robotXCoordinateInches;
    double robotYCoordinateInches;

    Position junctionCoordinates;
    Position robotPosition;

    //Files to access the algorithm constants
    private File robotEncoderWheelDistanceFile = AppUtil.getInstance().getSettingsFile("robotEncoderWheelDistance.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

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
     * This function Initializes the IMU to get the Z Angle later.
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
        ElapsedTime Timer;
        int colorNumber = 0;

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
        claw = hardwareMap.get(CRServo.class, "claw");
        color1 = hardwareMap.get(ColorSensor.class, "color1");

        // Initialize variables.
        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(robotEncoderWheelDistanceFile).trim());
        horizontalTickOffsetRadians = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        previousLeftEncoderPosition = 0;
        previousRightEncoderPosition = 0;
        previousNormalEncoderPosition = 0;
        robotOrientationRadians = 0;
        robotXCoordinate = 0;
        robotYCoordinate = 0;
        robotXCoordinateInches = 0;
        robotYCoordinateInches = 0;

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
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
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
        claw.setPower(1);

        // Initialize robot position and junction position
        junctionCoordinates = new Position(DistanceUnit.INCH, 47, 23.5, 16.25, System.nanoTime());

        waitForStart();

        if (opModeIsActive()) {
            //robotPosition = new Position(DistanceUnit.INCH, robotXCoordinateInches, robotYCoordinateInches, 0, System.nanoTime());
            //spinner.setTargetPosition(spinnerDegreesToTicks((int) Arm.getSpinnerTargetDegrees()));
            //verticalArm.setTargetPosition(-armDegreesToTicks((int) Arm.getArmTargetDegrees()));
            //extender.setTargetPosition(InchesToTicks((int) Arm.getExtenderTargetDistance()));

            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
            globalOdometryPositioning globalPositionUpdate = new globalOdometryPositioning(50, BL, FR, leftEncoder);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            sleep(500);

            claw.setPower(-1);
            BL.setPower(-0.3);
            BR.setPower(0.3);
            FL.setPower(0.3);
            FR.setPower(0.3);
            Timer = new ElapsedTime();
            Timer.reset();
            while (opModeIsActive() && ((DistanceSensor) color1).getDistance(DistanceUnit.INCH) >= 1 && Timer.seconds() <= 3) {
            }
            BL.setPower(0);
            BR.setPower(0);
            FL.setPower(0);
            FR.setPower(0);
            sleep(1000);
            if (opModeIsActive() && ((DistanceSensor) color1).getDistance(DistanceUnit.INCH) <= 1) {
                if (opModeIsActive() && color1.red() > color1.blue() && color1.red() > color1.green()) {
                    colorNumber = 1;
                } else if (opModeIsActive() && color1.green() > color1.blue() && color1.green() > color1.red()) {
                    colorNumber = 2;
                } else if (opModeIsActive() && color1.blue() > color1.red() && color1.blue() > color1.green()) {
                    colorNumber = 3;
                }
                BL.setPower(0.3);
                BR.setPower(0.3);
                FL.setPower(0.3);
                FR.setPower(0.3);
                sleep(2000);
                BL.setPower(0);
                BR.setPower(0);
                FL.setPower(0);
                FR.setPower(0);
                sleep(1000);
                extender.setPower(0.5);
                spinner.setPower(0.2);
                verticalArm.setPower(1);
                verticalArm.setTargetPosition(-2296);
                sleep(4000);
                extender.setTargetPosition(-976);
                spinner.setTargetPosition(1317);
                clawLift.setPosition(0.27);
                sleep(5000);
                claw.setPower(1);
                sleep(1000);
                claw.setPower(0);
                clawLift.setPosition(1);
                extender.setTargetPosition(0);
                spinner.setPower(0.3);
                spinner.setTargetPosition(0);
                verticalArm.setPower(0.5);
                verticalArm.setTargetPosition(2200);
                sleep(4000);
                if (colorNumber == 1) {
                    moveForward(0.8);
                    moveBackward(0.2);
                    moveLeft(1.4);
                    moveBackward(0.5);
                } else if (colorNumber == 3) {
                    moveForward(0.4);
                    moveBackward(0.2);
                    moveRight(1.5);
                    moveBackward(0.5);
                }
            }
            verticalArm.setTargetPosition(2200);
            clawLift.setPosition(1);
            claw.setPower(0);
            while (opModeIsActive()) {
                telemetry.addData("X Coordinate Inches", robotXCoordinateInches);
                telemetry.addData("Y Coordinate Inches", robotYCoordinateInches);
                telemetry.addData("Orientation Degrees", (Math.toDegrees(robotOrientationRadians) % 360));

            }
            //Stop the thread
            globalPositionUpdate.stop();
        }
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

    private int spinnerDegreesToTicks(int Degrees) {
        return Degrees * (2030 / 180);
    }

    private int armDegreesToTicks(int Degrees) {
        return Degrees * (977 / 45);
    }

    private int InchesToTicks(int Inches) {
        return -(Inches * 113);
    }
}