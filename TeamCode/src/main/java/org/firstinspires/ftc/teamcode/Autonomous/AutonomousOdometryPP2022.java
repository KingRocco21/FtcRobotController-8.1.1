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

@Autonomous(name = "AutonomousOdometryPP2022 Java", preselectTeleOp = "TeleOp 2022")
public class AutonomousOdometryPP2022 extends LinearOpMode {

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
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

    public static Position junctionCoordinates;
    public static Position robotPosition;

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
        BL.setPower(-0.3);
        sleep((long) (1000 * forwardSeconds));
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
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
        extender.setDirection(DcMotorSimple.Direction.REVERSE);
        spinner.setTargetPosition(0);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalArm.setTargetPosition(2200);
        verticalArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalArm.setPower(1);
        extender.setPower(1);
        spinner.setPower(1);
        clawLift.setPosition(1);
        claw.setPower(1);

        // Initialize junction position
        junctionCoordinates = new Position(DistanceUnit.INCH, 47, 47, 30, System.nanoTime());

        waitForStart();

        if (opModeIsActive()) {

            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
            globalOdometryPositioning globalPositionUpdate = new globalOdometryPositioning(50, BL, FR, leftEncoder);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            sleep(500);

            claw.setPower(-1);
            sleep(500);
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
                BL.setPower(-0.3);
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
                robotPosition = new Position(DistanceUnit.INCH, globalPositionUpdate.getXCoordinateInches(), globalPositionUpdate.getYCoordinateInches(), 15.75, System.nanoTime());
                Arm.MoveArmPosition(junctionCoordinates, robotPosition, globalPositionUpdate.getOrientationDegrees());
                telemetry.addData("X Inches", globalPositionUpdate.getXCoordinateInches());
                telemetry.addData("Y Inches", globalPositionUpdate.getYCoordinateInches());
                telemetry.update();
                verticalArm.setTargetPosition(armDegreesToTicks((int) Arm.getArmTargetDegrees()));
                sleep(3000);
                spinner.setTargetPosition(spinnerDegreesToTicks((int) Arm.getSpinnerTargetDegrees()));
                extender.setTargetPosition(InchesToTicks((int) Arm.getExtenderTargetDistance()));
                sleep(4000);
                clawLift.setPosition(0.247);
                sleep(1000);
                claw.setPower(1);
                sleep(1000);
                claw.setPower(0);
                clawLift.setPosition(1);
                extender.setTargetPosition(0);
                spinner.setPower(0.3);
                spinner.setTargetPosition(0);
                verticalArm.setPower(0.5);
                verticalArm.setTargetPosition(0);
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
            while (opModeIsActive()) {
                telemetry.addData("X Coordinate Inches", globalPositionUpdate.getXCoordinateInches());
                telemetry.addData("Y Coordinate Inches", globalPositionUpdate.getYCoordinateInches());
                telemetry.addData("Orientation Degrees", globalPositionUpdate.getOrientationDegrees());
                telemetry.addData("Extender", extender.getCurrentPosition());
                telemetry.update();
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
        BL.setPower(0.5);
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
        BL.setPower(-0.5);
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
        BL.setPower(0.5);
        sleep((long) (1000 * rightSeconds));
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    private int spinnerDegreesToTicks(int Degrees) {
        return Degrees * (1000 / 90);
    }

    private int armDegreesToTicks(int Degrees) {
        return Degrees * (-1705/80);
    }

    private int InchesToTicks(int Inches) {
        return (Inches * (2361/20));
    }
}