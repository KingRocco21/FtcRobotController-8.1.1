package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@Autonomous(name = "OdometryTest Java", preselectTeleOp = "TeleOp 2022")
public class OdometryTest extends LinearOpMode {

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

    double robotEncoderWheelDistance;
    double horizontalTickOffsetRadians;

    public static Position junctionCoordinates;
    public static Position robotPosition;

    //Files to access the algorithm constants
    private File robotEncoderWheelDistanceFile = AppUtil.getInstance().getSettingsFile("robotEncoderWheelDistance.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

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
        junctionCoordinates = new Position(DistanceUnit.INCH, 5, 5, 30, System.nanoTime());

        waitForStart();

        if (opModeIsActive()) {

            //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
            globalOdometryPositioning globalPositionUpdate = new globalOdometryPositioning(50, BL, FR, leftEncoder);
            Thread positionThread = new Thread(globalPositionUpdate);
            positionThread.start();

            sleep(500);

            extender.setPower(0.5);
            spinner.setPower(0.2);
            verticalArm.setPower(1);
            robotPosition = new Position(DistanceUnit.INCH, globalPositionUpdate.getXCoordinateInches(), globalPositionUpdate.getYCoordinateInches(), 15.75, System.nanoTime());
            Arm.MoveArmPosition(junctionCoordinates, robotPosition, globalPositionUpdate.getOrientationDegrees());
            verticalArm.setTargetPosition(armDegreesToTicks((int) Arm.getArmTargetDegrees()));
            sleep(3000);
            spinner.setTargetPosition(spinnerDegreesToTicks((int) Arm.getSpinnerTargetDegrees()));
            extender.setTargetPosition(InchesToTicks((int) Arm.getExtenderTargetDistance()));
            sleep(4000);
            clawLift.setPosition(0.247);
            telemetry.addData("X Coordinate Inches", globalPositionUpdate.getXCoordinateInches());
            telemetry.addData("Y Coordinate Inches", globalPositionUpdate.getYCoordinateInches());
            telemetry.addData("Orientation Degrees", globalPositionUpdate.getOrientationDegrees());
            telemetry.addData("Extender", extender.getCurrentPosition());
            telemetry.update();

            verticalArm.setTargetPosition(0);
            clawLift.setPosition(1);
            claw.setPower(0);
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