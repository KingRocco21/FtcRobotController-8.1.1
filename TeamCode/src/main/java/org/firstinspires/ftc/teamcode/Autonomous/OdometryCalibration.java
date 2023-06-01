package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@Autonomous(name = "OdometryCalibration")
public class OdometryCalibration extends LinearOpMode {

    private DcMotor FR;
    private DcMotor FL;
    private DcMotor BR;
    private DcMotor BL;
    private BNO055IMU imu;
    private DcMotor leftEncoder;

    int countsPerInch = 1875;
    double robotEncoderWheelDistance;
    double horizontalTickOffsetRadians;

    ElapsedTime timer = new ElapsedTime();

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File robotEncoderWheelDistanceFile = AppUtil.getInstance().getSettingsFile("robotEncoderWheelDistance.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private float getZRadians() {
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
    }

    private float getZAngle() {
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    private void Initialize_IMU() {
        BNO055IMU.Parameters IMUparameters;

        // Create a new IMU Parameters Object
        IMUparameters = new BNO055IMU.Parameters();
        // Set IMU mode to IMU so it calibrates itself
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

    private void calibrateOdometry() {
        double encoderDifference;
        double verticalEncoderTickOffsetPerDegree;
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
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
        }
        encoderDifference = Math.abs(leftEncoder.getCurrentPosition()) + Math.abs(BL.getCurrentPosition());
        verticalEncoderTickOffsetPerDegree = encoderDifference / getZAngle();
        wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * countsPerInch);
        robotEncoderWheelDistance = wheelBaseSeparation * countsPerInch;
        horizontalTickOffsetRadians = FR.getCurrentPosition() / getZRadians();

        //Write the constants to text files
        ReadWriteFile.writeFile(robotEncoderWheelDistanceFile, String.valueOf(robotEncoderWheelDistance));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffsetRadians));

        //Display calculated constants
        telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
        telemetry.addData("Horizontal Encoder Offset", horizontalTickOffsetRadians);

        //Display raw values
        telemetry.addData("IMU Angle", getZAngle());
        telemetry.addData("Vertical Left Position", -leftEncoder.getCurrentPosition());
        telemetry.addData("Vertical Right Position", BL.getCurrentPosition());
        telemetry.addData("Horizontal Position", FR.getCurrentPosition());
        telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

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

        // Init the IMU (will reset Z orientation to 0, will take 2 seconds)
        Initialize_IMU();
        waitForStart();
        if (opModeIsActive()) {
            calibrateOdometry();
            while (opModeIsActive()) {
            }
            // end
        }
    }
}