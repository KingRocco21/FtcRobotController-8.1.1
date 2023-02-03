package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Arm {

    private static double spinnerTargetDegrees;
    private static double armTargetDegrees;
    private static double extenderTargetDistance;

    public Arm(){}

    public static void MoveArmPosition(Position newPosition, Position robotPosition) {
        //Get the vector between the robot's current position and the desired position
        Position pos = new Position(DistanceUnit.INCH, newPosition.x - robotPosition.x, newPosition.y - robotPosition.y, newPosition.z - robotPosition.z, System.nanoTime());
        // Move Arm to an XYZ Coordinate (Inches)
        // Set Spinner Target to the tan of x and y, converting it to degrees
        spinnerTargetDegrees = Math.toDegrees(Math.atan2(pos.y, pos.x));
        // Set Arm Target to tan of of horizontal distance and Z, converting it to degrees
        armTargetDegrees = Math.toDegrees(Math.atan2(pos.z, Math.sqrt(Math.pow(pos.x, 2) + Math.pow(pos.y, 2))));
        // Set Extender Distance using Distance Formula
        extenderTargetDistance = Math.sqrt(Math.pow(pos.x, 2) + Math.pow(pos.y, 2) + Math.pow(pos.z, 2));
    }
    public static double getSpinnerTargetDegrees() {
        MoveArmPosition(AutonomousOdometryPP2022.junctionCoordinates, AutonomousOdometryPP2022.robotPosition);
        return spinnerTargetDegrees;
    }
    public static double getArmTargetDegrees() {
        MoveArmPosition(AutonomousOdometryPP2022.junctionCoordinates, AutonomousOdometryPP2022.robotPosition);
        return armTargetDegrees;
    }
    public static double getExtenderTargetDistance() {
        MoveArmPosition(AutonomousOdometryPP2022.junctionCoordinates, AutonomousOdometryPP2022.robotPosition);
        return extenderTargetDistance;
    }
}
