package com.ApexPathing.drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

public class TankDrive extends Drivetrain{

    boolean FourWheelDrive;
    public TankDrive(HardwareMap hardwareMap, Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName
                     ) { //2wd tank drive constructor
        super(hardwareMap,telemetry, useBrakeMode, leftFrontName, rightFrontName, leftFrontName, rightFrontName);
        this.FourWheelDrive = false;
    }
    public TankDrive(HardwareMap hardwareMap,
                     Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName,
                     @NotNull String leftRearName,
                     @NotNull String rightRearName) { //4wd tank
        super(hardwareMap, telemetry, useBrakeMode, leftFrontName, rightFrontName, leftRearName, rightRearName);
        this.FourWheelDrive = true;
    }

    @Override
    public void drive(double x, double y, double turn) {
        double left  = y + turn;
        double right = y - turn;

        left = Range.clip(left, -1, 1);
        right= Range.clip(right, -1, 1);

        if (!FourWheelDrive) {
            setPower(leftFront, left);
            setPower(rightFront, right);
        } else {
            setPower(leftFront, left);
            setPower(rightFront, right);
            setPower(leftRear, left);
            setPower(rightRear, right);
        }
    }
    @Override
    public void driveFieldCentric(double x, double y, double turn, double heading) {
        //no field centric in tank :(
    }
}
