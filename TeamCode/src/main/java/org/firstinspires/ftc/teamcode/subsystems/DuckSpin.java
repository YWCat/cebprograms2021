package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class DuckSpin implements Subsystem {
    private final DcMotorEx spinMotor;
    public final double drivePower;
    // power: positive clockwise

    public DuckSpin(Robot robot, double power) {
        spinMotor = robot.getMotor("duck");
        drivePower = power;
    }

    public void start() {
        spinMotor.setPower(drivePower);
    }

    public void stop() {
        spinMotor.setPower(0);
    }

    public void update(TelemetryPacket packet) {
        // todo: should implemnt DuckSpin as a subsystem!!!
    }
}