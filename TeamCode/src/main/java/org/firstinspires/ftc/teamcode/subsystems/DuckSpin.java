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
    public double drivePower;
    // power: positive clockwise

    public DuckSpin(Robot robot) {
        spinMotor = robot.getMotor("duck");
    }

    public void start(double power) {
        drivePower = power;
    }

    public void stop() { drivePower=0; }

    public void update(TelemetryPacket packet) {
        spinMotor.setPower(drivePower);
        // todo: should implemnt DuckSpin as a subsystem!!!
    }
}