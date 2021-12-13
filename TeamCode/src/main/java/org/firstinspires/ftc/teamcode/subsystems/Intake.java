package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

import android.util.Log;

public class Intake implements Subsystem {
    //Hardware: 1 motor, 1 encoder
    private DcMotorEx armMotor;
    private DcMotorEx sweeper;
    private static final double TICKS_PER_REV = 7168; // 28 * 256 = 7168

    //PID Stuff
    private PIDFController armPID;
    private static final PIDCoefficients ARM_PID_COEFFICIENTS = new PIDCoefficients(0.5, 0, 0);

    private static final double ARM_ACCEPTABLE_ERROR_MARGIN = 0.1;

    private int counter = 0;

    public enum Positions {
        RESET,
        INTAKE,
        DUMP
    }

    public Intake(Robot robot) {
        armMotor = robot.getMotor("armMotor");
        sweeper = robot.getMotor("sweeper");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armPID = new PIDFController(ARM_PID_COEFFICIENTS);
        //In order for the PID controller to find the most efficient way to go to the target position,
        //we need to bound the error inputted to the PID controller from -pi to pi radians
        armPID.setInputBounds(-Math.PI, Math.PI);
    }

    public void setTargetAngle(double targetAngle) {
        armPID.reset();
        armPID.setTargetPosition(targetAngle);
    }

    private void setTargetPosition(Positions position) {
        switch (position) {
            case RESET:
                setTargetAngle(0 * Math.PI / 180);
                break;
            case INTAKE:
                setTargetAngle(-120 * Math.PI / 180);
                break;
            case DUMP:
                setTargetAngle(20 * Math.PI / 180);
                break;
        }
    }

    public void reset() {
        setTargetPosition(Positions.RESET);
    }

    public void start () {
        setTargetPosition(Positions.INTAKE);
        if (counter>=5){
            sweeper.setPower(0.8);
        }
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Log.i("Intaking, start", "sweeper:"+ sweeper.getPower()+"armMotor:"+armMotor.getPower()+
                " counter:"+counter+" TargetAngle:"+ armPID.getTargetPosition());
    }
    public void stop () {
        sweeper.setPower(0);
        setTargetPosition(Positions.DUMP);
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Log.i("Intaking, stop", "sweeper:"+ sweeper.getPower()+"armMotor:"+armMotor.getPower()+
                " counter:"+counter+" TargetAngle:"+ armPID.getTargetPosition());
    }

    private double getRawArmAngle() {
        // encoder position * (2pi / TICKS_PER_REV)
        return armMotor.getCurrentPosition() * (2 * Math.PI / TICKS_PER_REV);
    }

    // should be private, but use public as some old opmode needs it
    public double getArmAngle() {
        return Angle.norm(getRawArmAngle());
    }

    // should be private, but use public as some old opmode needs it
    public double getPIDError() { return armPID.getLastError(); }

    // should be private, but use public as some old opmode needs it
    public boolean targetReached() {
        return Math.abs(armPID.getLastError()) <= ARM_ACCEPTABLE_ERROR_MARGIN;
    }

    @Override
    public void update(TelemetryPacket packet) {
        double armPower = armPID.update(getArmAngle());
        armMotor.setPower(armPower);
        if (armPID.getLastError() <= ARM_ACCEPTABLE_ERROR_MARGIN){
            counter++;
        }
        else{
            counter=0;
        }
    }
}
