package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

import android.util.Log;


public class Slide implements Subsystem{

    //Hardware: 1 motor, 1 encoder
    private DcMotorEx slideMotor;
    private double slidePower= 0.2;
    public static final double  TICKS_PER_REV = 537.7;
    public static final double PULLEY_DIAMETER = 38.0 / 25.4; //diam convert to inches
    public int level=0;
    public final int MAX_LEVEL=3;
    public static double SLIDE_LENGTH = 15.0;
    private static final double INCHES_PER_LEVEL = 3.0;
    private int targetPosition;
    private boolean needsUpdate = false;
    public Telemetry telemetry;


/*
    public enum slide_state {
        LEVEL_0 (0),
        LEVEL_1 (1),
        LEVEL_2 (2),
        LEVEL_MAX (3);
        public int levVal;
        slide_state(int levVal){
            this.levVal=levVal;
        }
    }*/

    public Slide(Robot robot, Telemetry telemetry) {
        slideMotor = robot.getMotor("slide");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePower = 0.3;
        level = 0;
        this.telemetry = telemetry;
        telemetry.addLine("declared"+level);
        telemetry.update();


    }

    public void setPower ( double power ){
        slidePower = power;
    }

    public int inchToTicks ( double inches) {

        return (int) (inches * TICKS_PER_REV / (PULLEY_DIAMETER * Math.PI));
    }
    public int getLevel() {
        return level;
    }

    public void goUp () {

        if (level < MAX_LEVEL) {

            level = level + 1;
            targetPosition = inchToTicks (INCHES_PER_LEVEL * level);


            needsUpdate=true;

            telemetry.addLine("up: "+ level);
            telemetry.update();



            // set encode to new position
        }

    }

    public void goDown() {
        if (level > 0) {

            level = level - 1;
            targetPosition = inchToTicks (INCHES_PER_LEVEL * level);

            telemetry.addLine("down: "+ level);
            telemetry.update();
            needsUpdate = true;
            // set encode to new position
        }
    }

    public void speedUp(double power){
        slideMotor.setPower(power);
    }

    @Override
    public void update(TelemetryPacket packet) {
        if (needsUpdate) {
            //slideMotor.setTargetPosition(targetPosition);
            //slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //slideMotor.setPower(slidePower);
            needsUpdate=false;
        }

        Log.i("Outtaking", "slideMotorPos:"+slideMotor.getCurrentPosition());

        // debug only,  remove it on release
        // packet.put("Current Position", slideMotor.getCurrentPosition());
       //  packet.put("target position", slideMotor.getTargetPosition());
    }
}

