package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class pruebasummer extends LinearOpMode {


    private DcMotor MDer;
    private DcMotor MIzq;

    @Override
    public void runOpMode() {


        MIzq  = hardwareMap.get(DcMotor.class, "MIzq");
        MDer = hardwareMap.get(DcMotor.class, "MDer");


        if(opModeIsActive()) {

            MIzq.setDirection(DcMotor.Direction.REVERSE);
            MDer.setDirection(DcMotor.Direction.FORWARD);

            while(opModeIsActive()) {

                MDer.setPower(1);
                MIzq.setPower(1);
                sleep(1000);
                MIzq.setPower(0);
                MDer.setPower(0);
            }

        }


    }
}
