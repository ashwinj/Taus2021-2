
//Run code for the left blue position

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name = "RedLeft", group = "Taus")
public class RedLeft extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(0);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 15);
                shoot(0.0, (2214.2*28)/60.0);
                forward(.5, 0, 13);
                strafeRight(.5, 1, 0);
                toAngle(180, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 0);
                toAngle(0, 1);
                break;
            case 1:
                //code
                forward(.5, 2, 16);
                shoot(1.0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 15);
                toAngle(180, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                forward(.5, 2, 15);
                toAngle(270, 1);
                setIntakePower(0);
                backward(.5, 0, 12);
                forward(.5, 0, 12);
                toAngle(360, 1);
                forward(.5, 0, 23);
                shoot(4.0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
            case 4:
                //code
                forward(.5, 2, 16);
                shoot(1.0, (2214.2*28)/60.0);
                toAngle(0, 1);
                forward(.5, 2, 8);
                strafeRight(.5, 1, 0);
                toAngle(165, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 4);
                toAngle(180, 1);
                forward(.5, 3, 2);
                toAngle(90, 1);
                setIntakePower(1);
                backward(.5, 0, 10);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 1);
                backward(.5, 0, 2);
                forward(.5, 0, 6);
                shoot(-12.0, (2358.0*28)/60.0);
                toAngle(-90, 1);
                backward(.5, 0, 11);
                forward(.5, 0, 17);
                toAngle(0, 1);
                forward(.5, 0, 23);
                shoot(4.0, (2214.0*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 12);
                break;
        }

    }
}
