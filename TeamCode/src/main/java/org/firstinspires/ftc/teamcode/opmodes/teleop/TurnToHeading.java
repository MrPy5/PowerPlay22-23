package org.firstinspires.ftc.teamcode.opmodes.teleop;


public class TurnToHeading {

    public void turnToHeading(double targetHeading) {
        boolean turnLeft;
        double currentHeading;
        double degreesToTurn;
        double wheelPower = 0;
        double setWheelPower = 0;
        double accelerationIncrement = 0.03;
        double desiredWheelPower;

        currentHeading = getCurrentHeading();

        degreesToTurn = Math.abs(targetHeading - currentHeading);

        turnLeft = targetHeading > currentHeading;

        if (degreesToTurn > 180) {
            turnLeft = !turnLeft;
            degreesToTurn = 360 - degreesToTurn;
        }

        while (degreesToTurn > .5 && opModeIsActive()) {
            desiredWheelPower = (Math.pow((degreesToTurn) / 35, 4) + 5) / 100;

            if (wheelPower < desiredWheelPower) {
                wheelPower += accelerationIncrement;  // accelerate gradually
                if (wheelPower > desiredWheelPower) {
                    wheelPower = desiredWheelPower;
                }
            } else {
                wheelPower = desiredWheelPower;  // decelerate it immediately
            }

            setWheelPower = wheelPower;
            if (turnLeft) {
                setWheelPower = -setWheelPower;
            }

            Robot.frontLeft.setPower(-setWheelPower);
            Robot.frontRight.setPower(setWheelPower);
            Robot.backLeft.setPower(-setWheelPower);
            Robot.backRight.setPower(setWheelPower);

            currentHeading = getCurrentHeading();
            degreesToTurn = Math.abs(targetHeading - currentHeading);

            turnLeft = targetHeading > currentHeading;
            if (degreesToTurn > 180) {
                turnLeft = !turnLeft;
                degreesToTurn = 360 - degreesToTurn;
            }
        }

        Robot.frontLeft.setPower(0);
        Robot.frontRight.setPower(0);
        Robot.backLeft.setPower(0);
        Robot.backRight.setPower(0);
    }
}