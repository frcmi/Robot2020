package frc.robot.lib.jetsoninterface;
import frc.robot.Robot;

// Stores x and y which represent the distance forward to the board and the
// sideways distance respectively, and the
// sideways angle of the board in radians (counterclockwise)
public class Delta {
    public double x;
    public double y;
    public double theta;
    public long timeStamp;

    public Delta(double x, double y, double theta, long timeStamp) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.timeStamp = timeStamp;
    }

    public void print() {
        Robot.debugStream.println("Delta object:");
        Robot.debugStream.println("    x=" + x);
        Robot.debugStream.println("    y=" + y);
        Robot.debugStream.println("    theta=" + theta);
        Robot.debugStream.println("    timeStamp=" + timeStamp);
    }
}