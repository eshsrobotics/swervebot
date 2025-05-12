package frc.robotSimulator;

import frc.robot.*;
public class Main {
    public static void main(String[] args) {
        System.out.println("Hello, Robot Simulator!");
        var a = new RobotCore();
        System.out.println(a.getInfo());
    }
}
