// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled (matches last-year structure)
    public static final double WHEEL_LOCK_TIME = 10.0;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class TestMotorConstants {
    public static final int MOTOR_ID = 18;
    public static final double TEST_POWER = 0.3;
  }

  private Constants() {
    // Utility class
  }
}
