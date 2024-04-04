// Copyright (c) 2024 FRC 9597
// https://github.com/White8848/FRC-9597-2024-Season
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    public static final double MaxSpeed = 5.6; // 6 meters per second desired top speed
    public static final double MaxAngularRate =
        1.7 * Math.PI; // 3/4 of a rotation per second max angular velocity
    public static final double DeadBand = 0.06;
    public static final double RotationalDeadband = 0.06;

    public static final double timestampPeriod = 0.02;

    public static Rotation2d HeadingTarget = new Rotation2d();
  }
}
