package com.apexpathing.util.math;

/**
 * Units class with conversion methods
 * @author Sohum Arora 22985 Paraducks
 */
public class Units {
    public enum DistanceUnits {
        INCHES,
        CENTIMETRES,
        METRES,
        MILLIMETRES,
        FEET
    }
    public static double toInches(DistanceUnits unit, double value) {
        switch (unit) {
            case INCHES: return value;
            case CENTIMETRES: return value / 2.54;
            case METRES: return value / 0.0254;
            case MILLIMETRES: return value / 25.4;
            case FEET: return value * 12;
            default: return value;
        }
    }

    public static double toMetres(DistanceUnits unit, double value) {
        switch (unit) {
            case METRES: return value;
            case INCHES: return value * 0.0254;
            case CENTIMETRES: return value / 100;
            case MILLIMETRES: return value / 1000;
            case FEET: return value * 0.3048;
            default: return value;
        }
    }

    public static double toCentimetres(DistanceUnits unit, double value) {
        switch (unit) {
            case CENTIMETRES: return value;
            case INCHES: return value * 2.54;
            case METRES: return value * 100;
            case MILLIMETRES: return value / 10;
            case FEET: return value * 30.48;
            default: return value;
        }
    }

    public static double toMillimetres(DistanceUnits unit, double value) {
        switch (unit) {
            case MILLIMETRES: return value;
            case INCHES: return value * 25.4;
            case METRES: return value * 1000;
            case CENTIMETRES: return value * 10;
            case FEET: return value * 304.8;
            default: return value;
        }
    }

    public static double toFeet(DistanceUnits unit, double value) {
        switch (unit) {
            case FEET: return value;
            case INCHES: return value / 12;
            case METRES: return value / 0.3048;
            case CENTIMETRES: return value / 30.48;
            case MILLIMETRES: return value / 304.8;
            default: return value;
        }

    }
    public enum AngleUnits {
        DEGREES, RADIANS
    }

    public static double toDegrees(double value) {
        return Math.toDegrees(value);
    }

    public static double toRadians(double value) {
        return Math.toRadians(value);
    }
}