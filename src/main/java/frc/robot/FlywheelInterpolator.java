package frc.robot;

public class FlywheelInterpolator {
    public static int findRangeIdx(double distance) { // in terms of meters
        // if the input is past the largest
        // recording it defaults to the biggest one for interpolating
        int idx = -1;
        double min = 0;
        double max;

        if (distance < Constants.flywheelRPMData[0][1]) {
            return 0; // return early
        }
        if (distance > Constants.flywheelRPMData[Constants.flywheelRPMData.length - 1][1]) {
            return Constants.flywheelRPMData.length - 1; // return early
        }
        for (int i = 0; i < Constants.flywheelRPMData.length; i++) {
            max = Constants.flywheelRPMData[i][1];

            if (min <= distance && distance <= max) {
                idx = i;
                break;
            } else {
                min = max;
            }
        }

        return idx;
    }

    public static double interpolateDataFromIdx(double[][] table, int idx, double input) { // input in terms of meters
        double rate;
        double x1;
        double y1;
        if (idx == 0) {
            x1 = table[0][1];
            y1 = table[0][0];
            rate = (table[1][0] - y1) / (table[1][1] - x1);
        } else if (idx == table.length - 1) {
            x1 = table[idx - 1][1];
            y1 = table[idx - 1][0];
            rate = (table[idx][0] - y1) / (table[idx][1] - x1);
        } else {
            double x2 = table[idx][1];
            double y2 = table[idx][0];
            x1 = table[idx - 1][1];
            y1 = table[idx - 1][0];
            rate = (y2 - y1) / (x2 - x1);
        }

        double output = rate * (input - x1) + y1;
        return Math.round(output*100)/100;
    }
}
