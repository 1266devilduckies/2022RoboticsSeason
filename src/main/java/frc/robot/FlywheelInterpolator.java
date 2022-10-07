package frc.robot;

public class FlywheelInterpolator {
    public static int findRangeIdx(double distance) { // in terms of meters
        // if the input is past the largest
        // recording it defaults to the biggest one for interpolating
        int idx = -1;
        double min = 0;
        double max;

        if (distance < Constants.flywheelRPMData[0][1]) {
            return 0; //return early
        }
        if (distance > Constants.flywheelRPMData[Constants.flywheelRPMData.length-1][1]) {
            return Constants.flywheelRPMData.length; //return early
        }
        for (int i = 0; i < Constants.flywheelRPMData.length; i++) {
            max = Constants.flywheelRPMData[i][1];

            if (min <= distance && distance <= max) {
                idx = i;
            } else {
                min = max;
            }
        }

        return idx;
    }

    public static double interpolateDataFromIdx(double[][] table, int idx, double input) { //input in terms of meters
        double rate;
        double x = table[idx][1];
        double y = table[idx][0];
        if (idx==0) {
            rate = table[1][0] - table[0][0];
        } else if (idx==table.length) {
            rate = table[table.length-1][0] - table[table.length-2][0];
        } else {
            rate = table[idx][0] - table[idx-1][0];
        }

        return rate*(input-x)+y;
    }
}
