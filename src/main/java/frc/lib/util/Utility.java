// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A class to put static utility functions in. */
public class Utility {

    /**
     * Initialize value on SmartDashboard for user input, but leave old value if already present.
     *
     * @param key The SmartDashboard key to associate with the value.
     * @param defValue The default value to assign if not already on dashboard.
     *
     * @return The current value that appears on the dashboard.
     */
    public static double createSmartDashboardNumber(String key, double defValue) {
        // See if already on dashboard, and if so, fetch current value
        double value = SmartDashboard.getNumber(key, defValue);
      
        // Make sure value is on dashboard, puts back current value if already set
        // otherwise puts back default value
        SmartDashboard.putNumber(key, value);
      
        return value;
    }
}
