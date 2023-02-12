// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Utils {
    /**
     * 
     * sets all values between min and max to 0 
     * @param maxBand 
     * @param minBand the min of the deadband 
     * @param value the value to dead band 
     * @return value if not in deadband and 0 if is in deadband 
     */
    public static double DeadBand(double maxBand,double minBand,double value){
        if(value >= minBand && value <=maxBand){
            return 0;
        }
        return value;
    }

}
