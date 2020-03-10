/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;
import java.util.logging.Logger;

public class Path{
    private ArrayList<Double> m_rightString = new ArrayList<Double>();
    private ArrayList<Double> m_leftString = new ArrayList<Double>();
    private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
    private static final String rootDir = "C://Users//Public//Documents//deploy//output//";
    private String pathName;
    public Path(String PathName){
        pathName = PathName;
    }    

    /** 
     * @return ArrayList<Double>
     * @throws IOException
     */
    public ArrayList<Double> returnLeftList() throws IOException{
            File file = new File(rootDir + pathName + ".left.pf1.csv");
            Scanner sc = new Scanner(file);
            String[] testArray;
        
        sc.next();
        while(sc.hasNext()){
            testArray = sc.next().split(",");
            m_leftString.add(Double.valueOf(testArray[4]));
        }
        sc.close();
        return m_leftString;
    }

    
    /** 
     * @return ArrayList<Double>
     * @throws IOException
     */
    public ArrayList<Double> returnRightList() throws IOException{
        File file = new File(rootDir + pathName + ".right.pf1.csv");
        Scanner sc = new Scanner(file);
        String[] testArray;
        sc.next();
        while(sc.hasNext()){
            testArray = sc.next().split(",");
            m_rightString.add(Double.valueOf(testArray[4]));
        }
        sc.close();
        return m_rightString;
    }
}