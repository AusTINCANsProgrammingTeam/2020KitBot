package frc.robot;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner; 
import java.util.logging.*;

import edu.wpi.first.wpilibj.Filesystem;

public class Path{
    private ArrayList<String> m_rightString = new ArrayList<String>();
    private ArrayList<String> m_leftString = new ArrayList<String>();
    private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
    private String pathName;
    public Path(String PathName){
        pathName = PathName;
    }    

    public ArrayList returnLeftList() throws IOException{
            File file = new File(pathName + "left.pf1.csv");
            Scanner sc = new Scanner(file);
            String[] testArray;
        
        sc.next();
        while(sc.hasNext()){
            testArray = sc.next().split(",");
            m_leftString.add(testArray[4]);
        }
        
        return m_leftString;
    }

    public ArrayList returnRightList() throws IOException{
        File file = new File(pathName + "right.pf1.csv");
        Scanner sc = new Scanner(file);
        String[] testArray;
        sc.next();
        while(sc.hasNext()){
            testArray = sc.next().split(",");
            m_rightString.add(testArray[4]);
        }
        return m_rightString;
    }
}