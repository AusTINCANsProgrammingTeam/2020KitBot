package frc.robot;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner; 
import java.util.logging.*;

public class Path{
    private ArrayList<Double> m_rightString = new ArrayList<Double>();
    private ArrayList<Double> m_leftString = new ArrayList<Double>();
    private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
    private String pathName;
    public Path(String PathName){
        pathName = PathName;
    }    

    public ArrayList<Double> returnLeftList() throws IOException{
            File file = new File("/home/lvuser/deploy/output/" + pathName + ".left.pf1.csv");
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

    public ArrayList<Double> returnRightList() throws IOException{
        File file = new File("/home/lvuser/deploy/output/" + pathName + ".right.pf1.csv");
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