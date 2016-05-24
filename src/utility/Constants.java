package utility;

public class Constants {
	
	public static final double kEarthGravity = 9.80665; /*m^2/s*/

	/*for GPS*/
	public static final double kSmallEPSILON = 1e-8;
	public static final double kEarthRadius = 6371 * 1000; /*m*/
	
	public static final double kMeterToMile = 0.000621371;
	public static final double kMeterPSToMilePH = 2.23694;
	public static final double kKmPHToMPH = 0.621371;
	public static final double kKmPHToMeterPS = 0.277778;
	
	public static final double kGPSMinimumDistance = 3; /*m*/

	public static final double kGallonToLiter = 3.78541;
	public static final double kMPGToKPL = 0.425143707;
	/**/
    public static final String kInputSeperator = "\t";
	public static final String kOutputSeperator = "\t";
    public static final String slash = "/";
    
    public static final double kAirFuelRatio = 14.67;
    public static final double kMAFtoAFR = 3600.0/(kAirFuelRatio * 6.17 * 454);
    
    public static final double PERCENT_ = 0.7;
    
    //refers one sample per second
    public static final double kSampleRate = 1.0;
    
	public static final double kExponentialMovingAverageAlpha = 0.1;
    /**
     * works for Lei only
     */
    public static final String kHome = System.getProperty("user.home");
	public static final String dbPath = Constants.kHome + "/Dropbox/projects/obd/data/rawdb/";
	public static final String datPath = Constants.kHome + "/Dropbox/projects/obd/data/rawdat/";
	public static final String outputPath = Constants.kHome + "/Dropbox/projects/obd/data/workingset/";
	
    
    /*====================database related======================================================*/
    

    /*
    public static final String kDatabaseName = "drivesense";
    public static final String kDatabaseURL = "jdbc:mysql://144.92.202.85:3306/"+kDatabaseName+"?useUnicode=true&characterEncoding=GBK";
    public static final String kDatabaseUserName = "root";
    public static final String kDatabasePassword = "soekris;";
    */
    
    public static final String kDatabaseName = "drivesense";
    public static final String kServerIP = "71.87.61.229";
    public static final String kDatabaseURL = "jdbc:mysql://" + kServerIP + ":3306/"+kDatabaseName+"?useUnicode=true&characterEncoding=GBK";
    public static final String kDatabaseUserName = "root";
    public static final String kDatabasePassword = "kanglei";
    
    
    /**/
    public static final String kMagnetic = "MAGNETIC_FIELD";
    public static final String kGyroscope = "GYROSCOPE";
    public static final String kAccelerometer = "ACCELEROMETER";
    public static final String kGPS = "GPS";
    

    /*====================database related======================================================*/
    
}
