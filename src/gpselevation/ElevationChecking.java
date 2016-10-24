package gpselevation;

import gpsevaluation.GPSAbstraction;
import io.DirectoryWalker;
import io.ReadWriteTrace;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Hashtable;
import java.util.List;

import utility.Trace;
import utility.Trip;

public class ElevationChecking {

	private static String fpath = "/home/bozhao/Dropbox/OBD/data/rawdat/broken/urban/";
	
	private static String databasePath = "/home/bozhao/Dropbox/OBD/data/roadsegments/elevation/broken/urban/";
//	private static String databasePath = "C:/Users/Bozhao Qi/Dropbox/OBD/data/roadsegments/elevation/bozhao/urban/";

	public static void start() {
		List<String> folders = DirectoryWalker.getFolders(fpath);
		Collections.sort(folders);
		
		for (String folder : folders) {	
		folder = folder.concat("/");
		List<Trace> gps = null, speed = null, gps_db = null;
		try{
			gps = ReadWriteTrace.readFile(folder.concat("gps.dat"), 3);
			speed = ReadWriteTrace.readFile(folder.concat("speed.dat"), 1);
			gps_db = ReadWriteTrace.readFile(databasePath.concat("gps_elevationDB.dat"), 3);
		} catch (Exception e) {
			
		}
		Hashtable<String, Double> gps_elevDB = new Hashtable<String, Double>();

		for(Trace gps_elev: gps_db){
			String lat = Double.toString(gps_elev.values[0]);
			String lon = Double.toString(gps_elev.values[1]);
			String tmp = lat.concat(lon);		
			gps_elevDB.put(tmp, gps_elev.values[2]);
		}
		List<Trace> gps_check = new ArrayList<Trace>();

		if(!gps.isEmpty()){
			Trip tmp = new Trip();
			long time = GPSAbstraction.sychronization(speed, gps);
//			long time = (long)500;
//			gps_check.addAll(elevationCheck(gps, gps_db, gps_elevDB, Constants.kTimeShiftLei));
			gps_check.addAll(elevationCheck(gps, gps_db, gps_elevDB, time));

		}
		ReadWriteTrace.writeFile(gps_check, folder.concat("gps_sychronized.dat"));

		}

	}
	
	
	/**
	 * check elevation for each gps point. Using hashtable store gps elevation database.
	 * @param gps
	 * @param gps_allDB, gps elevation data saved in List<Trace>
	 * @param Hashtable<String, Double> gps_db, gps elevation data saved in hashtable
	 * @param time_diff
	 * @return
	 */
	public static List<Trace> elevationCheck(List<Trace> gps, List<Trace> gps_allDB, Hashtable<String, Double> gps_db, Long time_shift){
		List<Trace> gps_check = new ArrayList<Trace>();
		for(int i=0; i< gps.size(); i++){
			Trace tmp = new Trace(2);
			BigDecimal lat = new BigDecimal(gps.get(i).values[0]);
	        double lat_round = lat.setScale(9, BigDecimal.ROUND_HALF_UP).doubleValue();
			tmp.values[0] = lat_round;
			BigDecimal lon = new BigDecimal(gps.get(i).values[1]);
	        double lon_round = lon.setScale(9, BigDecimal.ROUND_HALF_UP).doubleValue();
			tmp.values[1] = lon_round;
			
			String lat1 = Double.toString(tmp.values[0]);
			String lon1 = Double.toString(tmp.values[1]);
			String tmp1 = lat1.concat(lon1);
			
			Trace trace = new Trace(4);
			trace.time = gps.get(i).time;
			trace.values[0] = gps.get(i).values[0];
			trace.values[1] = gps.get(i).values[1];
			trace.values[3] = gps.get(i).values[2];
//			trace.copyTrace(gps.get(i));
			trace.time = trace.time + time_shift;
			if(gps_db.get(tmp1)!=null){
			trace.values[2] = gps_db.get(tmp1);
			gps_check.add(trace);
			}else{
				trace.values[2] = ElevationRecording.getElevation(gps_allDB, gps.get(i));
				gps_check.add(trace);
				//Log.log(gps.get(i));
			}
		}
		return gps_check;
	}
	
	/**
	 * check elevation for each gps point
	 * @param gps
	 * @param gps_db
	 * @param time_diff
	 * @return
	 */
	public static List<Trace> elevationCheck(List<Trace> gps, List<Trace> gps_db, Long time_shift){
		List<Trace> gps_check = new ArrayList<Trace>();
		for(int i=0; i< gps.size(); i++){
			Trace trace = new Trace(3);
			trace.copyTrace(gps.get(i));
			trace.time = trace.time - time_shift;
			trace.values[2] = ElevationRecording.getElevation(gps_db, gps.get(i));
			gps_check.add(trace);
		}
		return gps_check;
	}
	
	/**
	 * check elevation for each gps point. find nearest two points, interpolation
	 * @param gps
	 * @param gps_db
	 * @param time_diff
	 * @return
	 */
	public static List<Trace> elevationCheck_inter(List<Trace> gps, List<Trace> gps_db, Long time_shift){
		List<Trace> gps_check = new ArrayList<Trace>();
		for(int i=0; i< gps.size(); i++){
			Trace trace = new Trace(3);
			trace.copyTrace(gps.get(i));
			trace.time = trace.time - time_shift;
			trace.values[2] = ElevationRecording.getElevation_interpolate(gps_db, gps.get(i));
			gps_check.add(trace);
		}
		return gps_check;
	}
}
