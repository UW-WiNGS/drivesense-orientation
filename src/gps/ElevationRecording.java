package gps;

import java.util.List;

import tracereplay.DirectoryWalker;
import tracereplay.ReadWriteTrace;
import utility.Trace;

public class ElevationRecording {
	
	private static String datPath = "/home/lkang/Dropbox/projects/obd/data/rawdat/";
	private static String elevationPath = "/home/lkang/Dropbox/projects/obd/data/roadsegments/elevation/";
	private static String name = "lei";
	
	public static void start() {
		// TODO Auto-generated method stub
		List<Trace> elevationDB = ReadWriteTrace.readFile(elevationPath.concat(name + "/gps_elevation.dat"), 3);

		List<String> folders = DirectoryWalker.getFolders(datPath.concat(name) + "/urban/");
		for (String folder : folders) {
			folder = folder.concat("/");
			List<Trace> gps = ReadWriteTrace.readFile(folder.concat("gps.dat"), 3);
			for(int i = 0; i < gps.size(); ++i) {
				Trace trace = new Trace(3);
				trace.copyTrace(gps.get(i));
				trace.values[2] = getElevation(elevationDB, gps.get(i));
				gps.set(i, trace);
			}
			//ReadWriteTrace.writeFile(gps, folder.concat("gps_elevation.dat"));
		}
	}
	
	public static double getElevation(List<Trace> elevationDB, Trace point) {
		double dist = Double.MAX_VALUE;
		double elevation = 0.0;
		for(int i = 0; i < elevationDB.size(); ++i) {
			double dis = GPSAbstraction.distance(elevationDB.get(i), point);
			if(dis < dist) {
				dist = dis;
				elevation = elevationDB.get(i).values[2];
			}
		}
		return elevation;
	}
	
	public static double getElevation_interpolate(List<Trace> elevationDB, Trace point) {
		double min_1 = Double.MAX_VALUE;
		double min_2 = Double.MAX_VALUE;
		int index_1 = 0;
		int index_2 = 0;
		double elevation = 0.0;
		Trace gps_min1 = elevationDB.get(0);
		Trace gps_min2 = elevationDB.get(0);
		for(int i = 0; i < elevationDB.size(); ++i) {
			double dis = GPSAbstraction.distance(elevationDB.get(i), point);
			if(dis < min_1) {
				min_2 = min_1;
				min_1 = dis;
				index_2 = index_1;
				index_1 = i;
/*				gps_min2.copyTrace(gps_min1);
				gps_min1 = elevationDB.get(i);
*/			}else if(dis < min_2)
			{
				min_2 = dis;
				index_2 = i;
//				gps_min2 = elevationDB.get(i);
			}
		}
		gps_min1 = elevationDB.get(index_1);
		gps_min2 = elevationDB.get(index_2);
		if(gps_min2.values[2]-gps_min1.values[2] == 0){
			elevation = gps_min2.values[2];
		}
		else{
			double angle_AB = GPSAbstraction.direction(gps_min1, gps_min2);
			double angle_AC = GPSAbstraction.direction(gps_min1, point);
			double alpha = Math.abs(GPSAbstraction.turnAngle(angle_AB, angle_AC));
			double dist_AB = GPSAbstraction.distance(gps_min1, gps_min2);
			double dist_AC = GPSAbstraction.distance(gps_min1, point);
			elevation = gps_min1.values[2] + (gps_min2.values[2]-gps_min1.values[2]) * dist_AC * Math.cos(Math.toRadians(alpha)) / dist_AB;
		}
		return elevation;
	}
	// URL Request Method
}
