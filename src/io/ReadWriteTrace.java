package io;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import utility.Constants;
import utility.Log;
import utility.Trace;
import utility.Trip;


public class ReadWriteTrace {

	
	private static final boolean LOG = false;
	/**
	 * 
	 * @param folder
	 * @return
	 */
	public static List<Trip> loadNTrips(String folder, int n) {
		Log.log(folder);
		List<String> folders = DirectoryWalker.getFolders(folder);
		Collections.sort(folders);
		//trips used to calculate fuel consumption
		List<Trip> trips = new ArrayList<Trip>();		
		int cc = 0;
		for(String cur: folders) {
			cur = cur.concat(Constants.slash);
			cc++;
			if(cc > n)
				break;
			//load the trip
			Log.log("load trip from", cc, cur);
			Trip trip = loadTrip(cur);
			trips.add(trip);
			
		}
		return trips;
	}
	
	public static Trip loadTrip(String cur) {
		Trip trip = new Trip();
		try {
			trip.readOBDDataFromFolder(cur);
			trip.readSensorDataFromFolder(cur);
		} catch (Exception e) {
			e.printStackTrace();
		}
		String [] tokens = cur.split("/");
		trip.name_ = tokens[tokens.length - 1];
		long time = 1;
		try {
			time = Long.valueOf(tokens[tokens.length - 1]);
		} catch (Exception e) {
			time = 1;
		}
		trip.setTime(time);
		return trip;
	}
	
	/**
	 * 
	 * @param folder
	 * @return
	 */
	public static List<Trip> loadTrips(String folder) {
		List<String> folders = DirectoryWalker.getFolders(folder);
		Collections.sort(folders);
		//trips used to calculate fuel consumption
		List<Trip> trips = new ArrayList<Trip>();		
		for(String cur: folders) {
			cur = cur.concat(Constants.slash);
			//load the trip
			if(LOG) {
				Log.log("load trip from", cur);
			}
			Trip trip = loadTrip(cur);
			trips.add(trip);
		}
		return trips;
	}
	
	/*given a file path, read the traces and return*/
	static public List<Trace> readFile(String filePath, int dim) throws Exception
	{
		List<Trace> data = new ArrayList<Trace>();
		FileReader fr = null;
		BufferedReader br = null;
		try {
			fr = new FileReader(filePath);
			br = new BufferedReader(fr); 
			String line; 
			while((line = br.readLine()) != null) { 
				Trace trace = new Trace(dim);
				trace.getTrace(line);
				data.add(trace);
			} 
			fr.close();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			throw e;
		} finally {
			if (br != null) {
				try {
					br.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		return data;  
	}
	
	
	/*write traces into a file*/
	public static void writeFile(List<Trace> traces, String filePath) {
		BufferedWriter bw = null;
		try {
			bw = new BufferedWriter(new FileWriter(filePath));
			for (Trace tr: traces) {
				String line = tr.toString() + "\n";
				bw.write(line);	
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} finally {
			if (bw != null) {
				try {
					bw.flush();
					bw.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		
	}
	
	

}

