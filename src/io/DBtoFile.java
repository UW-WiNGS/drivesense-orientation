package io;


import java.util.Collections;
import java.util.List;

import utility.Constants;
import utility.Log;
import utility.Trace;
import utility.Trip;


public class DBtoFile {
	
	public static int counter_ = 0;
	public static int total_ = 0;
	
	public static double nogpsdistance_ = 0.0;
	public static double distance_ = 0.0;
	
	
	/**
	 * deletion
	 */
	public static void batchDetele() {
		String dir = Constants.datPath;
		DirectoryWalker.delete(dir);
	}
	
	/**
	 * 
	 * @param dbPath
	 * @param datPath
	 */
	public static void batchDBConvert(String dbPath, String datPath, String name) {
		
		List<String> folders = DirectoryWalker.getFolders(dbPath);
		for(String dir: folders) {
			String nameFolder = dir.concat(Constants.slash);			
			Log.log(nameFolder);
			if(!nameFolder.contains(name))
				continue;
			List<String> subFolders = DirectoryWalker.getFolders(nameFolder);		
			for(String cur: subFolders) {
				//each folder contains a series of db files
				cur = cur.concat(Constants.slash);
				convertDBtoDAT(nameFolder.substring(dbPath.length()), cur, datPath);
			}
		}
	}
	

	/**
	 * 
	 * @param inputpath: the outer most path to the db folders ./data/rawdb/
	 * @param folder: the folder path that contains the db files 
	 * @param outpath: the outer most output path  ./data/rawdat/
	 */

	public static void convertDBtoDAT(String name, String folder, String outpath) {		

		Log.log(name, folder, outpath);
		List<String> dbs = DirectoryWalker.getFileNames(folder);
		Collections.sort(dbs);
		for(String db: dbs) {
			Log.log("converting db: ", db);
			writeDBtoFile(folder, db, outpath + name);
		}
	}
	
	/**
	 * 
	 * @param infolder: input folder
	 * @param db: database file name usually <time>.db
	 * @param outfolder: output folder, usually name after the database file name
	 */
	private static void writeDBtoFile (String infolder, String db, String outfolder) {
		String dbfile = infolder.concat(db);
		if(!db.endsWith(".db")) return;
		
		long start = Long.parseLong(db.substring(0, db.length() - 3));
		Trip trip = new Trip();
		trip.time_ = start;
		
		String flushFolder = "";
		trip.tripStatistics();
		if(trip.duration() < 10000)
			return;
		if(trip.isHighwayTrip()) {
			flushFolder = outfolder + "highway/" ;
			//return;
		} else {
			flushFolder = outfolder + "urban/";		
		}
		Log.log(flushFolder);
		DirectoryWalker.createFolder(flushFolder + String.valueOf(start));			
			
		trip.flushOBDDataToFolder(flushFolder + String.valueOf(start) + "/");
		
		trip.gps_ = SqliteAccess.loadSensorData(dbfile, start, Trace.GPS);
		/*
		distance_+= trip.distance_;
		total_++;
			if(trip.gps_.size() ==0) {
				counter_++;
			nogpsdistance_ += trip.distance_;
			}

			Log.log(nogpsdistance_, distance_);
		*/
		
		if(false == trip.nosensor_) {
			trip.accelerometer_ = SqliteAccess.loadSensorData(dbfile, start, Trace.ACCELEROMETER);
			trip.gyroscope_ = SqliteAccess.loadSensorData(dbfile, start, Trace.GYROSCOPE);
			trip.rotation_matrix_ = SqliteAccess.loadSensorData(dbfile, start, Trace.ROTATION_MATRIX);
			
		}
		trip.flushSensorDataToFolder(flushFolder + String.valueOf(start) + "/");
		
	}

}
