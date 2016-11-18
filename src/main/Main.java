package main;

import gpsevaluation.GPSDirection;
import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import sensors.OrientationChangeDetection;
import tracereplay.RealTimeTiltCalculation;
import utility.Constants;
import utility.Log;
import utility.Trace;
import utility.Trip;

public class Main {

	/**
	 * @param args
	 */	
	public static void main(String[] args) {
		//System.out.println("Hello DriveSense-Orientation!");
		
		//GPSEvaluation.start();
		
		//SlopeAwareAlignment.start();
		
		
		//GPSDirection.start();
		OrientationChangeDetection.start();
		
	}
}
