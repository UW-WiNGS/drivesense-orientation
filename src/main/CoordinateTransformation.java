package main;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.regex.Pattern;

import tracereplay.PreProcess;
import utility.Constants;
import utility.Formulas;
import utility.PairDouble;
import utility.Trace;

public class CoordinateTransformation {
	

	

	
	
	/**
	 * 
	 * @param raw_xys 
	 * @param slope
	 * @return
	 */

	public static PairDouble[] AxesUnitVector(ArrayList<PairDouble> raw_xys, double slope) {
		PairDouble unit_x = new PairDouble();
		PairDouble unit_y = new PairDouble();

		int rightnum = 0;
		double perpendicular_slope = -1/slope;

		for (int i = 0; i < raw_xys.size(); i++) {
			PairDouble xy = raw_xys.get(i);
			if (slope > 0 ^ xy.x * perpendicular_slope > xy.y)
				rightnum ++;
		}
		

		int y_indicator = (rightnum/raw_xys.size() > Constants.PERCENT_)? 1:-1;
		int x_indicator = ((y_indicator > 0) ^ (slope > 0))? -1:1;

		unit_x = Formulas.UnitVector( new PairDouble(x_indicator, x_indicator*perpendicular_slope) );
		unit_y = Formulas.UnitVector( new PairDouble(y_indicator, y_indicator*slope) );

		PairDouble[] res = {unit_x, unit_y};

		return res;
	}

	/**
	 * Get the best fit line through origin.
	 * @param points
	 * @return: the slope of the line, (from 0--45--90, 0--1--infinite)
	 */
	public static double BestFitOrigin(ArrayList<PairDouble> points) {
		int n = points.size();
		double sum_xy, sum_x2, sum_y2;
		sum_xy = sum_x2 = sum_y2 = 0;
		for (int i = 0; i < n; i++) {
			double x = points.get(i).x;
			double y = points.get(i).y;
			sum_xy += x*y;
			sum_x2 += Math.pow(x, 2);
			sum_y2 += Math.pow(y, 2);
		}

		double temp = sum_y2 - sum_x2;

		// get both + and - slopes.
		double slope1 = (temp + Math.sqrt(Math.pow(temp, 2) + 4 * Math.pow(sum_xy, 2) )) / (2 * sum_xy);
		double slope2 = (temp - Math.sqrt(Math.pow(temp, 2) + 4 * Math.pow(sum_xy, 2) )) / (2 * sum_xy);
		// plug into the distance calculation equation to compare which one is smaller.
		double ds1 = Formulas.DistanceSquare(slope1, sum_x2, sum_y2, sum_xy);
		double ds2 = Formulas.DistanceSquare(slope2, sum_x2, sum_y2, sum_xy);
		return (ds1 < ds2)? slope1 : slope2;
	}

	/**
	 * 
	 * @param raw_accels: the raw accelerations
	 * @param parameters: the parameter, could be orientaion or rotation matrix.
	 * @param method: orientation or rotation matrix.
	 * @return
	 */
	public static ArrayList<Trace> Calculate(List<Trace> sampled, Trace parameters, String method) {
		ArrayList<Trace> true_accels = new ArrayList<Trace>();

		double[] arg = parameters.values;

		for (Trace tr : sampled) {
			Trace true_tr = RotationMethod(tr, arg);
			true_accels.add(true_tr);
		}

		return true_accels;
	}

	/**
	 * Using 3D angular mathematical mapping.
	 * @param raw_tr
	 * @param orientation
	 * @return
	 */
	public static Trace OrientationMethod(Trace raw_tr, double[] orientation) {
		Trace calculated_tr = new Trace();
		double x, y, z;
		x = raw_tr.values[0];
		y = raw_tr.values[1];
		z = raw_tr.values[2];
		double azimuth, pitch, yaw;
		azimuth = orientation[0];
		pitch = orientation[1];
		yaw = orientation[2];

		calculated_tr.time = raw_tr.time;
		calculated_tr.values[0] = (double) (x
				* (Math.cos(yaw) * Math.cos(azimuth) + Math.sin(yaw)
						* Math.sin(pitch) * Math.sin(azimuth)) + y
						* (Math.cos(pitch) * Math.sin(azimuth)) + z
						* (-Math.sin(yaw) * Math.cos(azimuth) + Math.cos(yaw)
								* Math.sin(pitch) * Math.sin(azimuth)));
		calculated_tr.values[1] = (double) (x
				* (-Math.cos(yaw) * Math.sin(azimuth) + Math.sin(yaw)
						* Math.sin(pitch) * Math.cos(azimuth)) + y
						* (Math.cos(pitch) * Math.cos(azimuth)) + z
						* (Math.sin(yaw) * Math.sin(azimuth) + Math.cos(yaw)
								* Math.sin(pitch) * Math.cos(azimuth)));
		calculated_tr.values[2] = (double) (x * (Math.sin(yaw) * Math.cos(pitch)) + y
				* (-Math.sin(pitch)) + z * (Math.cos(yaw) * Math.cos(pitch)));
		return calculated_tr;
	}

	/**
	 * Using rotational matrix.
	 * @param raw_tr
	 * @param rM
	 * @return
	 */
	public static Trace RotationMethod(Trace raw_tr, double[] rM) {
		Trace calculated_tr = new Trace();
		calculated_tr.time = raw_tr.time;
		double x, y, z;
		x = raw_tr.values[0];
		y = raw_tr.values[1];
		z = raw_tr.values[2];

		calculated_tr.values[0] = x * rM[0] + y * rM[1] + z * rM[2];
		calculated_tr.values[1] = x * rM[3] + y * rM[4] + z * rM[5];
		calculated_tr.values[2] = x * rM[6] + y * rM[7] + z * rM[8];

		return calculated_tr;
	}

}
