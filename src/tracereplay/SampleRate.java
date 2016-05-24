package tracereplay;

import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;

import utility.Constants;
import utility.Log;
import utility.Trace;

public class SampleRate {
	
	private static final String root = Constants.kHome + "/backups/projects/DriveSense/data/android/";
	
	private static final String rotated = root + "/data/rotated/";
	private static final String output = root + "/data/rotated/";
	
	
	public void start() {
		
		List<Trace> accelerometer = ReadWriteTrace.readFile(rotated.concat("accelerometer.dat"));
		List<Trace> sampled_accelerometer = interpolate(accelerometer, 2.0);
		
		
		
		List<Trace> gyroscope = new ArrayList<Trace>();
		gyroscope = ReadWriteTrace.readFile(rotated.concat("gyroscope.dat"));
		List<Trace> sampled_gyroscope = new ArrayList<Trace>();
		sampled_gyroscope = interpolate(gyroscope, 2.0);
		

		List<Trace> rotation_matrix = ReadWriteTrace.readFile(rotated.concat("rotation_matrix.dat"), 9);
		List<Trace> gps = ReadWriteTrace.readFile(rotated.concat("gps.dat"), 4);
		
		List<Trace> calculated_accelerometer = new ArrayList<Trace>();
		List<Trace> calculated_gyroscope = new ArrayList<Trace>();
		
		List<Trace> window = new LinkedList<Trace>();	
		List<Trace> fittingpoints = new ArrayList<Trace>();
		
		List<Trace> projected_accelerometer = new ArrayList<Trace>();
		List<Trace> smoothed_accelerometer = new ArrayList<Trace>();


		long lastTime = 0;
		double lastSpeed = -1.0;
		
		int i = 0, j = 0, k = 0;
		RotationMatrix rm = new RotationMatrix();
		
		boolean rotating = false;
		
		boolean accelerating = false;
		boolean braking = false;
		
		Trace curProjectedAccelerometer = new Trace(3);
		
		Trace lastSmoothedAccelerometer = null;
		Trace lastSmoothedGyroscope = null;
		

		
		final double alpha = 0.7;
		for(; i < sampled_accelerometer.size() && j < sampled_gyroscope.size() && k < gps.size(); ) {
			//Log.log(i, j, k);
			Trace curacce = sampled_accelerometer.get(i);
			Trace curgyro = sampled_gyroscope.get(j);
			Trace curgps = gps.get(k);
			//Log.log(curacce.time, curgyro.time);
			if(curacce.time <= curgps.time /*&& curacce.time <= curgyro.time*/) {
				//process accelerometer
				++i;
				//Log.log(curacce);
				window.add(curacce);
				if(window.size() > 8) {
					window.remove(0);
					boolean steady = stopped(window);
					if(steady && rm.rm_set == false) {
						Trace tmprm = PreProcess.getTraceAt(rotation_matrix, curacce.time);
						if(tmprm !=null) {
							rm.rotation_matrix = tmprm.values;
							rm.setRotationMatrix(tmprm);
							rm.rm_set = true;
							Log.log("rotation matrix is set");
						}
					}
				}
				
				if(lastSmoothedAccelerometer == null) {
					if(rm.rm_set==true) {
						lastSmoothedAccelerometer = new Trace(3);
						lastSmoothedAccelerometer.copyTrace(curacce);
					}
				} else {
					for(int m = 0; m < 3; ++m) {
						curacce.values[m] = alpha * curacce.values[m] + (1.0 - alpha) * lastSmoothedAccelerometer.values[m];
						lastSmoothedAccelerometer.values[m] = curacce.values[m];
					}
				}
				
				//Log.log(smoothed_accelerometer.size());
				//smoothed_accelerometer.add(curacce);
				
				
		
				//when rotation matrix is set
				if(rm.rm_set == true) {
					//Log.log("rmset is true");
					Trace ntr = rm.Rotate(curacce);
					//Log.log(ntr);
					//calculated_accelerometer.add(ntr);
										
					if(Math.sqrt(Math.pow(ntr.values[0], 2.0) + Math.pow(ntr.values[1], 2.0)) > 0.05 && rotating == false && rm.aligned == false) {
						if(fittingpoints.size() < 200) {
						   fittingpoints.add(ntr);
						} else {
							double slope = rm.curveFit(fittingpoints);
							rm.setUnitVector(fittingpoints, slope);
							
							rm.aligned = true;
							Log.log("rotation matrix is aligned");
						}
					}
					if(rm.all_set == true) {
						curProjectedAccelerometer = rm.Alignment(ntr);
						processProjectedAccelerometer(curProjectedAccelerometer);
						
						Trace otr = new Trace(3);
						otr.copyTrace(curProjectedAccelerometer);
						projected_accelerometer.add(otr);						
					} else if(rm.aligned == true) {
						curProjectedAccelerometer = rm.Alignment(ntr);						
					} else {
						curProjectedAccelerometer = ntr;
					}
				}
			} 
			
			else if (curgyro.time <= curacce.time && curgyro.time <= curgps.time) {
				//process gps
				
                if(lastSmoothedGyroscope == null) {
                    if(rm.rm_set == true) {
                        lastSmoothedGyroscope = new Trace(3);
                        lastSmoothedGyroscope.copyTrace(curgyro);
                    }
                } else {
                    for(int m = 0; m < 3; ++m) {
                    	curgyro.values[m] = alpha * curgyro.values[m] + (1.0 - alpha) * lastSmoothedGyroscope.values[m];
                        lastSmoothedGyroscope.values[m] = curgyro.values[m];
                    }
                }

				
				++j;
				if(rm.rm_set==true) {
					Trace calgyro = rm.Rotate(curgyro);
					
					calculated_gyroscope.add(calgyro);
					processProjectedGyroscope(calgyro, curProjectedAccelerometer, rm);
					
					if(Math.abs(calgyro.values[2]) > 0.03) {
						rotating = true;
					} else {
						rotating = false;
					}
					
					//deal with turns and lane changes
				}
			} 
			else {
				++k;
				long time = curgps.time/1000;
				time -= 1381611476;
				double speed = curgps.values[3];
				Date date = new Date(time);

				//Log.log(speed);
				if(lastSpeed < 0.0) {
					lastSpeed = speed;
					lastTime = curgps.time;
				} else {
					double a = (speed - lastSpeed)/((time - lastTime)/1000.0);
					//Log.log(curgps, a);

					//Log.log(speed, a);
					lastSpeed = speed;
					lastTime = time;
					if(accelerating) {
						//check if the project is reversed
						if(rm.all_set == false && rm.aligned == true) {
							if(curProjectedAccelerometer.values[1] < 0.0) {
								rm.reversed = true;
							} else {
								rm.reversed = false;
							}
							rm.all_set = true;
							Log.log(time, "rotation matrix is all set");
						}
						if(a < 0.5) {
							accelerating = false;							
							//Log.log(time, "end accelerating by GPS");
						}
					} else if(braking) {
						if(a > -1.0) {
							braking = false;
							//Log.log(time, "end braking by GPS");
						}
					} else {
						if(a > 1.5) {
							accelerating = true;
							//Log.log(time, "starting accelerating by GPS");
						} else if(a < -1.5) {
							braking = true;
							//Log.log(time, "start braking by GPS");
						} else {
							//accelerating = false;
							//braking = false;
						}
					}
					
				}
			}
		}
		
		
		ReadWriteTrace.writeFile(sampled_accelerometer, output.concat("sampled_accelerometer.dat"));
		ReadWriteTrace.writeFile(smoothed_accelerometer, output.concat("smoothed_accelerometer.dat"));
		ReadWriteTrace.writeFile(projected_accelerometer, output.concat("projected_accelerometer.dat"));
		
		ReadWriteTrace.writeFile(calculated_gyroscope, output.concat("calculated_gyroscope.dat"));
		
		
	}
	
	private Trace maxAccelerometer = new Trace(3);
	
	boolean acceleratingByAccelerometer = false;
	boolean brakingByAccelerometer = false;	
	
	final boolean logAccelerometer = false;
	private void processProjectedAccelerometer(Trace acce) {
		long time = acce.time/1000;
		time -= 1384304896;
		Date date = new Date(time);
		double a = acce.values[1];
		if(acceleratingByAccelerometer) {
			if(a < 0.5) {
				acceleratingByAccelerometer = false;	
				if(logAccelerometer)
					Log.log(time, "end accelerating by Accelerometer", maxAccelerometer.values[1]);
			} else {
				maxAccelerometer.values[1] = Math.max(maxAccelerometer.values[1], Math.abs(acce.values[1]));							
			}
		} else if(brakingByAccelerometer) {
			if(a > -1.0) {
				brakingByAccelerometer = false;
				if(logAccelerometer)
					Log.log(time, "end braking by Accelerometer", maxAccelerometer.values[1]);
			} else {
				maxAccelerometer.values[1] = Math.max(maxAccelerometer.values[1], Math.abs(acce.values[1]));	
			}
		} else {
			if(a > 1.3) {
				acceleratingByAccelerometer = true;
				maxAccelerometer.values[1] = acce.values[1];
				if(logAccelerometer)
					Log.log(time, "starting accelerating by Accelerometer");
			} else if(a < -2.0) {
				brakingByAccelerometer = true;
				maxAccelerometer.values[1] = Math.abs(acce.values[1]);
				if(logAccelerometer)
					Log.log(time, "start braking by Accelerometer");
			} else {
				//accelerating = false;
				//braking = false;
			}
		}
	}

	private List<Trace> windowGyro = new LinkedList<Trace>();
	private boolean turningByGyroscope = false;
	private double accumZ = 0.0;
	private double accumZabs = 0.0;
	
	  private void processProjectedGyroscope(Trace gyro, Trace acce, RotationMatrix rm) {
	        windowGyro.add(gyro);
	        long time = gyro.time/1000 - 1384304896;
	        if(windowGyro.size() > 6) {
	            windowGyro.remove(0);
	            boolean isTurning = isTurning(windowGyro);
	            //Log.log(isTurning, turning);
	            if(turningByGyroscope == true) {
	                if(isTurning == false) {
	                    turningByGyroscope = false;
	                    Log.log("end turnning:");

	                    if(Math.abs(Math.toDegrees(accumZ)) < 10 && Math.toDegrees(accumZabs) > 30) {
	                        Log.log("this is a lane change", time);
	                        
	                    } else if(Math.abs(Math.toDegrees(accumZ)) > 60 && Math.toDegrees(accumZabs) > 60) {
	                        Log.log("this is a turn", time);
	                    } else {

	                    }
	                    accumZ = 0.0;
	                    accumZabs = 0.0;
	                } else {
	                    double sumAcce = Math.sqrt(Math.pow(acce.values[0], 2.0) + Math.pow(acce.values[1], 2.0));

	                    double change = windowGyro.get(0).values[2] * ((windowGyro.get(1).time - windowGyro.get(0).time)/1000.0);
	                    accumZ += change;
	                    accumZabs += Math.abs(change);
	                }
	            } else {
	                if(isTurning == true) {
	                    Log.log("start turning", time);
	                    turningByGyroscope = true;
	                    double sumAcce = Math.sqrt(Math.pow(acce.values[0], 2.0) + Math.pow(acce.values[1], 2.0));
	                    
	                }
	            }
	        }
	    }
	  
	/*
	private void processProjectedGyroscope(Trace gyro, Trace acce, RotationMatrix rm) {
		long time = gyro.time/1000;
		time -= 1384304896;
		windowGyro.add(gyro);
		int sz = windowGyro.size();
		if(sz > 6) {
			windowGyro.remove(0);
			boolean isTurning = isTurning(windowGyro);
			//Log.log(isTurning, turning);
			if(isTurning == true) {
				if(turning == false) {
					Log.log(time, "start turning:");
					turning = true;
					double sumAcce = Math.sqrt(Math.pow(acce.values[0], 2.0) + Math.pow(acce.values[1], 2.0));
					maxAccelerometer.values[0] = sumAcce;
					//maxAccelerometer.values[0] = Math.abs(acce.values[0]);
				} else {					
					//maxAccelerometer.values[0] = Math.max(maxAccelerometer.values[0], Math.abs(acce.values[0]));
					
					double sumAcce = Math.sqrt(Math.pow(acce.values[0], 2.0) + Math.pow(acce.values[1], 2.0));
					maxAccelerometer.values[0] = Math.max(sumAcce, maxAccelerometer.values[0]);

					double change = windowGyro.get(0).values[2] * ((windowGyro.get(1).time - windowGyro.get(0).time)/1000.0);
					accumZ += change;
					accumZabs += Math.abs(change);
				}
			} else {
				if(turning == true) {
					Log.log("end turnning:", Math.toDegrees(accumZ), Math.toDegrees(accumZabs));
					if(Math.abs(Math.toDegrees(accumZ)) < 10 && Math.toDegrees(accumZabs) > 30) {
						Log.log("this is a lane change", maxAccelerometer.values[0]);
					} else if(Math.abs(Math.toDegrees(accumZ)) > 60 && Math.toDegrees(accumZabs) > 60) {
						Log.log("this is a turn", maxAccelerometer.values[0]);
					} else {
						
					}
					turning = false;
					accumZ = 0.0;
					accumZabs = 0.0;
				}
			}
		}
	}
	*/
	
	private boolean isTurning(List<Trace> window) {
		int sz = window.size();
		double rads = 0.0;
		for(int i = 0; i < sz - 1; ++i) {
			Trace cur = window.get(i);
			long time_diff = window.get(i + 1).time - cur.time;			
			double z = cur.values[2];
			rads += Math.abs(z * (time_diff/1000.0));
		}
		if(Math.abs(Math.toDegrees(rads)) > 8.0)
			return true;
		return false;
	}
	
	private static List<Trace> notMoving(List<Trace> accelerometer) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> window = new LinkedList<Trace>();
		for(int i = 0; i < accelerometer.size(); ++i) {
			Trace cur = accelerometer.get(i);
			window.add(cur);
			if(window.size() > 6) {
				window.remove(0);
				boolean test = stopped(window);
				Trace ntrace = new Trace(1);
				ntrace.time = cur.time;
				ntrace.values[0] = true == test? 1.0 : 0.0;
				res.add(ntrace);
			}
		}
		return res;
	}

	/**
	 * check if the phone is static based on accelerometer
	 * @param window
	 * @return
	 */
	private static boolean stopped(List<Trace> window) {
		double [] maxs = {-100.0, -100.0, -100.0};
		double [] mins = {100.0, 100.0, 100.0};		
		final int dim = 3;
		final double threshold = 0.15;
		for(int i = 0; i < window.size(); ++i) {
			Trace cur = window.get(i);
			for(int j = 0; j < dim; ++j) {
				if(cur.values[j] > maxs[j]) {
					maxs[j] = cur.values[j];
				}
				if(cur.values[j] < mins[j]) {
					mins[j] = cur.values[j];
				}
			}
		}
		for(int i = 0; i < dim; ++i) {
			if(Math.abs(maxs[i] - mins[i]) > threshold) {
				return false;
			}
		}
		return true;
	}
	
	
	/**
	 * 
	 * @param raw the input data to be interpolated 
	 * @param rate samples per second
	 * @return
	 */
	public static List<Trace> interpolate(List<Trace> raw, double rate) {
		List<Trace> res = new ArrayList<Trace>();
		
		int sz = raw.size();
		if(0==sz) return res;
		assert sz > 0 && rate>=1;
		long x = raw.get(0).time/1000 * 1000 + 1000;
		for(int i = 0; i < sz - 1; ++i) {
			Trace cur = raw.get(i);
			Trace next = raw.get(i + 1);
			if(x >= cur.time && x < next.time) {
				Trace inter = new Trace();
				inter.copyTrace(cur);
				inter.time = x;
				//Log.log(x/1000 - 1379879638, cur.values[2]);
				//assert (x/1000 - 1379879638) < 190;
				for(int j = 0; j < inter.dim; ++j) {
					long x1 = cur.time, x2 = next.time;
					double y1 = cur.values[j], y2 = next.values[j];
					
					double v1 = y1 + (x - x1) * (y2 - y1) / (x2 - x1);
					//double v2 = y2 - (x2 - x) * (y2 - y1) / (x2 - x1);
					inter.values[j] = v1;
				}
				res.add(inter);
				x += (1000.0/rate);
				--i;
			}
		}
		return res;
		
	}

	
}
