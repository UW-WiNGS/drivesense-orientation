package database;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.List;

import utility.Log;
import utility.Trace;

public class SqliteAccess {
	
	private static Connection connection = null;
	private final static String TAG = "SqliteAccess";
	 
	private static String prefix = "jdbc:sqlite:"; 
	
	private static void close() {
		try {
			if(connection != null) {
				connection.close();
			}
		} catch(SQLException e) {
	        // connection close failed.
			System.err.println(e);  
		}
	}
	
	private static void connect (String dbpath) {		  
		try {
			Class.forName("org.sqlite.JDBC");
			connection = DriverManager.getConnection(dbpath);
		} catch (ClassNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	

	public static List<Trace> loadSensorData(String path, long start, String type) {
		
		connect(prefix.concat(path));
		
		List<Trace> res = new ArrayList<Trace>();
		
		Statement statement = null;
		ResultSet rs = null;
		//long start = Long.MAX_VALUE;
		String table = null;
		int dim = 3;
		if(type.equals(Trace.ACCELEROMETER)) {
			table = "accelerometer";
		} else if (type.equals(Trace.GYROSCOPE)) {
			table = "gyroscope";
		} else if (type.equals(Trace.GPS)) {
			table = "gps";
		} else if (type.equals(Trace.ROTATION_MATRIX)) {
			table = "rotation_matrix";
			dim = 9;
		} else  {
			assert 0 == 1;
		}
		
		try {
			statement = connection.createStatement();
		    rs = statement.executeQuery("select * from ".concat(table));
		    while(rs.next()) {
		    	Trace trace = new Trace(dim);
		    	long time = Long.parseLong(rs.getString("time"));

		    	for(int i = 0; i < dim; ++i) {
		    		trace.values[i] = Double.parseDouble(rs.getString("x".concat(String.valueOf(i))));
		    	}
		    	trace.time = time - start;
		    	res.add(trace);
		    }
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	    close();
	    return res;
	}
	
	
	
	public static List<Trace> loadOBDData(String path, long start, String type) {
	
		Log.log(TAG, "Load OBD data from " + path);
		connect(prefix.concat(path));
		
		List<Trace> res = new ArrayList<Trace>();
		Statement statement = null;
		ResultSet rs = null;
		//long start = Long.MAX_VALUE;
	    Trace trace = null;
	    String cmdName = "", cmdResult = "";
		try {
			statement = connection.createStatement();
		    rs = statement.executeQuery("select * from records");

		    while(rs.next()) {
		    	trace = new Trace(1);
		    	long time = Long.parseLong(rs.getString("time"));
		    	cmdName = rs.getString("cmdName");
		    	cmdResult = rs.getString("cmdResult");
		    	if(!type.equals(cmdName)) {
		    		continue;
		    	}
		    	trace.time = time - start;
		    	int len = cmdResult.length();
		    	trace.values[0] = Integer.parseInt(cmdResult.substring(0, len - 4));
		    	trace.type = type;
		    	res.add(trace);
		    }
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
	    close();
	    return res;
	}
	

}
