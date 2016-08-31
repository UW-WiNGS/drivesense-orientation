package io;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import utility.Log;


public class DirectoryWalker {

	private final static String TAG = "DirectoryWalker";
	private static List<String> files_;
	
	/*rules to match files:*/
	
	
	/**
	 * get all the files under the directory of root
	 * 
	 * */
	public static List<String> recursive(String root) {
		files_ = new ArrayList<String>();
		File cur = new File(root);
		if(cur.exists()) {
			walk(cur.getAbsoluteFile());	
		} else {
			Log.log(root + " does not exists!");
		}
		return files_;
	}
	
	public static void createFolder(String directory) {
		Log.log(TAG, "create folder:" + directory);
		File dir = new File(directory);
		if(dir.exists()) {
			Log.log(TAG, directory + " already exists");
			return;
		}
		if(true != dir.mkdirs()) {
			Log.log(TAG, "create folder error: folder:" + directory);
		}
	}
	
	/**
	 * 
	 * get all the files under directory
	 * @param directory
	 * @return
	 */
	public static List<String> getFileNames(String directory) {
		File cur = new File(directory);
		List<String> res = new ArrayList<String>();
		File[] list = cur.listFiles();
		for ( File f : list ) {
			if ( f.isFile() ) {    
				res.add(f.getName());
			}
		}
		return res;
	}
	
	/**
	 * 
	 * get all the folders under directory
	 * @param directory
	 * @return
	 */
	public static List<String> getFolders(String directory) {
		File cur = new File(directory);
		List<String> res = new ArrayList<String>();
		File[] list = cur.listFiles();
		for ( File f : list ) {
			if ( f.isDirectory() ) {    
				res.add(f.getAbsolutePath());
			}
		}
		return res;
	}
	
	private static void walk(File root) {		
		File[] list = root.listFiles();
		if (list == null) return;
		for ( File f : list ) {
			if ( f.isDirectory() ) {    
				walk(f);   
			}
            else {
            	files_.add(f.getAbsolutePath());
            }
        }
	}
	
	public static boolean deleteDir(String path) {
		File dir = new File(path);
		if (dir.isDirectory()) {
			String[] children = dir.list();
			for (int i = 0; i < children.length; i++) {
				boolean success = deleteDir(path.concat("/" + children[i]));
	            if (!success) {
	               return false;
	            }    
			}
		}
		return dir.delete();  
	}
	
	public static void delete(String path) {
		File f = new File(path);
		if(!f.exists()) {
			return;
		}
		f.delete();
	}
}
