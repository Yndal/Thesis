package core;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.GradientPaint;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Stroke;
import java.awt.geom.Ellipse2D;
import java.awt.image.BufferedImage;
import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Scanner;

import javax.imageio.ImageIO;

public class Main {
	private static Color[] colors = {
			Color.black,
			Color.blue,
			Color.red,
			Color.green,
			Color.magenta,
			Color.pink,
			Color.cyan,
			Color.yellow,
			Color.gray,
			Color.orange
			};
	
	
	private static char SEP_CHAR = '\t';
	private String imagePath;

	private BufferedImage bi;
	private Graphics2D ig2;
	//private Point lastPoint;

	private Scanner scanner;
	private LinkedList<LinkedList<Point>> tracks;

	public Main(String imgPath){
		//TODO Add preconditions

		this.imagePath = imgPath;
		loadImage();

		//this.lastPoint = null;
		this.tracks = null;
	}


	private void loadImage(){
		//bi = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
		try {
			this.bi = ImageIO.read(new File(imagePath));
		} catch (IOException e) {
		}

		this.ig2 = bi.createGraphics();
	}

	public void addTracks(String tracks, Stroke stroke){
		loadTracks(tracks);

		//TODO Define line width, etc
		
		
		ig2.setStroke(stroke);
		int colorCounter = 0;
		for(LinkedList<Point> points : this.tracks){
			Color color = colors[colorCounter++];
			ig2.setPaint(color);
					Point lastPoint = null;
				for(Point p : points){
					if(lastPoint == null){
						lastPoint = p;
						continue;
					}

					ig2.drawLine(lastPoint.x, lastPoint.y, p.x, p.y);
					lastPoint = p;
				}
			
			}
		if(colors.length <= colorCounter)
			colorCounter = 0;			
		}
		
		
		
		/*
		
		Point p;
		Iterator<LinkedList<Point>> it1 = this.tracks.iterator();
		while(it1.hasNext()){
			p = it1.next();
			if(this.lastPoint == null){
				this.lastPoint = p;
				continue;
			}

			ig2.drawLine(this.lastPoint.x, this.lastPoint.y, p.x, p.y);
			this.lastPoint = p;
		}		
	}*/

	private void loadTracks(String trackPaths){
		this.tracks = new LinkedList<>();
		File dir = new File(trackPaths);
		
		String[] files = dir.list(); 
		for(int i=0; i<files.length; i++){
			if(!files[i].endsWith("txt"))
				continue;
			String file = trackPaths +  files[i];
			LinkedList<Point> points = new LinkedList<Point>();
			try {
				scanner = new Scanner(/*"%Frame Number	x (image)	y (image)	x (world)	y (world)	Area	Orientation	Compactness	TimeStamp (ms)\n"
					+ "214	100.0	0.0	0	0	243	0.0523599	0.473516	7166\n"
					+ "214	941.845	248.403	0	0	130.5	4.79966	0.663611	7166\n"
					+ "215	925.997	285.246	0	0	232	0.0174533	0.494424	7200\n"
					+ "215	942.887	245.349	0	0	130	4.79966	0.70036	7200\n");*/
						new BufferedInputStream(new FileInputStream(new File(file))));
				while(scanner.hasNextLine()){
					try{
					String line = scanner.nextLine();
					if(line.startsWith("%") || line.isEmpty())
						continue;

					int space = line.indexOf(SEP_CHAR);
					//Remove frame number
					line = line.substring(space+1);
					space = line.indexOf(SEP_CHAR);

					double x = Double.valueOf(line.substring(0,space));
					line = line.substring(space+1);
					space = line.indexOf(SEP_CHAR);

					double y = Double.valueOf(line.substring(0,space));
					line = line.substring(space+1);
					space = line.indexOf(SEP_CHAR);

					//double xW = Double.valueOf(line.substring(0,space));
					line = line.substring(space+1);
					space = line.indexOf(SEP_CHAR);

					//double yW = Double.valueOf(line.substring(0,space));
					line = line.substring(space+1);
					space = line.indexOf(SEP_CHAR);

					//double area = Double.valueOf(line.substring(0,space));
					line = line.substring(space+1);
					space = line.indexOf(SEP_CHAR);

					//double orientation = Double.valueOf(line.substring(0,space));
					line = line.substring(space+1);
					space = line.indexOf(SEP_CHAR);

					//double compactness = Double.valueOf(line.substring(0,space));
					line = line.substring(space+1);
					//space = line.indexOf(SEP_CHAR);


					//long timestampMs = Long.valueOf(line);//.substring(0,space));

					points.add(new Point((int)x,(int)y));
					} catch (NumberFormatException nfe){
						//Ignore
					}


				}
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			this.tracks.add(points);
		}
	}


	public void save(){
		String pathAndName = this.imagePath.substring(0, this.imagePath.lastIndexOf('.'));
		String ext = this.imagePath.substring(this.imagePath.lastIndexOf('.'));
		pathAndName += "_withTracks" + ext;
		
		try{
			ImageIO.write(bi, "PNG", new File(pathAndName));
			System.out.println("Image has been updated!");
		}catch(IOException ioe){
			//Tough luck!
			System.err.println("An error occured while saving the image");
		}	
	}



	public static void main(String[] args) {
		//String imgPath = "D:\\OwnClouds\\Yndal\\Thesis\\SwisTrack\\Recordings\\Init design\\3 - Docking\\background.jpg";
		//String trackPath = "D:\\OwnClouds\\Yndal\\Thesis\\SwisTrack\\Recordings\\Init design\\3 - Docking\\Tracks\\";
		
		String imgPath = "/Users/yndal/OwnClouds/Yndal/Thesis/SwisTrack/Recordings/2 iteration/3 - Docking/background.jpg";
		String trackPath = "/Users/yndal/OwnClouds/Yndal/Thesis/SwisTrack/Recordings/2 iteration/3 - Docking/Tracks/";
		Main main = new Main(imgPath);
		main.addTracks(trackPath, new BasicStroke(3));
		main.save();
		//Main main = new Main(imgPath, trackPath, Color.green);
	}

}


