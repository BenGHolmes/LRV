import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import SimpleOpenNI.*;

SimpleOpenNI kinect;

PVector com = new PVector();
PVector com2d = new PVector();
Table table;

// Rectangle filters
int above = 75;
int below = 25;
int side = 25;
int width = 640;
int height = 480;

// Timer variables
float a = 0;

// Breath rates
double minAmp = 0.5;
int longAvgLength = 400;
int shortAvgLength = 20;
int breaths = 0;
double rate = 10;
int diff5 = 0;
boolean in = false;
boolean warned = false;

ArrayList<Double> longAvg = new ArrayList();
ArrayList<Double> shortAvg = new ArrayList();
ArrayList<Integer> breathAvg = new ArrayList();

// Used to avoid extra calculations
float[] depthLookUp = new float[2048];

void setup() {
  size(640, 480);
  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  //kinect.enableIR();
  kinect.enableUser();// because of the version this change
  //size(640, 480);
  fill(255, 0, 0);
  //size(kinect.depthWidth()+kinect.irWidth(), kinect.depthHeight());
  kinect.setMirror(false);
  
  table = new Table();
  table.addColumn("timestamp");
  table.addColumn("distance");
}

void draw() {
  kinect.update();
  image(kinect.userImage(),0,0); // Black and white point cloud'
  int[] depth = kinect.depthMap();
  IntVector userList = new IntVector();
  kinect.getUsers(userList);
  
  for (int i = 0; i < userList.size(); i ++) {
    int userId = userList.get(0); 
    
    // Get centre of mass
    kinect.getCoM(userId, com);
    kinect.convertRealWorldToProjective(com,com2d);
    
    noFill();
    rect(com2d.x - side, com2d.y - above, 2*side, above + below);
    
    // Get box around centre of mass
    int[] breathBox = getBreathBox(depth, (int)com2d.x, (int)com2d.y);
    
    double avg = avgDist(breathBox);
    int now = millis();
    
    TableRow newRow = table.addRow();
    newRow.setInt("timestamp", now);
    newRow.setDouble("distance", avg);
    
    longAvg.add(avg);
    shortAvg.add(avg);
    
    if(longAvg.size() > longAvgLength) longAvg.remove(0);
    if(shortAvg.size() > shortAvgLength) shortAvg.remove(0);
    
    // Counting
    if(longAvg.size() == longAvgLength){
      if(getAvg(shortAvg) > (getAvg(longAvg) + minAmp) && !in){
        breathAvg.add(now);
        if(breathAvg.size() > 5){
          breathAvg.remove(0);
          rate = 5.0 * 60000 / (breathAvg.get(4) - breathAvg.get(0));
          println("Breathing rate: " + rate + " per minute");
        }
        breaths ++;
        in = true;
      }
      
      if(getAvg(shortAvg) < getAvg(longAvg) - minAmp && in){
        in = false;
      }
    }
    
    if(now > 120E3){
      saveTable(table, "breathing.csv");
      exit();
    }
    
    if(breathAvg.size() > 0) diff5 = now - breathAvg.get(0);
    
    // Warnings
    if(rate < 8){ if(!warned) println("WARNING: BREATH RATE LOW");}
    else if(rate > 20){ if (!warned) println("WARNING: BREATH RATE HIGH");}
    else if(diff5 > 40){ if (!warned) println("WARNING: NO BREATHING DETECTED");}
    else warned = false;
  }
}

double avgDist(int[] data){
   int sum = 0;
   int count = 0;
   
   for(int i = 0; i < data.length; i++){
     if(data[i] != 2047){
       sum += data[i];
       count++;
     }
   }
   
   return (double) sum / count;
}

double getAvg(ArrayList<Double> data){
  double sum = 0;
  for(int i = 0; i < data.size(); i++) sum += data.get(i);
  
  return sum/data.size();
}

int[] getBreathBox(int[] depth, int x, int y){
  int[] trimmed = new int[depth.length];

  for (int i = 0; i < width; i += 1) {
    for (int j = 0; j < height; j += 1) {
      if(i > x - side && i <= x + side && j > y - above && j <= y + below){
        trimmed[j*width + i] = depth[j*width + i];
      } else {
        trimmed[j*width + i] = 2047;
      }
    }
  }

  return trimmed;
}
