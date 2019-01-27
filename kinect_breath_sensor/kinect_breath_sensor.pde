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

HashMap<Integer, ArrayList<Double>> avgDistMap = new HashMap();
HashMap<Integer, ArrayList<Integer>> lastFiveMap = new HashMap();


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
  table.addColumn("distance1");
  table.addColumn("distance2");
  table.addColumn("distance3");
}

void draw() {
  kinect.update();
  image(kinect.userImage(), 0, 0); // Black and white point cloud'
  int[] depth = kinect.depthMap();
  IntVector userList = new IntVector();
  kinect.getUsers(userList);
  
  int now = millis();
  TableRow newRow = table.addRow();
  newRow.setInt("timestamp", now);

  for (int i = 0; i < userList.size(); i ++) {
    int userId = userList.get(i); 

    PVector min = new PVector();
    PVector max = new PVector();

    kinect.getBoundingBox(userId, min, max);

    float chestWidth = (max.x - min.x) * 0.75;

    // Get centre of mass
    kinect.getCoM(userId, com);
    kinect.convertRealWorldToProjective(com, com2d);

    noFill();
    stroke(255, 255, 255);
    rect(com2d.x - chestWidth/2, com2d.y - chestWidth/2, chestWidth, chestWidth);

    // Get box around centre of mass
    int[] breathBox = getBreathBox(depth, (int)com2d.x, (int)com2d.y, (int)chestWidth);

    double avg = avgDist(breathBox);
    
    text(userId, com2d.x, com2d.y + 40);

    ArrayList<Double> longAvg;
    ArrayList<Integer> lastFive;
    ArrayList<Double> shortAvg = new ArrayList();

    if (!avgDistMap.keySet().contains(userId)) {
      longAvg = new ArrayList();
      lastFive = new ArrayList();

      avgDistMap.put(userId, longAvg);
      lastFiveMap.put(userId, lastFive);
    } else {
      longAvg = avgDistMap.get(userId);
      lastFive = lastFiveMap.get(userId);
    }

    longAvg.add(avg);
    if (longAvg.size() > longAvgLength) {
      longAvg.remove(0);
      shortAvg = new ArrayList(longAvg.subList(longAvg.size() - 20, longAvg.size()));
    }

    // Counting
    if (longAvg.size() == longAvgLength) {
      if (getAvg(shortAvg) > (getAvg(longAvg) + minAmp) && !in) {
        lastFive.add(now);
        if (lastFive.size() > 5) {
          lastFive.remove(0);
          rate = 5.0 * 60000 / (lastFive.get(4) - lastFive.get(0));
          println("Breathing rate: " + rate + " per minute");
          text((int) rate, com2d.x, com2d.y);
        }
        breaths ++;
        in = true;
      }

      if (getAvg(shortAvg) < getAvg(longAvg) - minAmp && in) {
        in = false;
      }
    }
    
    switch(userId){
      case 1:
        newRow.setDouble("distance1", avg);
        break;
      case 2:
        newRow.setDouble("distance2", avg);
        break;
      case 3:
        newRow.setDouble("distance3", avg);
        break;
    }

    if (now > 120E3) {
      saveTable(table, "breathing.csv");
      exit();
    }
  }
}

double avgDist(int[] data) {
  int sum = 0;
  int count = 0;

  for (int i = 0; i < data.length; i++) {
    if (data[i] != 2047) {
      sum += data[i];
      count++;
    }
  }

  return (double) sum / count;
}

double getAvg(ArrayList<Double> data) {
  double sum = 0;
  for (int i = 0; i < data.size(); i++) sum += data.get(i);

  return sum/data.size();
}

int[] getBreathBox(int[] depth, int x, int y, int chestWidth) {
  int[] trimmed = new int[depth.length];

  int space = chestWidth / 2;

  for (int i = 0; i < width; i += 1) {
    for (int j = 0; j < height; j += 1) {
      if (i > x - space && i <= x + space && j > y - space && j <= y + space) {
        trimmed[j*width + i] = depth[j*width + i];
      } else {
        trimmed[j*width + i] = 2047;
      }
    }
  }

  return trimmed;
}
