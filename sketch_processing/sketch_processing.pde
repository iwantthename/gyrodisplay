import processing.serial.*;
 
PShape box1;
PVector v_center, v_camera;
Serial myPort;
int lf = 10;
String str_get_data = null;
String buf[];
float ax,ay,az;
 
void setup(){
 
  size(500,500,P3D);
 
  v_center = new PVector(0.0,0.0,0.0);
  v_camera = new PVector(700.0,900.0,700.0);
 
  box1 = createShape(BOX,100,100,100);
  box1.translate(50,50,50);
   
  myPort = new Serial(this, "COM7", 9600);  
}
 
void draw(){
  background(200);
 
  //枠  
  /*
  noFill();
  strokeWeight(1);
  stroke(0,0,0);
  translate(250, 250, 250);
  box(500, 500, 500);
  translate(-250, -250, -250);
  */
   
  //XYZ方向
  strokeWeight(3);
  stroke(255,0,0);
  line(0,0,0,500,0,0);
  stroke(0,255,0);
  line(0,0,0,0,500,0);
  stroke(0,0,255);
  line(0,0,0,0,0,500);
  
  //オブジェクト配置
  shape(box1);
   
  //カメラ配置
  camera(v_camera.x, v_camera.y, v_camera.z, v_center.x,v_center.y,v_center.z, 0.0, 0.0, -1.0);
 
}
 
void serialEvent(Serial myPort){
  str_get_data = myPort.readStringUntil(lf);
        
  if (str_get_data != null){
    str_get_data = trim(str_get_data);    //改行コード取り除き
    buf = split(str_get_data,",");
     
    try{
      fill(255,127,0);
      stroke(0,0,0);
      /*
      ax = float(buf[0])/100*2;
      ay = float(buf[1])/100*2;
      */
      ax = -1.0*float(buf[0]);
      ay = float(buf[1]);
      print(ax);
      print(",");
      println(ay);
      /*
      box1 = createShape(BOX,ax,ay,100);
      box1.translate(ax/2,ay/2,100);
      */
      box1 = createShape(BOX,500,500,500);
      box1.rotateX(radians(ax));
      box1.rotateY(radians(ay));
    } catch(Exception e) {
      e.printStackTrace();
    }
  }
}
