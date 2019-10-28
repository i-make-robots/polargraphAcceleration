//---------------------------------
// Visualize the maximum acceleration of a polargraph at any given position.
// Also accelerate the polargraph with PID tuned motors.
//
// please visit http://marginallyclever.com to check out our other great robot stuff.
//
// Special thanks to https://www.reddit.com/user/zebediah49 for his math help.
//
// 2018-06-08 dan@marginallyclever.com
// Written in Processing 3.3.7
// CC-BY-SA-NC (https://creativecommons.org/licenses/by-nc-sa/4.0/)
//---------------------------------

final float GRAVITYx=0;
final float GRAVITYy=1;
final float GRAVITYmag=9.80;
final float MS_PER_FRAME = 1000/30;
final float MOTOR_MAX_ACCELERATION = 800.0;  // steps-per-second
final float MOTOR_MAX_VELOCITY = 1000000.0/50.0; // 50us per step


class Machine {
  public float w,h;  // machine dimensions
  public float x,y;  // plotter position
  public float l,r;  // right and left belt length
  public float lv,rv;
  public float la,ra;
  public float p;  // pulley diameter
  public float R1x, R1y, R2x, R2y;

  Machine() {
    w=width;
    h=height;
    x=w/2;
    y=h/2;
    p = 4.0/PI;
    lv=0;
    rv=0;
    la=0;
    ra=0;
  }
  
  // update l,r based on x,y and w,h
  void IK() {
    l = findLeft(x,y);
    r = findRight(x,y);
    
    // normal vectors pointing from plotter to motor
    R1x = (0 - x) / l;
    R1y = (0 - y) / l;
    R2x = (w - x) / r;
    R2y = (0 - y) / r;
  }
  
  float findLeft(float x,float y) {
    float adj = x;
    float opp = y;
    return sqrt(adj*adj+opp*opp);
  }
  
  float findRight(float x,float y) {
    float adj = w-x;
    float opp = y;
    return sqrt(adj*adj+opp*opp);
  }
  
  // update x,y based on l,r and w,h
  void FK() {
    float a = l;
    float b = w;
    float c = r;
    float theta = ((a*a+b*b-c*c)/(2.0*a*b));
    x = theta * a;
    y = (sqrt( 1.0 - theta * theta ) * a);
  }
  
  void teleport(float xx,float yy) {
    x=xx;
    y=yy;
    IK();
  }
  
  // get maximum acceleration in cartesian direction T(x,y) at point actual(x,y)
  // @param Tx unit vector x component
  // @param Ty unit vector y component
  // @return positive number for limit.  if result is negative it means there is no limit.
  float getMaxT(float Tx,float Ty) {
    // solve cT = -gY + k1 R1 for c [and k1]
    // solve cT = -gY + k2 R2 for c [and k2]
    /*
     * c Tx = k Rx
     * c Ty = -g + k Ry
     * k = c Tx / Rx
     * c Ty = -g + (c Tx / Rx) Ry
     * g = c (Tx Ry / Rx - Ty)
     * c = g / [ (Tx Ry - Ty Rx) /Rx ]
     *   = g Rx / (Tx Ry - Rx Ty)
     */
    float c1 = -GRAVITYy*GRAVITYmag * R1x / (Tx*R1y - Ty*R1x);
    float c2 = -GRAVITYy*GRAVITYmag * R2x / (Tx*R2y - Ty*R2x);
    
    //println(i+"\t"+c1+"\t"+c2);
    
    float cT=-1;
    // If c is negative, that means that that support rope doesn't limit the
    // acceleration; discard that c.
    if( c1>0 || c2>0 ) {
      // If c is positive in both cases, take the smaller one.
      if( c1>0 && c2>0 ) {
        cT = ( c1 < c2 ) ? c1 : c2;
      } else if(c1>0) cT=c1;
      else cT=c2;
    }
    return cT;
  }
};


class PID {
  public float Kp = 1.0;
  public float Ki = 0.0005;
  public float Kd = 0.4;
  public float oldP;
  public float i;

  public PID() {
    oldP=0;
    i=0;
  }
  
  public float update(float dt,float p) {
    i += p*dt;
    float d = (p-oldP)/dt;
    float u = (Kp*p + Ki*i + Kd*d);
    oldP=p;

    return u;
  }
};


Machine actual;
float tx=0,ty=0;
long lastFrame;
long nextFrame;

PID pid1,pid2;

HScrollBar hs1,hs2,hs3;

void setup() {
  size(650,800);  // machine size
  strokeWeight(1);
  actual = new Machine();
  
  pid1 = new PID();
  pid2 = new PID();
  pid1.Kp = pid2.Kp = 1.0;
  pid1.Ki = pid2.Ki = 0.005;
  pid1.Kd = pid2.Kd = 0.2;
        
  tx = actual.x;
  ty = actual.y;
  
  hs1 = new HScrollBar(10, 50, 100, 16, 1);
  hs2 = new HScrollBar(10, 70, 100, 16, 1);
  hs3 = new HScrollBar(10, 100, 100, 16, 1);
  hs1.setPos(pid1.Kp*5);
  hs2.setPos(pid1.Ki*2);
  hs3.setPos(pid1.Kd);
  
  lastFrame=nextFrame=millis();
}


void mouseDragged() {
  float mdx = mouseX - pmouseX;
  float mdy = mouseY - pmouseY;
  // move the target
  if(!hs1.overEvent() && !hs2.overEvent() && !hs3.overEvent()) {
    tx+=mdx;
    ty+=mdy;
  }
}


void draw() {
  long tnow = millis();
  if(tnow>=nextFrame) {
    nextFrame += MS_PER_FRAME;
    step((float)MS_PER_FRAME*0.001);
  }
  
  hs1.update();
  hs2.update();
  hs3.update();
  
  drawEverything();
  
  hs1.display();
  hs2.display();
  hs3.display();
  fill(255,255,255);
  text("P="+pid1.Kp,hs1.xpos+hs1.swidth+10,hs1.ypos+hs1.sheight);
  text("I="+pid1.Ki,hs2.xpos+hs2.swidth+10,hs2.ypos+hs2.sheight);
  text("D="+pid1.Kd,hs3.xpos+hs3.swidth+10,hs3.ypos+hs3.sheight);
}


int state=0;


void squareTest() {
  state=(state+1)%4;
  switch(state) {
    case 0:      tx=width*1/4;      ty=height*1/4;      break;
    case 1:      tx=width*3/4;      ty=height*1/4;      break;
    case 2:      tx=width*3/4;      ty=height*3/4;      break;
    case 3:      tx=width*1/4;      ty=height*3/4;      break;
  }
}


void circleTest() {
  state=(state+5)%360;
  tx = width /2+sin(radians(state))*width *0.25;
  ty = height/2+cos(radians(state))*height*0.25;
}

boolean first=true;

void step(float dt) {
  if(abs(tx-actual.x)<1 && abs(ty-actual.y)<1) {
    //squareTest();
    circleTest();
  }
  
  actual.IK();
  actual.FK();
  
  // the plotter head has an error term, d.
  float dx=tx - actual.x;
  float dy=ty - actual.y;
  // the error term in cartesian space
  float d = sqrt(dx*dx+dy*dy);
  if(d>0) {
    dx/=d;
    dy/=d;
  }
  
  // max cartesian acceleration in this direction at this point
  float maxT = actual.getMaxT(dx,dy);
  
  // the error term in polar space.
  switch(5) {
  case 1:{
      // the difference in velocity in polar space
      float lvn=dot(dx,dy,-actual.R1x,-actual.R1y)*d/100.0;
      float rvn=dot(dx,dy,-actual.R2x,-actual.R2y)*d/100.0;
      pid1.Kp = pid2.Kp = 1.0;
      pid1.Ki = pid2.Ki = 0.0000;
      pid1.Kd = pid2.Kd = 0.0;
      actual.la += pid1.update(dt,lvn); 
      actual.ra += pid2.update(dt,rvn); 
    } break;
  case 2:{
      // the difference in position in polar space
      float ld = (actual.findLeft (tx,ty) - actual.l)*5;
      float rd = (actual.findRight(tx,ty) - actual.r)*5;
      pid1.Kp = pid2.Kp = 1.0;
      pid1.Ki = pid2.Ki = 0.006;
      pid1.Kd = pid2.Kd = 0.6;
      float lvn = pid1.update(dt,ld); 
      float rvn = pid2.update(dt,rd); 
      actual.la = lvn-actual.lv;
      actual.ra = rvn-actual.rv;
    } break;
  case 3:{
      // the difference in position in cartesian space
      pid1.Kp = pid2.Kp = 0.7;
      pid1.Ki = pid2.Ki = 0.0;
      pid1.Kd = pid2.Kd = 0.2;
      float vx = pid1.update(dt,dx*d); 
      float vy = pid2.update(dt,dy*d);
      float lvn=dot(vx,vy,-actual.R1x,-actual.R1y);
      float rvn=dot(vx,vy,-actual.R2x,-actual.R2y);
      actual.la = lvn-actual.lv;
      actual.ra = rvn-actual.rv;
    } break;
  case 4:{
      float vx = dx*d; 
      float vy = dy*d;
      float lvn=dot(vx,vy,-actual.R1x,-actual.R1y);
      float rvn=dot(vx,vy,-actual.R2x,-actual.R2y);
      float lan = lvn-actual.lv;
      float ran = rvn-actual.rv;

      pid1.Kp = pid2.Kp = hs1.getPos()*50.0;
      pid1.Ki = pid2.Ki = hs2.getPos()*50.0;
      pid1.Kd = pid2.Kd = hs3.getPos();
      
      actual.la += pid1.update(dt,lan); 
      actual.ra += pid2.update(dt,ran);
      
      float nax = actual.la*actual.R1x + actual.ra * actual.R2x;
      float nay = actual.la*actual.R1y + actual.ra * actual.R2y;
      float a = sqrt(nax*nax+nay*nay);
      if(maxT>0 && maxT<a) {
        actual.la*= maxT/a;
        actual.ra*= maxT/a;
      }
    } break;
    case 5: {
      float ox = actual.x;
      float oy = actual.y;
      float ol = actual.l;
      float or = actual.r;
      actual.x =tx;
      actual.y =ty;
      actual.IK();
      float ld=actual.l-ol;
      float rd=actual.r-or;
      actual.x =ox;
      actual.y =oy;
      actual.IK();

      pid1.Kp = pid2.Kp = 300.0+actual.y;// + (width/2 - abs(width/2-actual.x));//hs1.getPos()*5;
      pid1.Ki = pid2.Ki =   2.0;//hs2.getPos()*2;
      pid1.Kd = pid2.Kd = 50.0;//hs3.getPos();
      
      actual.la += pid1.update(dt,ld); 
      actual.ra += pid2.update(dt,rd);
      if(first) {
        //println("LD\tRD\tLV\tRA\tLV\tRA");
        first=false;
      }
      //println(ld+"\t"+rd+"\t"+actual.lv+"\t"+actual.rv+"\t"+actual.la+"\t"+actual.ra);
      
      float nax = actual.la*actual.R1x + actual.ra * actual.R2x;
      float nay = actual.la*actual.R1y + actual.ra * actual.R2y;
      float a = sqrt(nax*nax+nay*nay);
      if(maxT>a) {
        actual.la *= maxT/a;
        actual.ra *= maxT/a;
      }
    } break;
  }
    
  float scale=1;
  if(MOTOR_MAX_ACCELERATION<abs(actual.la)) {
    scale = MOTOR_MAX_ACCELERATION/abs(actual.la); 
  }
  if(MOTOR_MAX_ACCELERATION<abs(actual.ra)) {
    scale = MOTOR_MAX_ACCELERATION/abs(actual.ra);
  }
  actual.la *= scale;
  actual.ra *= scale;
  
  actual.lv += actual.la*dt;
  actual.rv += actual.ra*dt;
  scale=1;
  if(abs(actual.lv)>MOTOR_MAX_VELOCITY) {
    scale = MOTOR_MAX_VELOCITY/abs(actual.lv);
  }
  if(abs(actual.rv)>MOTOR_MAX_VELOCITY) {
    scale = MOTOR_MAX_VELOCITY/abs(actual.rv);
  }
  actual.lv *= scale;
  actual.rv *= scale;
  
  actual.l+=actual.lv*dt;
  actual.r+=actual.rv*dt;
  actual.FK();
}


void drawEverything() {
  background(128,128,128);
  stroke(0,255,0);
  // pulley diameter
  noFill();
  stroke(255,0,0);
  ellipse(width /2 - actual.w/2,
          height/2 - actual.h/2,
          actual.p*100,
          actual.p*100);
  ellipse(width /2 + actual.w/2,
          height/2 - actual.h/2,
          actual.p*100,
          actual.p*100);

  // calculate maximum acceleration around actual.p
  // https://www.reddit.com/r/AskPhysics/comments/8pem1d/please_help_me_build_a_more_accurate_model_of_my/e0b97ee/
  // A = -g Y + k1 R1 + k2 R2
  
  
  // if T is your target direction unit vector,
  float i;
  for(i=0;i<360;++i) {
    float Tx = cos(radians(i));
    float Ty = sin(radians(i));
    float cT=actual.getMaxT(Tx,Ty);
    
    if(cT>0) {
      // You maximum acceleration is given by cT.
      stroke(255,128,0);
    } else {
      stroke(255,64,0);
      cT=10*atan2(1.0-actual.R1y,actual.R1x);
    }
      line(
        actual.x,
        actual.y,
        actual.x+Tx * cT*10,
        actual.y+Ty * cT*10
      );
  }
  // draw R1 and R2 to confirm
  stroke(0,0,255);
  line(actual.x,
      actual.y,
      actual.x-actual.R1x*actual.lv,
      actual.y-actual.R1y*actual.lv);

  stroke(0,255,0);
  line(actual.x,
      actual.y,
      actual.x-actual.R2x*actual.rv,
      actual.y-actual.R2y*actual.rv);
  
  // draw R1 and R2 to confirm
  stroke(0,0,128);
  line(actual.x,
      actual.y,
      actual.x-actual.R1x*actual.la,
      actual.y-actual.R1y*actual.la);

  stroke(0,128,0);
  line(actual.x,
      actual.y,
      actual.x-actual.R2x*actual.ra,
      actual.y-actual.R2y*actual.ra);
  
  stroke(255,192,0);  // target
  line(tx-10,ty,tx+10,ty);
  line(tx,ty-10,tx,ty+10);
}


float dot(float ax,float ay,float bx,float by) {
  return ax*bx + ay*by;
}
