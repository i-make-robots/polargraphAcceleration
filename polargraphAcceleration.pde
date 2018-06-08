//---------------------------------
// Visualize the maximum acceleration of a polargraph at any give position.
//
// please visit http://marginallyclever.com to check out our other great robot stuff.
//
// 2018-06-08 dan@marginallyclever.com
// Written in Processing 3.3.7
// CC-BY-SA-NC (https://creativecommons.org/licenses/by-nc-sa/4.0/)
//---------------------------------

class Machine {
  public float w,h;  // machine dimensions
  public float x,y;  // plotter position
  public float l,r;  // right and left belt length
  public float p;  // pulley diameter
  public float k1;  // motor maximum acceleration
  public float k2;  // motor maximum acceleration

  Machine() {
    w=width;
    h=height;
    x=w/2;
    y=h/2;
    p = 4.0/PI;
    k1=100;
    k2=100;
  }
  
  // update l,r based on x,y and w,h
  void IK() {
    float adj = x;
    float opp = y;
    l = sqrt(adj*adj+opp*opp);
    adj = w - x;
    r = sqrt(adj*adj+opp*opp);
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
};


Machine actual;
final float GRAVITYx=0;
final float GRAVITYy=9.8;


void setup() {
  size(650,800);  // machine size
  strokeWeight(1);
  actual = new Machine();
}


void mouseDragged() {
  float mdx = mouseX - pmouseX;
  float mdy = mouseY - pmouseY;
  actual.x+=mdx;
  actual.y+=mdy;
  // will update entire picture on next draw()
}


void draw() {
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
  
  actual.IK();
  actual.FK();
  
  float R1x = (    0 - actual.x) / actual.l;
  float R1y = (    0 - actual.y) / actual.l;
  float R2x = (width - actual.x) / actual.r;
  float R2y = (    0 - actual.y) / actual.r;
  // draw R1 and R2 to confirm
  stroke(0,0,255);
  line(actual.x,
  actual.y,
  actual.x+R1x*10,
  actual.y+R1y*10);

  stroke(0,255,255);
  line(actual.x,
  actual.y,
  actual.x+R2x*10,
  actual.y+R2y*10);
  
  stroke(255,0,255);
  line(actual.x,
  actual.y,
  actual.x+GRAVITYx*10,
  actual.y+GRAVITYy*10);

  // draw a bounding region showing the maximum acceleration around this point
  float ax = R1x * actual.k1 + GRAVITYx;
  float ay = R1y * actual.k1 + GRAVITYy;
  float bx = R2x * actual.k2 + GRAVITYx;
  float by = R2y * actual.k2 + GRAVITYy;
    
  triangle(
    actual.x+ax,
    actual.y+ay,
    actual.x+bx,
    actual.y+by,
    actual.x+GRAVITYx,
    actual.y+GRAVITYy
  );
    
    
  // if T is your target direction unit vector,
  
  float i;
  for(i=0;i<360;++i) {
    float Tx = sin(radians(i));
    float Ty = cos(radians(i));
    // solve cT = -gY + k1 R1 for c [and k1]
    // solve cT = -gY + k2 R2 for c [and k2]
    float c1 = dot(ax,ay,Tx,Ty);
    float c2 = dot(bx,by,Tx,Ty);
    
    float cT=0;
    // If c is negative, that means that that support rope doesn't limit the
    // acceleration; discard that c.
    if( c1<0 && c2<0 ) continue;
    // If c is positive in both cases, take the smaller one.
    if( c1>0 && c2>0 ) {
      cT = ( c1 < c2 ) ? c1 : c2;
    } else if(c1>0) cT=c1;
    else cT=c2;

    // You maximum acceleration is given by cT.
    stroke(255,255,0);
    point(
      actual.x+Tx * cT,
      actual.y+Ty * cT
    );
  }
}  

float dot(float ax,float ay,float bx,float by) {
  return ax*bx + ay*by;
}
