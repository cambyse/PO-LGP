#include<MT/ors.h>
#include<MT/aico.h>
#include<MT/algos.h>
#include<MT/opengl.h>
#include<MT/plot.h>
#include<GL/gl.h>

#include<fstream>
#include<sstream>
#include<cmath>

#define LOSS_HOMING             (1e0)
#define LOSS_TIPS_POSITION      (1e-2)
#define LOSS_ORTHO_POSITION     (1e-1)
#define LOSS_ORTHO_ALIGN_TYPE1  (1e0)
#define LOSS_ORTHO_ALIGN_TYPE2  (5e-1)
//#define LOSS_JOINT_LIMITS       (1e1)   // breaks if too small
#define LOSS_NUM                (6)

#if defined LOSS_ORTHO_ALIGN_TYPE1 \
    || defined LOSS_ORTHO_ALIGN_TYPE2
#define LOSS_ORTHO_ALIGN_ANY
#endif

#if !defined LOSS_HOMING \
    && !defined LOSS_TIPS_POSITION \
    && !defined LOSS_ORTHO_POSITION \
    && !defined LOSS_ORTHO_ALIGN_ANY \
    && !defined LOSS_JOINT_LIMITS
#error  At least one loss macro must be defined
#endif

const double pi=4*atan(1);

void testHand();
void readCSVline(std::ifstream &file,int &id,int &hindex, int &sensor,int &JUNK,ors::Vector &p,ors::Vector &r);
ors::Transformation getRotationT(ors::Transformation t);
void showTransformation(ors::Transformation t);
double maxZero(double n);
int identity(bool b);

int main(int argc,char **argv){
    testHand();
    return 0;
}

void testHand() {
    const uint num_fings=5;

    // tips and knucks represent the body names of the last and first "bodies" of each finger.
    const char *tips[num_fings]={
        "fing0d",
        "fing1d",
        "fing2d",
        "fing3d",
        "fing4d"
    };
    const char *knucks[num_fings]={
        "fing0a",
        "fing1a",
        "fing2a",
        "fing3a",
        "fing4a"
    };

    // f -> finger index (defined as 0: Thumb, 1: Index, etc..)
    // j -> joint index (obtained by the ors model)
    // s -> sensor index (hardcoded, determined by the HW setup)
    // h -> hub index (hard coded, determined by the HW setup)
    int findex,jindex,sindex,hindex;
    int fid_to_bid[num_fings];
    int fid_to_jid[num_fings];
    int sid_to_fid[num_fings+1]={0,0,1,2,3,4}; // the first one is a dummy offset

    // ors objects and related init
    ors::Graph G;
    OpenGL gl;
    init(G,gl,"models/hand.ors");

    uint n=G.getJointStateDimension();
    cout << "Joint dimension: " << n << endl;

    // indexes' and translation tables' init
    for(uint i=0;i<num_fings;i++) {
        fid_to_bid[i]=G.getBodyIndexByName(tips[i]);
        fid_to_jid[i]=G.getBodyByName(knucks[i])->inLinks(0)->index;
        cout << "bid(\"" << tips[i] << "\"): " << fid_to_bid[i] << endl;
        cout << "jid(\"" << knucks[i] << "\"): " << fid_to_jid[i] << endl;
    }


    double m,qi,qlo,qhi,qtmp;
    // Task-specific loss, and 
    double Lp,Ltot;
    arr q(n);
    arr Phi,PhiJ,PhiV[LOSS_NUM],W;
    arr J,JR,Jtmp;
    arr y,yR,Y[num_fings];

    ors::Body *bStip,*bSorthoP,*bMorthoP,*bSorthoV,*bMorthoV;

    std::ifstream file("tracks/individual.csv");

    for(uint N=0;N<num_fings;N++)
        Y[N]=ARR(0,0,0);

    ors::Vector p,r,sortho,mortho;
    ors::Vector morthoV(0,.05,0);
    ors::Transformation T_palm_set,T_set_palm,T_model_palm,T_model_set,T_set_track[num_fings];
    int S,JUNK;

    // Misc.
    uint t;

    // press a button to start
    //cout << "Press Enter to start." << endl;
    //getchar();

    W.setDiag(1.,n);
    m=5*pi/180;
    qhi=10*pi/180;
    qlo=-qhi;

    cout << std::showpos << std::fixed << std::setprecision(4) << std::setw(6);
    gl.text << std::showpos << std::fixed << std::setprecision(2) << std::setw(6);
    bool no_palm=true;
    for(t=0;file.good();t++) {
        readCSVline(file,sindex,hindex,S,JUNK,p,r);

        // if palm sensor
        if(sindex==6) {
            no_palm=false;

            cout << endl;
            cout << "## Palm Found: " << p << endl;
            cout << "               " << r << endl;
            cout << endl;

            // update palm transformations
            T_set_palm.setZero();

            T_set_palm.addRelativeTranslation(p.x,p.y,p.z);

            T_set_palm.addRelativeRotationDeg(r.x,0,0,1);
            T_set_palm.addRelativeRotationDeg(r.y,0,1,0);
            T_set_palm.addRelativeRotationDeg(r.z,1,0,0);

            T_palm_set.setInverse(T_set_palm);

            T_model_palm.setZero();
            T_model_palm.addRelativeRotationDeg(90,1,0,0);
            T_model_palm.addRelativeRotationDeg(90,0,0,1);

            T_model_set=T_model_palm*T_palm_set;

            // Joints should stay close to zero
            G.getJointState(q);
            J.setDiag(1.,n);
            
            // initialization
            Phi.setZero();
            PhiJ.setZero();
            for(int i=0;i<10;i++)
                PhiV[i].setZero();

#ifdef LOSS_HOMING
            Phi=q/(LOSS_HOMING);
            PhiJ=J/(LOSS_HOMING);
            PhiV[0].append(q/(LOSS_HOMING));
#endif

            // Tracking the finger tips
            for(uint N=0;N<num_fings;N++) {
                // finds current points
                G.kinematics(y,fid_to_bid[N]);
                G.jacobian(J,fid_to_bid[N]);
                G.kinematicsVec(yR,fid_to_bid[N],&morthoV);
                G.jacobianVec(JR,fid_to_bid[N],&morthoV);

                // finds bodies relative to finger
                bStip=G.getBodyByName(STRING("stip"<<N));
                bSorthoP=G.getBodyByName(STRING("sorthoP"<<N));
                bMorthoP=G.getBodyByName(STRING("morthoP"<<N));
                bSorthoV=G.getBodyByName(STRING("sorthoV"<<N));
                bMorthoV=G.getBodyByName(STRING("morthoV"<<N));

#ifdef LOSS_TIPS_POSITION
                // visualizes target fingertip points
                bStip->X.pos=Y[N];
                // eval cost function contribution (distance between positions)
                Phi.append((y-Y[N])/(LOSS_TIPS_POSITION));
                PhiJ.append(J/(LOSS_TIPS_POSITION));
                PhiV[1].append((y-Y[N])/(LOSS_TIPS_POSITION));
#endif

#ifdef LOSS_ORTHO_POSITION
                // visualize sensor orthogonal points
                sortho=getRotationT(T_model_set*T_set_track[N])*-Vector_z;
                bSorthoP->X.pos=Y[N]+.05*sortho;
                // visualize model orthogonal points
                bMorthoP->X.pos=y+yR;
                // eval cost function contribution (distance between orthogonals)
                Phi.append((y+yR-ARRAY(bSorthoP->X.pos))/(LOSS_ORTHO_POSITION));
                PhiJ.append((J+JR)/(LOSS_ORTHO_POSITION));
                PhiV[2].append((y+yR-ARRAY(bSorthoP->X.pos))/(LOSS_ORTHO_POSITION));
#endif

#ifdef LOSS_ORTHO_ALIGN_ANY
                // visualize sensor orthogonal points
                sortho=getRotationT(T_model_set*T_set_track[N])*-Vector_z;
                bSorthoV->X.setZero();
                bSorthoV->X.appendTransformation(getRotationT(T_model_set*T_set_track[N]));
                bSorthoV->X.pos=Y[N]+(.05*sortho)*.5;
                // visualize model orthogonal points
                bMorthoV->X.setZero();
                bMorthoV->X.appendTransformation(getRotationT(G.getBodyByName(STRING("fing"<<N<<"d"))->X));
                bMorthoV->X.addRelativeRotationDeg(-90,1,0,0); // TODO only doing this because it seems impossible to specify a body extending in other directions..
                bMorthoV->X.pos=y+(yR*.5);
#endif

                ors::Vector a;
                ors::Vector b;
#ifdef LOSS_ORTHO_ALIGN_TYPE1
                // eval cost function contribution (distance between orthogonals)
                a=yR;
                b=.05*sortho;
                a*=20;
                b*=20;
                Jtmp=-(~ARRAY(b)*JR/sqrt(1-(a*b)*(a*b)));
                Phi.append(acos(a*b)/(LOSS_ORTHO_ALIGN_TYPE1));
                PhiJ.append(Jtmp/(LOSS_ORTHO_ALIGN_TYPE1));
                PhiV[3].append(acos(a*b)/(LOSS_ORTHO_ALIGN_TYPE1));
#endif

#ifdef LOSS_ORTHO_ALIGN_TYPE2
                // eval cost function contribution (distance between orthogonals)
                a=yR;
                b=.05*sortho;
                a*=20;
                b*=20;
                Jtmp=-(~ARRAY(b)*JR);
                Phi.append((1-a*b)/(LOSS_ORTHO_ALIGN_TYPE2));
                PhiJ.append(Jtmp/(LOSS_ORTHO_ALIGN_TYPE2));
                PhiV[4].append((1-a*b)/(LOSS_ORTHO_ALIGN_TYPE2));
#endif
            }
#ifdef LOSS_JOINT_LIMITS
            qtmp=0;
            Jtmp=zeros(1,n);
            for(uint N=0;N<num_fings;N++) {
                jindex=fid_to_jid[N];
                qi=q(jindex);
                qtmp+=(maxZero(m-qi+qlo)+maxZero(m+qi-qhi))/m;
                Jtmp(0,jindex)=(identity((m-qi+qlo)>0)+identity((m+qi-qhi)>0))/m;
            }
            Phi.append(qtmp/LOSS_JOINT_LIMITS);
            PhiJ.append(Jtmp/(LOSS_JOINT_LIMITS));
            PhiV[5].append(qtmp/LOSS_JOINT_LIMITS);
#endif

            q-=inverse(~PhiJ*PhiJ+W)*~PhiJ*Phi;
            G.setJointState(q);
            G.calcBodyFramesFromJoints();

            gl.text.clear();
            gl.text << "t = " << t << endl;

            Ltot=0;
#ifdef LOSS_HOMING
            Lp=(~PhiV[0]*PhiV[0]).scalar();
            Ltot+=Lp;
            gl.text << "Homing: " << Lp << endl;
#endif // LOSS_HOMING
#ifdef LOSS_TIPS_POSITION
            Lp=(~PhiV[1]*PhiV[1]).scalar();
            Ltot+=Lp;
            gl.text << "Tips_pos: " << Lp << endl;
#endif // LOSS_TIPS_POTISION
#ifdef LOSS_ORTHO_POSITION
            Lp=(~PhiV[2]*PhiV[2]).scalar();
            Ltot+=Lp;
            gl.text << "Ortho_pos: " << Lp << endl;
#endif // LOSS_ORTHO_POSITION
#ifdef LOSS_ORTHO_ALIGN_TYPE1
            Lp=(~PhiV[3]*PhiV[3]).scalar();
            Ltot+=Lp;
            gl.text << "Ortho_align_1: " << Lp << endl;
#endif // LOSS_ORTHO_ALIGN_TYPE1
#ifdef LOSS_ORTHO_ALIGN_TYPE2
            Lp=(~PhiV[4]*PhiV[4]).scalar();
            Ltot+=Lp;
            gl.text << "Ortho_align_2: " << Lp << endl;
#endif // LOSS_ORTHO_ALIGN_TYPE2
#ifdef LOSS_JOINT_LIMITS
            Lp=(~PhiV[5]*PhiV[5]).scalar();
            Ltot+=Lp;
            gl.text << "Joint_limits: " << Lp << endl;
#endif // LOSS_JOINT_LIMITS
            gl.text << "Tot: " << Ltot << endl;
            gl.update();

            cout << "Loss Function: " << Ltot << endl << endl;
            cout << "##############################" << endl << endl;

            continue;
        }
        if(no_palm)
            continue;

        findex=sid_to_fid[sindex];
        // apply palm transformation to tracked finger position
        p=T_model_set*p;
        cout << "Finger " << tips[findex] << ": " << p << endl;

        // TODO fix the translation / scale stuff
        double sx=2;
        double sy=2;
        double sz=2;
        double tx=-.035;
        double ty=.025;
        double tz=.485;

        Y[findex]=ARRAY(
                ors::Vector(sx*p.x,sy*p.y,sz*p.z)+
                ors::Vector(tx,ty,tz)
                );

        T_set_track[findex].setZero();

        T_set_track[findex].addRelativeRotationDeg(r.x,0,0,1);
        T_set_track[findex].addRelativeRotationDeg(r.y,0,1,0);
        T_set_track[findex].addRelativeRotationDeg(r.z,1,0,0);
    }
    return;
}

void readCSVline(std::ifstream &file,int &sindex,int &hindex, int &sensor,int &JUNK,ors::Vector &p,ors::Vector &r) {
    std::stringstream line;
    char buff[256];

    file.getline(buff,256);
    line << buff;

    line >> sindex;
    line.ignore(100,',');
    line >> hindex;
    line.ignore(100,',');
    line >> sensor;
    line.ignore(100,',');
    line >> JUNK;
    line.ignore(100,',');
    line >> JUNK;
    line.ignore(100,',');

    double x,y,z;
    line >> x;
    line.ignore(100,',');
    line >> y;
    line.ignore(100,',');
    line >> z;
    line.ignore(100,',');
    // convert cm -> m
    p.set(x/100,y/100,z/100);
    
    line >> x;
    line.ignore(100,',');
    line >> y;
    line.ignore(100,',');
    line >> z;
    line.ignore(100,',');
    r.set(x,y,z);
}

ors::Transformation getRotationT(ors::Transformation t) {
    double m[16];
    t.getAffineMatrix(m);
    m[3]=m[7]=m[11]=0;

    ors::Transformation out;
    out.setAffineMatrix(m);

    return out;
}


void showTransformation(ors::Transformation t) {
    double m[16];
    t.getAffineMatrix(m);
    cout << "m: " << endl;
    for(int i=0;i<4;i++) {
        for(int j=0;j<4;j++) {
            cout << m[i*4+j] << " ";
        }
        cout << endl;
    }
}

inline double maxZero(double n) {
    return n>0?n:0;
}

inline int identity(bool b) {
    return b?1:0;
}

