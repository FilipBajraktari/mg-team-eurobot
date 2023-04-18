/*
 * A background dameon implementation of real-time rrt* from [1]
 * Designed for the purposes of eurobot
 *
 * [1]Naderi, Kourosh & Rajamäki, Joose & Hämäläinen, Perttu.
 * (2015). RT-RRT*: a real-time path planning algorithm based 
 * on RRT*. 113-118. 10.1145/2822013.2822036. 
 * 
 * AUTHORS: Petar Vukotic
 */
#define _USE_MATH_DEFINES
#include <list>
#include <unordered_set>
#include <Python.h>
//#include <numpy/arrayobject.h>
//#include <math.h>
#include <cmath>
#include <queue>
#include <cstring>
#include <time.h>
#include <random>

using namespace std;


/**
 *
 * TODO: Treba se napraviti odrediti parametri raspodele grid-a
 * --Refrence materijal je drzao nekih 7000 node-ova i 225 polja za 2m*2m
 *
 */

/**
 *  
 * IMPORTANT: Rs = sqrt(Povrs*MaxNodesInArea/(pi*maxNodesc))
 * 
 */
#define AREAX (3000-500)
#define AREAY (2000-500)
#define CLOSEDIST 110
#define GRID_X 15
#define GRID_Y 15
#define MAXOBSTC 50
#define MAXNODES 7000
#define MAXNODESINAREA 7
#define TICKTIME 1000
#define CLOSE 100000
#define GOALDELTA2 2500
#define ROBOTRADIUS 200
#define GOALPERCENT 0.1
#define ELIPSEPERCENT 0.5
#define MAXDTIME 0.1

struct cPoint /* Coordinates */
{ 
    float x;
    float y;
    float radius;
    cPoint operator-(cPoint b)
    {
        cPoint n;
        n.x=x-b.x;
        n.y=y-b.y;
        return n;
    }
    cPoint operator+(cPoint b)
    {
        cPoint n;
        n.x=x+b.x;
        n.y=y+b.y;
        return n;
    }
    cPoint operator*(float b)
    {
        cPoint n;
        n.x=x*b;
        n.y=y*b;
        return n;
    }
    cPoint operator/(float b)
    {
        cPoint n;
        n.x=x/b;
        n.y=y/b;
        return n;
    }
    cPoint operator*(int b)
    {
        cPoint n;
        n.x=x*b;
        n.y=y*b;
        return n;
    }

};

typedef list<cPoint> qcPoint;
typedef list<int>    li;
typedef unordered_set<int>     si;

static int           addNode(cPoint xRand, int xClose, li &near);
static void          blockNodes(cPoint Obst);
static bool          checkPoint(cPoint x);
static void          c_error(const char e);
static float         cost(int Node);
static int           depth(int Node);
static float         dot(cPoint x, cPoint y);
inline static float  dist2(cPoint x, cPoint y);
static float         dist(cPoint x, cPoint y);
static int           dTime(); /* Returns time from last call */
static void          expandAndRewireTree();
static void          findNear(cPoint node, list<list<int> *> &subset, li &near);
static void          getGridLoc(cPoint x, int &i, int&j);
static float         getH(int Node);
static void          getSubset(cPoint node, list<list<int> *> &subset);
static void          getSubset(cPoint node, list<list<int> *> &subset, int radius);
static int           getTime();
static void          initVariables();
static void          pingTargetStateDbus(cPoint previous, cPoint next);
static void          planPath();
static float         randFloat(float low, float high);
static int           randInt(int low, int high);
static void          readStateFromDbus();
static void          realTimeRRT(); /* Starts the rt-rrt* loop */
static void          rewireRandomNode();
static void          rewireFromRoot();
static cPoint        sampleElipse();
static cPoint        sampleLine();
static cPoint        sampleNewNode();
static cPoint        samplePoint();
static char          testLine(cPoint x, cPoint y);
static void          tpRoot(int Node);
static void          updatePP();

static const float     BlockSizeX = AREAX/GRID_X;
static const float     BlockSizeY = AREAY/GRID_Y;

static si              AdjecenyMatric[MAXNODES];
static si              Blocked;
static int             ClosestToGoal;
static float           DistanceFromRoot[MAXNODES]; /* Resets Every Tick/Zeros every 1000000000 */
static int             DistanceLastUpdated[MAXNODES];
static bool            EXIT;
static cPoint          FakeObstacles[MAXOBSTC]; /* Used for movement counteraction */
static bool            GoalBlocked;
static int             GoalPath[MAXNODES];
static int             GoalPathLength;
static int             GoalPathPrime[MAXNODES];
static bool            GoalReached;
static cPoint          Goal;
static li              Grid[GRID_Y][GRID_X];
static int             MaxdTime;
static cPoint          Nodes[MAXNODES];
static int             NodesC;
static cPoint          Obstacles[MAXOBSTC]; /* An array of obstacles */
static int             ObstaclesC;
static int             Parent[MAXNODES];
static li              Path;
static int             PreviousTick;
static cPoint          Robot;
static int             Root;
static li              Qr;
static li              Qs;
static clock_t         TickStart;
static int             TicksSinceLastReset;
static int             TpPath[MAXNODES];
static si              WhasInQs;
static si              Visited;

static unsigned long long tick=0;

static random_device rand_dev;
static mt19937 generator(rand_dev());
static uniform_real_distribution<float> distribution(0,100);

static PyObject *InfoMeth;
static PyObject *PpMeth;
static PyObject *ErrMeth;

static int
addNode(cPoint xRand, int xClose, li &near)
{
    int xMin = xClose;
    float cMin = cost(xClose) + dist(xRand,Nodes[xClose]);
    for(const auto &x: near)
    {
        int cNew=cost(x) + dist(xRand,Nodes[x]);
        if (cNew < cMin && testLine(Nodes[x], xRand))
        {
            
            cMin = cNew;
            xMin = x;
        }
    }
    int GridI,GridJ;
    if((dist2(Goal,xRand)<GOALDELTA2 && !GoalReached) || (GoalReached && dist2(Goal,xRand) < dist2(Nodes[ClosestToGoal],Goal)))
    {
        GoalReached=true;
        ClosestToGoal = NodesC;
    }
    getGridLoc(xRand, GridI, GridJ);
    Grid[GridI][GridJ].push_back(NodesC);
    Nodes[NodesC] = xRand;
    Parent[NodesC] = xMin;
    AdjecenyMatric[xMin].insert(NodesC);
    AdjecenyMatric[NodesC++].insert(xMin);
    return NodesC-1;
}

static void
blockNodes(cPoint obst)
{
    list<list<int> *> pointNearArea;
    getSubset(obst,pointNearArea, ceil(obst.radius/(AREAX/GRID_X)));
    for(const auto &q : pointNearArea)
    {
        for(const auto &x: *q)
        {
            if(obst.radius+ROBOTRADIUS > dist(Nodes[x],obst))
            {
                Blocked.insert(x);
            }
        }
    }
}

static void
c_error(const char* s)
{

        PyObject *str = PyUnicode_FromString(s);
        PyObject *args = Py_BuildValue("(O)",str);
        PyObject_CallObject(ErrMeth, args);
        PyErr_Print();
        Py_DECREF(str);
        Py_DECREF(args);
        PyErr_Print();
}

static bool
checkPoint(cPoint x)
{
    if(x.x>=AREAX || x.y>=AREAY || x.x<=0 || x.y<=0)
    {
        return false;
    }

    if(isnan(x.x)  || isnan(x.y))
    {
        return false;
    }

    for(int i=0; i<ObstaclesC;i++)
    {
        if(dist2(x,Obstacles[i])<=(Obstacles[i].radius+ROBOTRADIUS)*(Obstacles[i].radius+ROBOTRADIUS))
        {
            return false;
        }
    }
    
    return true;
}

static float
cost(int node)
{


    float c=0;
    int rec = node;
    while(Root != rec){
        if(Blocked.count(rec))
        {
            return INFINITY;
        }
        c += dist(Nodes[rec],Nodes[Parent[rec]]);
        rec = Parent[rec];
    }
    return c;
}

static int
depth(int Node)
{
    int x=0;
    int c = Node;
    while (c!=Root)
    {
        if(Blocked.count(c)>0)
        {
            return -1;
        }
        c = Parent[c];
        x++;
    }
    return x;
}

static float
dot(cPoint x, cPoint y){
    return x.x * y.x + x.y * y.y;
}

static float 
dist2(cPoint x, cPoint y)
{
    return (x.x - y.x) * (x.x - y.x)
           + (x.y - y.y) * (x.y - y.y);
}

static float 
dist(cPoint x, cPoint y)
{
    return sqrt(dist2(x, y));
}


static void
expandAndRewireTree()
{
    cPoint xRand;
    
    /* SAMPLE */
    xRand = sampleNewNode();
    if(!checkPoint(xRand))
    {
        return;
    }
    /* Push nearest queues */
    list<list<int> *> pointNearArea;
    getSubset(xRand,pointNearArea);
    /* Find closest */
    int xClose=-1;
    float Closest = -1.0f;
    for(const auto &q : pointNearArea)
    {
        for(const auto &x: *q)
        {
            if(Closest == -1.0f || Closest > dist2(Nodes[x],xRand))
            {
                Closest = dist2(Nodes[x],xRand);
                xClose = x;
            }
        }
    }
    if(xClose < 0)
    {
        return;
    }
    xRand = Nodes[xClose]+(xRand-Nodes[xClose])/dist(xRand,Nodes[xClose])*CLOSEDIST;
    if(xRand.x<0 || xRand.y<0)
    {
        return;
    }
    if(testLine(xRand, Nodes[xClose]))
    {
        li near;
        findNear(xRand, pointNearArea, near);
        if(NodesC<MAXNODES && (near.size() < MAXNODESINAREA || dist2(Nodes[xClose], xRand) > CLOSEDIST * CLOSEDIST ))
        {
            int xRandI = addNode(xRand, xClose, near);
            Qr.push_front(xRandI);
        } else
        {
           //printf("Rejected: {%f,%f}\n",xRand.x,xRand.y);
            Qr.push_front(xClose);
        }
        rewireRandomNode();
    }
    rewireFromRoot();
    
}

static void
findNear(cPoint node, list<list<int> *> &subset, li &near)
{

    for(const auto &q : subset)
    {
        for(const auto &x: *q)
        {
            if(dist2(Nodes[x],node)<=CLOSEDIST*CLOSEDIST)
            {
                near.push_back(x);
            }
        }
    }
    
}


static float
getH(int Node)
{
    if(Visited.count(Node))
        return INFINITY;
    return dist(Nodes[Node],Goal);
}

static void
getSubset(cPoint Node, list<list<int> *> &subset)
{
    int gridI, gridJ;
    getGridLoc(Node,gridJ,gridI);
    
    int searchRadius = 1;
    while (subset.empty() && searchRadius < GRID_X)
    {
        for(int i=0; i< searchRadius*2; i++)
        {
            int x1,x2,x3,x4;
            int y1,y2,y3,y4;
            x1 = gridJ - searchRadius + i;
            y1 = gridI + searchRadius;

            x2 = gridJ + searchRadius;
            y2 = gridI + searchRadius - i;

            x3 = gridJ + searchRadius - i;
            y3 = gridI - searchRadius;

            x4 = gridJ - searchRadius;
            y4 = gridI - searchRadius + i;

            if(x1 >= 0 && GRID_X > x1  && y1 >= 0 && GRID_Y > y1 && !Grid[x1][y1].empty())
                subset.push_back(&Grid[x1][y1]);
            if(x2 >= 0 && GRID_X > x2  && y2 >= 0 && GRID_Y > y2 && !Grid[x2][y2].empty())
                subset.push_back(&Grid[x2][y2]);
            if(x3 >= 0 && GRID_X > x3  && y3 >= 0 && GRID_Y > y3 && !Grid[x3][y3].empty())
                subset.push_back(&Grid[x3][y3]);
            if(x4 >= 0 && GRID_X > x4  && y4 >= 0 && GRID_Y > y4 && !Grid[x4][y4].empty())
                subset.push_back(&Grid[x4][y4]);
        }
        searchRadius++;
    }
    if(!Grid[gridI][gridJ].empty())
    {
        subset.push_back(&Grid[gridI][gridJ]);
    }
}

static void
getSubset(cPoint Node, list<list<int> *> &subset, int radius)
{
    int gridI, gridJ;
    getGridLoc(Node,gridJ,gridI);
    
    for(int searchRadius = 1; searchRadius < radius && searchRadius < GRID_X; searchRadius++)
    {
        for(int i=0; i< searchRadius*2; i++)
        {
            int x1,x2,x3,x4;
            int y1,y2,y3,y4;
            x1 = gridJ - searchRadius + i;
            y1 = gridI + searchRadius;

            x2 = gridJ + searchRadius;
            y2 = gridI + searchRadius - i;

            x3 = gridJ + searchRadius - i;
            y3 = gridI - searchRadius;

            x4 = gridJ - searchRadius;
            y4 = gridI - searchRadius + i;


            if(x1 >= 0 && GRID_X > x1  && y1 >= 0 && GRID_Y > y1 && !Grid[x1][y1].empty())
                subset.push_back(&Grid[x1][y1]);
            if(x2 >= 0 && GRID_X > x2  && y2 >= 0 && GRID_Y > y2 && !Grid[x2][y2].empty())
                subset.push_back(&Grid[x2][y2]);
            if(x3 >= 0 && GRID_X > x3  && y3 >= 0 && GRID_Y > y3 && !Grid[x3][y3].empty())
                subset.push_back(&Grid[x3][y3]);
            if(x4 >= 0 && GRID_X > x4  && y4 >= 0 && GRID_Y > y4 && !Grid[x4][y4].empty())
                subset.push_back(&Grid[x4][y4]);
        }
        searchRadius++;
    }
    if(!Grid[gridI][gridJ].empty())
    {
        subset.push_back(&Grid[gridI][gridJ]);
    }
}
static void
getGridLoc(cPoint x, int &i, int&j)
{
    
    i = x.x/((AREAX+10)/GRID_X);
    j = x.y/((AREAY+10)/GRID_Y);
    i = min(i,GRID_X-1);
    j = min(i,GRID_Y-1);
}
static void
initVariables()
{
    srand(time(NULL));

    EXIT       = false;
    ObstaclesC = 0;
    NodesC = 0;
    memset(DistanceLastUpdated,0,MAXNODES*sizeof(int));
    TicksSinceLastReset = -1;
    readStateFromDbus();
    int i,j;
    getGridLoc(Robot, i, j);
    Grid[i][j].push_back(NodesC);
    Parent[NodesC] = -1;
    Root = NodesC;
    Nodes[NodesC++]=Robot;
    GoalReached = false;
    GoalBlocked=false;
}
static void
planPath()
{
    if(GoalReached)
    {
        int d = depth(ClosestToGoal);
        if(d>=0)
        {
            GoalBlocked=false;
            GoalPathLength = d+1;
            int c = ClosestToGoal;
            for(int i=d;i>=0;i--)
            {
                GoalPath[i] = c;
                c=Parent[c];
            }
            return;
        } else
        {
            GoalBlocked=true;
        }
    }
    
    int GoalPathPrimeLength = 1;
    GoalPathPrime[0]=Root;
    int TempNode = Root;
    while(AdjecenyMatric[TempNode].size()>1 || (TempNode==Root && AdjecenyMatric[TempNode].size()>0))
    {
        int Old = TempNode;
        si &Cons = AdjecenyMatric[TempNode];
        TempNode = -1;
        float MinCost = INFINITY;
        float actCost = INFINITY;
        for(const auto& x : Cons)
        {
            if(x==Parent[Old]) continue;
            if(TempNode<0)
            {
                TempNode = x;
                actCost = cost(x);
                MinCost = actCost+getH(x);
                continue;
            }

            float actCost_ = cost(x);
            float Cost_ = actCost_+getH(x);
            if(Cost_ < MinCost)
            {
                TempNode = x;
                MinCost = Cost_;
                actCost = actCost_;
            }
            
        }
        GoalPathPrime[GoalPathPrimeLength++] = TempNode;

        if((actCost==INFINITY && AdjecenyMatric[TempNode].size()>1 )|| (TempNode==Root && AdjecenyMatric[TempNode].size()>0))
        {
            Visited.insert(GoalPathPrime[GoalPathPrimeLength-1]);
            break;
        }
    }


    if(GoalPathLength==0)
    {
        GoalPath[GoalPathLength++]=Root;
    }
    if((Blocked.count(Root) <= 0 && cost(GoalPath[0]) == INFINITY) || dist2(Nodes[GoalPathPrime[GoalPathPrimeLength-1]],Goal) <= dist2(Nodes[GoalPath[GoalPathLength-1]],Goal))
    {
        swap(GoalPath,GoalPathPrime);
        swap(GoalPathLength,GoalPathPrimeLength);
    }
}

static float
randFloat(float l, float r)
{
    float x = distribution(generator);
    return (r-l)*x/100 + l;
}

static int
randInt(int l, int r)
{
    float x = distribution(generator);
    return (r-l)*x/100 + l;
}

static void
readStateFromDbus()
{
    PyObject *res=PyObject_CallObject(InfoMeth,NULL);
    PyObject *obst;
    cPoint _Robot;
    cPoint _Goal;
    int _ObstaclesC;
    if(!PyArg_ParseTuple(res, "ffffOi", &_Robot.x,&_Robot.y,&_Goal.x,&_Goal.y,&obst, &_ObstaclesC))
    {
        PyErr_Print();
        exit(-1);
    }
    Py_INCREF(obst);
    ObstaclesC=_ObstaclesC;
    Blocked.clear();
    for(int i=0;i<ObstaclesC;i++){
        PyObject *o;
        o = PyList_GetItem(obst,i);
        Py_INCREF(o);
        if(!PyArg_ParseTuple(o,"fff",&Obstacles[i].x,&Obstacles[i].y,&Obstacles[i].radius))
        {
            PyErr_Print();
            exit(1);
        }
        list<list<int> *> subset;
        blockNodes(Obstacles[i]);
        Py_DECREF(o);
        PyErr_Print();
    }
    if(_Goal.x == -1,_Goal.y == -1)
    {
        list<list<int> *> subset;
        getSubset(Robot,subset);
        li field;
        int xClose=-1;
        float Closest = -1.0f;
        for(const auto &q : subset)
        {
            for(const auto &x: *q)
            {
                if(Closest == -1.0f || Closest > dist2(Nodes[x],Robot) && !Blocked.count(x))
                {
                    Closest = dist2(Nodes[x],Robot);
                    xClose = x;
                }
            }
        }
        if(xClose<0)
        {
            c_error("[PANIC]: change root");
        }
        else
        {
            tpRoot(xClose);
        }
 
    } else if(Goal.x!=_Goal.x && Goal.y!=_Goal.y)
    {
        GoalReached=false;
        list<list<int> *> pointNearArea;
        getSubset(_Goal,pointNearArea);
        /* Find closest */
        int xClose=-1;
        float Closest = -1.0f;
        for(const auto &q : pointNearArea)
        {
            for(const auto &x: *q)
            {
                if(Closest == -1.0f || Closest > dist2(Nodes[x],_Goal))
                {
                    Closest = dist2(Nodes[x],_Goal);
                    xClose = x;
                }
            }
        }
        if(xClose >= 0)
        {
            GoalReached=true;
            ClosestToGoal=xClose;
        }
    }
    Robot=_Robot;
    Goal=_Goal;
    Py_DECREF(obst);
    PyErr_Print();
    return;
}

static void
realTimeRRT()
{
    while(!EXIT)
    {
        
        readStateFromDbus();
        Py_BEGIN_ALLOW_THREADS
        TickStart=clock();
        
        while (((double)(clock()-TickStart))/CLOCKS_PER_SEC < MAXDTIME)
        {
            expandAndRewireTree();
        }
        
        planPath();


        
        int k=1;
        while(GoalPathLength>k+1 && CLOSE>dist2(Robot,Nodes[GoalPath[k]]))
        {
            
            Parent[Root]=GoalPath[k-1];
            Root = GoalPath[k-1];

            Visited.clear();
            Qr.clear();
            Qs.clear();
            k++;
        }
        Py_END_ALLOW_THREADS
        updatePP();

    }
}

static void
rewireFromRoot()
{
    if(Qs.empty())
    {
        Qs.push_back(Root);
        WhasInQs.clear();
    }

    while(((double)(clock()-TickStart))/CLOCKS_PER_SEC < MAXDTIME  && !Qs.empty())
    {
        int xS = Qs.front();
        Qs.pop_front();
        li near;
        list<list<int> *> subset;
        getSubset(Nodes[xS], subset);
        findNear(Nodes[xS], subset, near);

        for(const auto &x : near){
            float cOld = cost(x);
            float cNew = cost(xS) + dist(Nodes[xS],Nodes[x]);
            if(cNew < cOld && testLine(Nodes[xS],Nodes[x]))
            {
                AdjecenyMatric[Parent[x]].erase(x);
                AdjecenyMatric[x].erase(Parent[x]);
                Parent[x] = xS;
                AdjecenyMatric[xS].insert(x);
                AdjecenyMatric[x].insert(xS);
            }
            if(WhasInQs.count(x) <= 0)
            {
                Qs.push_back(x);
                WhasInQs.insert(x);
            }
        }
    }
    //if(Qs.empty())
        //printf("Done rand\n");
    
}

static void
rewireRandomNode()
{
    while(((double)(clock()-TickStart))/CLOCKS_PER_SEC < MAXDTIME*0.5  && !Qr.empty())
    {
        int xR = Qr.front();
        Qr.pop_front();
        li near;
        list<list<int> *> subset;
        getSubset(Nodes[xR], subset);
        findNear(Nodes[xR], subset, near);
        int lf = 0;
        for(const auto &x : near){
            if(Parent[xR]==x) continue;
            float cOld = cost(x);
            float cNew = cost(xR) + dist(Nodes[xR],Nodes[x]);
            if(cNew < cOld && testLine(Nodes[xR],Nodes[x]))
            {
                AdjecenyMatric[Parent[x]].erase(x);
                AdjecenyMatric[x].erase(Parent[x]);
                Parent[x] = xR;
                AdjecenyMatric[xR].insert(x);
                AdjecenyMatric[x].insert(xR);
                Qr.push_back(x);
            }
        }
    }
    
}
static cPoint
sampleElipse()
{

    cPoint F1 = {Nodes[Root].x,Nodes[Root].y,Nodes[Root].radius};
    cPoint F2 = {Goal.x,Goal.y,Goal.radius};
    cPoint C = (F1+F2)*0.5f;
    float cMin=cost(GoalPath[GoalPathLength-1]);
    float cBest=dist(F1,F2);
    float a = cBest/2;
    float b = sqrt(cBest*cBest-cMin*cMin)/2;
    cPoint dir = F2-F1;
    dir = dir/dist({0.0f,0.0f,0.0f},dir);

    float rangle = randFloat(-M_PI , M_PI );

    float r = a*b/(pow(b*cos(rangle),2)+pow(a*sin(rangle),2));
    cPoint x = {r*cos(rangle),r*sin(rangle),0.0f};
    cPoint s = {x.x*dir.x-x.y*dir.y,x.x*dir.y+x.y*dir.x,0.0f};
    s = s*randFloat(0,1);
    s = s+C;
    return s;
    
}

static cPoint
sampleLine()
{
    float R = randFloat(0,1);
    return Goal+(Goal - Nodes[Root])*R;
}

static cPoint 
sampleNewNode()
{
    float x = randFloat(0,1);
    if(x<GOALPERCENT)
    {
        if(GoalReached&& x/GOALPERCENT<ELIPSEPERCENT)
        {
            return sampleElipse();
        }
        return sampleLine();
    }
    return samplePoint();
    return {-1.0f,-1.0f,0.0f};
}

static cPoint
samplePoint(){
    cPoint x;
    x.x = randFloat(0,AREAX);
    x.y = randFloat(0,AREAY);
    x.radius=-1;
    return x;
}

static char
testLine(cPoint x, cPoint y)
{
    for(int i=0;i<ObstaclesC;i++)
    {

        float h = min(1.0f,max(0.0f,dot(Obstacles[i]-x,y-x)/dot(y-x,y-x)));

        if(dist2(Obstacles[i]-x,(y-x)*h)<(Obstacles[i].radius+ROBOTRADIUS)*(Obstacles[i].radius+ROBOTRADIUS))
        {
            return false;
        }
    }
    return true;
}

static void
tpRoot(int Node)
{
    int x=0;
    int c = Node;
    while (c!=Root)
    {
        TpPath[x]=c;
        c = Parent[c];
        x++;
    }
    TpPath[x++]=Root;
    for(int i = x;i>0;i--)
    {
        int a = TpPath[x];
        int b = TpPath[x-1];
        Parent[b]=a;
        Root = b;

        Visited.clear();
    }
    Qr.clear();
    Qs.clear();
}
static void
updatePP()
{
    if(Blocked.count(Root))
    {
        char* s= "Root is blocked";
        PyObject *str = PyUnicode_FromString(s);
        PyObject *args = Py_BuildValue("(O)",str);
        PyObject_CallObject(ErrMeth, args);
        PyErr_Print();
        Py_DECREF(str);
        Py_DECREF(args);
        PyErr_Print();
    }
    PyObject *pl = PyList_New(0);
    for(int i=0; i< GoalPathLength;i++)
    {
        PyObject *x = Py_BuildValue("ff",Nodes[GoalPath[i]].x,Nodes[GoalPath[i]].y);
        PyList_Append(pl, x);
        Py_DECREF(x);
    }
    if(GoalReached && !GoalBlocked)
    {
        PyObject *x = Py_BuildValue("ff",Goal.x,Goal.y);
        PyList_Append(pl, x);
        Py_DECREF(x);
    }
    PyObject *pr = PyList_New(0);
    for(int i=0; i< NodesC;i++)
    {
        PyObject *x = Py_BuildValue("ffi",Nodes[i].x,Nodes[i].y,Parent[i]);
        PyList_Append(pr, x);
        Py_DECREF(x);
    }
    PyObject *args = Py_BuildValue("OO",pl,pr);
    PyObject_Call(PpMeth,args,NULL);
    Py_DECREF(args);
    Py_DECREF(pl);
    Py_DECREF(pr);
    //PyErr_Print();
    return;
}

/*
int
main()
{
    initVariables();
    realTimeRRT();
    return 0; 
}
*/

static PyObject *
_startRRT(PyObject* self, PyObject* args)
{
    if(!PyArg_ParseTuple(args, "OOO", &InfoMeth, &PpMeth, &ErrMeth))
    {
        return NULL;
    }
    
    initVariables();
    realTimeRRT();
    
    return NULL;
}

static struct PyMethodDef methods[]
{
    {"startRRT", (PyCFunction)_startRRT, METH_VARARGS},
    {NULL,NULL, 0 ,NULL}
};

static struct PyModuleDef module =
{
    PyModuleDef_HEAD_INIT,
    "_rtrrt",
    NULL,
    -1,
    methods
};

PyMODINIT_FUNC PyInit__rtrrt(void) {
    return PyModule_Create(&module);
}