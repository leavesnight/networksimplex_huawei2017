#ifndef __SOLVER_H__
#define __SOLVER_H__

#include "deploy_solver_heap.h"
#include "lib_io.h"
#include <vector>
#include <queue>
#include <sstream>
#include <string>
using namespace std;

#include <ctime>

extern clock_t g_tmStart;
extern int g_count,g_count2;

class ServerSelectionSolver{
public:
	ServerSelectionSolver(char* topo[MAX_EDGE_NUM],int line_num,int fast_mode=0);
	~ServerSelectionSolver();
	bool isapNA();
	void startSolver(int timeS=88);
	const char* getResult();
	void xjbs();
	int getRawCost(vector<unsigned char>& pos);
private:
	struct NodePoint{
		int x;
		int vid;
	};
	vector<NodePoint> vecRef;

	int g_numVert,g_numDem,g_m,g_totalDem;
	struct NodeServer{
		int id;
		int cap;
		int cost;
	};
	vector<NodeServer> g_pServFast;
	vector<int> m_pos_serv;//every server type for pos i
	int m_numSSFast,m_numSS;//super source
	int* m_pnumSS;
	int m_fast_mode,m_maxCS;
	int* g_pExtraCost;
	int g_totalCost,g_leftCus,m_minCost;
	string strTopo;
	vector<unsigned char> m_bestServerPos;
	vector<unsigned char> m_fixedServerPos;

	struct NodeEdge{
		int u;//upper bound, uij=[0,100];lower bound is lij=0
		int c;//cost, cij=[0,100]
		int x;//flow, xij
		//int cpi;//Cπij, 4 answers from pi[2]&pj[2] but we can just use 2 types
		//[1]:cij-πi0+πj0 for LorU && [0]:cij-πi0+πjk(<0 then in) for L;[0]:cij-πik+πj0(>0 then in) for U
		//though [0] is smaller for L or bigger for U, it may cause a contradiction that x0j/x0i for L/U doesn't change from >0 to 0
		int idBegin;
		int idEnd;
	};
	struct NodeVertex{//can store the NodeTree data
		int d;//demand, di=[0,5000]
		int id;//customer's id, πid
		//NodeTree
		int parent;bool directToParent;//true means the arc is pointing to the parent else it's pointing from the parent
		int depth;
		int next;//deep first search
		int prec;//anti deep first search
		int pi;//node potential//two types for the step function from the ultimate source, πi[2]
		vector<int> idEdge;
		vector<int> idEdgeFrom;
	};
	int m_root;
	int** g;//0 is the super point/ultimate source&& just save the edge id
	NodeEdge* g_edge;//max 20 edges connecting to one point, and add 0i;start from 0 is easy to find the opposite edge
	int g_edgeCount,g_edgeCoreCount;
	NodeEdge* g_edgeTmp;
	NodeVertex* v;
	int* m_maxout;
	double* m_lagn;
	bool m_copy;
	int* vCons;//record the id in the network,+1
	int** g_srcEdge;//,g_snkEdge;
	struct NodeEdgeUnD{
		int cpi;//modified Cπij,if x==0 it will be -cπij
		int idBegin;
		int idEnd;
		int idHeap;
	};
	NodeEdgeUnD* g_edgeUnD;//use this to boost up the searching cpi speed
	int g_edgeUnDCount,g_edgeCoreUnDCount;
	//ZkwHeap<int,less<int>> g_zkwtree(0);//use big heap
	vector<NodeHeap<int>> g_heap;//use big heap
	const bool g_bUseHeap;

	void netSAinit(NodeEdge edge[]);
	int networkSimplexAlg(vector<unsigned char>& pos,NodeEdge edge[]);
	inline void changeEdgeDUnD(int i,NodeEdge g_edge[]);
	inline void changeEdgeUnD(int i,NodeEdge g_edge[]);
	inline void zkwtreeModify(int i);
	inline void findOutgoing(int& k,int& l,bool& outPTop,bool bResult,
					  int& min,int p,int q,bool bEqual=false,bool directPlus=false);

	void printPath(int i,int& flow,ostream& sout,int kserv);
	void printTree();

	int m_judgesum,m_unfixednum;
	vector<int> m_judgePos;
	void makejude(vector<int>& m_judgePos);
	void processNetwork(vector<unsigned char>& pos,int& totalCost,NodeEdge edge[],bool bMakeRoute=false,int bBug=2);
	void deleteFromNet(int j,int& flow,vector<unsigned char>& pos,NodeEdge edge[]);
	void deleteAndCalcFromNet(int j,int& flow,vector<unsigned char>& pos,int& flowCost,int& srcID,NodeEdge edge[]);
	bool calcCost(int& totalCost,NodeEdge edge[],bool bCheck=true);
	void branchbound(int timeS);
	int calBBbound(vector<unsigned char> pos,int& posBranch);
	void startSA(int timeS);
	void updateSAInfo(int& server_num,vector<int>& server_level,vector<unsigned char>& pos);
	int judgefunc(vector<unsigned char>& pos);
	//bool startXjbs(int nMinCost,set<int> nMinPos,set<int> nGreedyServerPos);
	friend struct Flow;
	friend class GenAlg;
	void xjb_search(int timeS);
	friend struct xjb_dude;
	friend struct xjb_set;
};
struct Flow{
	ServerSelectionSolver* psssolver;
	int cost;
	vector<unsigned char> servers;
	int val(){
		return cost;
	}
	Flow(ServerSelectionSolver* _psss):cost(~0u>>1),psssolver(_psss){}
	Flow(vector<unsigned char>& svrs,ServerSelectionSolver* _psss):cost(0),servers(svrs),psssolver(_psss){
		vector<unsigned char> posTmp(svrs);
		int nTmTmp=clock();
		int& nTotalCost=cost;
		nTotalCost=psssolver->networkSimplexAlg(posTmp,psssolver->g_edge);
		int type=3;//servers.count();
		if (type==1){
			for (int i=0;i<psssolver->g_edgeCoreCount;++i){
				psssolver->g_edgeTmp[i].x=psssolver->g_edge[i].x;
			}
			psssolver->processNetwork(posTmp,nTotalCost,psssolver->g_edgeTmp,true,1);
			g_count+=clock()-nTmTmp;
			psssolver->calcCost(nTotalCost,psssolver->g_edgeTmp);
			servers=posTmp;
		}else{
			//cost=psssolver->getRawCost(posTmp);
			//psssolver->processNetwork(servers,cost,psssolver->g_edge,true,0);
			psssolver->calcCost(cost,psssolver->g_edge);
		}
	}
};
struct dude{
	Flow* flow;
	bool operator<(const dude& d)const{
		return flow->val()<d.flow->val();
	}
	bool operator==(const dude& d)const{
		return flow->servers==d.flow->servers;
	}
};
Flow& randomwalk();
template<class T> struct ISAP{
	struct edge{
		int v;
		T c,l;
		edge(int _v,T _c):v(_v),c(_c),l(_c){}
	};
	ISAP(int _n,int _src,int _snk):bge(_n),hei(_n,_n),gap(_n+1),n(_n),cur(_n),frm(_n),
		src(_src),snk(_snk){}
	void lab(){//bfs && init gap
		hei[snk]=0;
		queue<int> qu;
		qu.push(snk);
		for (int u;qu.empty()?0:(u=qu.front(),qu.pop(),1);)
			for (int i=0;i<bge[u].size();++i){
				edge& e=egs[bge[u][i]],& ev=egs[bge[u][i]^1];
				if (ev.c>0&&hei[e.v]==n)
					hei[e.v]=hei[u]+1,qu.push(e.v);
			}
		for (int i=0;i<n;++i)
			++gap[hei[i]];
	}
	T aug(){
		T f=0;
		for (int u=snk;u!=src;u=egs[frm[u]^1].v)
			if (f<=0||f>egs[frm[u]].c)
				f=egs[frm[u]].c;
		for (int u=snk;u!=src;u=egs[frm[u]^1].v)
			egs[frm[u]].c-=f,egs[frm[u]^1].c+=f;
		return f;
	}
	void add(int u,int v,T c){
		bge[u].push_back(egs.size());
		egs.push_back(edge(v,c));
		bge[v].push_back(egs.size());
		egs.push_back(edge(u,0));
	}
	T run(){
		lab();
		T r=0;
		for (int u=src;hei[src]!=n;){
			if (u==snk)
				r+=aug(),u=src;
			int f=0;
			for (int i=cur[u];i<bge[u].size();++i){
				edge& e=egs[bge[u][i]];
				if (e.c>0&&hei[u]==hei[e.v]+1){
					f=1;
					frm[e.v]=bge[u][i];
					u=e.v;
					break;
				}
			}
			if (!f){
				int mh=n-1;
				for (int i=0;i<bge[u].size();++i){
					edge& e=egs[bge[u][i]];
					if (e.c>0&&mh>hei[e.v])
						mh=hei[e.v];
				}
				if (!--gap[hei[u]])
					break;
				++gap[hei[u]=mh+1];
				cur[u]=0;
				if (u!=src)
					u=egs[frm[u]^1].v;
			}
		}
		return r;
	}
	int n,src,snk;
	vector<edge> egs;
	vector<vector<int>> bge;
	vector<int> hei,gap,cur,frm;
};

#endif