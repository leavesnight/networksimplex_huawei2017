#include "deploy.h"
#include <stdio.h>
#include <sstream>
#include <string>
#include <limits.h>
#include <ctime>
#include <map>
#include <algorithm>
#include <random>
#include <functional>
#include <bitset>
//#include <iterator>
using namespace std;

const int MAX_NODE_NUM=1000,MAX_LINES_NUM=50000,MAX_CONSUME_NUM=500;
int g_numVert,g_numDem,g_costServ,g_totalDem;
int g_count,g_leftCus,g_totalCost,g_count2;
clock_t g_tmStart=clock();

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
struct NodeConsumer{
	int vid;//record the id in the network,+1
};
int g[MAX_NODE_NUM+1][MAX_NODE_NUM+1];//0 is the super point/ultimate source&& just save the edge id
NodeEdge g_edge[MAX_NODE_NUM*(40+1)];//max 20 edges connecting to one point, and add 0i;start from 0 is easy to find the opposite edge
int g_edgeCount=0;
NodeEdge g_edgeTmp[MAX_NODE_NUM*(40+1)];
NodeVertex v[MAX_NODE_NUM+1];
NodeConsumer vCons[MAX_CONSUME_NUM+1];
set<int> sFixedServerPos;
int g_srcEdge[MAX_NODE_NUM+1],g_snkEdge[MAX_NODE_NUM+1];
struct NodeEdgeUnD{
	int cpi;//modified Cπij,if x==0 it will be -cπij
	int idBegin;
	int idEnd;
	int idHeap;
};
NodeEdgeUnD g_edgeUnD[MAX_NODE_NUM*(20+1)];//use this to boost up the searching cpi speed
int g_edgeUnDCount=0;
//ZkwHeap<int,less<int>> g_zkwtree(0);//use big heap
vector<NodeHeap<int>> g_heap;//use big heap
const bool g_bUseHeap=0;

void netSAinit(NodeEdge edge[]);
void networkSimplexAlg(set<int>& pos,NodeEdge edge[]);
inline void changeEdgeUnD(int i,NodeEdge g_edge[]);
inline void zkwtreeModify(int i);
inline void findOutgoing(int& k,int& l,bool& outPTop,bool bResult,
				  int& min,int p,int q,bool bEqual=false,bool directPlus=false);
void processNetwork(set<int>& pos,int& totalCost,NodeEdge edge[],bool bMakeRoute=false,int bBug=2);
void deleteFromNet(int j,int& flow,set<int>& pos,NodeEdge edge[]);
void deleteAndCalcFromNet(int j,int& flow,set<int>& pos,int& flowCost,int& srcID,NodeEdge edge[]);
bool calcCost(int& totalCost,int& totalNum,NodeEdge edge[],bool bTest=true);
void printPath(int i,int& flow,ostream& sout);
void printTree();
void xjbs();
bool startSA(int nMinCost,set<int> nMinPos,set<int> nGreedyServerPos);
bool startXjbs(int nMinCost,set<int> nMinPos,set<int> nGreedyServerPos);
	
struct Flow{
	//NodeVertex* pbg;
	//vector<NodeEdge> eg;
	int flow,cost;
	bitset<1010> servers;
	bitset<1010> modifiedservers;
	int val(){
		//if (flow!=g_totalDem)
		//	return ~0u>>1;
		return cost;
	}
	Flow():flow(0),cost(~0u>>1){}
	Flow(bitset<1010> svrs):flow(0),cost(0),servers(svrs){
		set<int> setTmp;
		for (int i=0;i<g_numVert;i++){
			if (servers[i])
				setTmp.insert(i+1);
		}
		int nTmTmp=clock();
		//netSAinit(g_edge);
		networkSimplexAlg(setTmp,g_edge);
        int type=3,nTotalNum=0;//servers.count();
		int& nTotalCost=cost;
        if (type==1){
			for (int i=0;i<g_edgeCount;++i){
				g_edgeTmp[i].x=g_edge[i].x;
			}
			processNetwork(setTmp,nTotalCost,g_edgeTmp,true);
			//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
			g_count+=clock()-nTmTmp;
			//nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(nTotalCost,nTotalNum,g_edgeTmp);//false
		}else if (type==0){
			processNetwork(setTmp,nTotalCost,g_edge,true,0);
			networkSimplexAlg(setTmp,g_edge);
			g_count+=clock()-nTmTmp;
			nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(nTotalCost,nTotalNum,g_edge,false);
		}else if (type==2){
			for (int i=0;i<g_edgeCount;++i){
				g_edgeTmp[i].x=g_edge[i].x;
			}
			processNetwork(setTmp,nTotalCost,g_edgeTmp,true,1);
			//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
			g_count+=clock()-nTmTmp;
			//nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(nTotalCost,nTotalNum,g_edgeTmp,false);
		}else{
			for (int k=1;k<g_numDem+1;k++){
				int i=vCons[k].vid;
				if (!servers[i-1]){//v[i].d>0
					int iMap=g_srcEdge[i];
					if (g_edge[iMap].x>0){
						nTotalCost=~0u>>1;
						break;
					}
				}
			}
			if (nTotalCost!=~0u>>1){
				//calcCost(nTotalCost,nTotalNum,g_edge,false);
				nTotalCost+=servers.count()*g_costServ;
				for (int i=0;i<g_edgeCount-g_numVert;++i){//0 is empty;*2-1
					if (g_edge[i].x>0){
						nTotalCost+=g_edge[i].x*g_edge[i].c;
					}
				}
			}
		}
		for (int i=1;i<g_numVert+1;i++){
			if (setTmp.find(i)!=setTmp.end()){
				modifiedservers[i-1]=1;
			}
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
Flow xjb_search();

//你要完成的功能总入口
void deploy_server(char * topo[MAX_EDGE_NUM], int line_num,char * filename)
{

	// 需要输出的内容
	/*char * topo_file = (char *)"17\n\n0 8 0 20\n21 8 0 20\n9 "
		"11 1 13\n21 22 2 20\n23 22 2 8\n1 3 3 11\n24 3 3 17\n27 "
		"3 3 26\n24 3 3 10\n18 17 4 11\n1 19 5 26\n1 16 6 15\n15 "
		"13 7 13\n4 5 8 18\n2 25 9 15\n0 7 10 10\n23 24 11 23";*/
	const char * topo_file;
	string strTopo="";
	int m,nTmp,nTmp2;
	string strTmp(topo[0]);
	nTmp=strTmp.find(' ');
	g_numVert=atoi(strTmp.substr(0,nTmp).c_str());//[0,1000]
	nTmp2=strTmp.find(' ',nTmp+1);
	m=atoi(strTmp.substr(nTmp+1,nTmp2-nTmp).c_str());//[0,1000*20]
	g_numDem=atoi(strTmp.substr(nTmp2+1,strTmp.length()-nTmp2-1).c_str());//[0,500]
	//cout<<g_numVert<<endl<<m<<endl<<g_numDem<<endl;
	g_costServ=atoi(string(topo[2]).c_str());//[0,5000]

	int nNum[4],nTmp3;
	for (int line=4;line<m+4;line++){
		strTmp=topo[line];
		nTmp=strTmp.find(' ');
		nNum[0]=atoi(strTmp.substr(0,nTmp).c_str());
		nTmp2=strTmp.find(' ',nTmp+1);
		nNum[1]=atoi(strTmp.substr(nTmp+1,nTmp2-nTmp).c_str());
		nTmp3=strTmp.find(' ',nTmp2+1);
		nNum[2]=atoi(strTmp.substr(nTmp2+1,nTmp3-nTmp2).c_str());
		nNum[3]=atoi(strTmp.substr(nTmp3+1,strTmp.length()-nTmp3-1).c_str());
		if (g[nNum[0]+1][nNum[1]+1]==0){
			g_edge[g_edgeCount].u=nNum[2];
			g_edge[g_edgeCount].c=nNum[3];
			g_edge[g_edgeCount].idBegin=nNum[0]+1;//it starts from point 0, but we let it from 1
			g_edge[g_edgeCount].idEnd=nNum[1]+1;
			g[nNum[0]+1][nNum[1]+1]=g_edgeCount;
			v[nNum[0]+1].idEdge.push_back(g_edgeCount);
			v[nNum[1]+1].idEdgeFrom.push_back(g_edgeCount);
			g_edgeCount++;
			g_edge[g_edgeCount].u=nNum[2];
			g_edge[g_edgeCount].c=nNum[3];
			g_edge[g_edgeCount].idBegin=nNum[1]+1;//bidirectional
			g_edge[g_edgeCount].idEnd=nNum[0]+1;
			g[nNum[1]+1][nNum[0]+1]=g_edgeCount;
			v[nNum[1]+1].idEdge.push_back(g_edgeCount);
			v[nNum[0]+1].idEdgeFrom.push_back(g_edgeCount);
			g_edgeCount++;
			g_edgeUnD[g_edgeUnDCount].idBegin=nNum[0]+1;
			g_edgeUnD[g_edgeUnDCount].idEnd=nNum[1]+1;
			++g_edgeUnDCount;
		}
	}
	/*for (int i=0;i<g_numVert+1;++i){
		sort(v[i].idEdgeFrom.begin(),v[i].idEdgeFrom.end(),[](int& a,int& b){return g_edge[a].c<g_edge[b].c;});
	}*/
	for (int line=m+5;line<line_num;line++){
		strTmp=topo[line];
		nTmp=strTmp.find(' ');
		nNum[0]=atoi(strTmp.substr(0,nTmp).c_str());
		nTmp2=strTmp.find(' ',nTmp+1);
		nNum[1]=atoi(strTmp.substr(nTmp+1,nTmp2-nTmp).c_str());
		nNum[2]=atoi(strTmp.substr(nTmp2+1,strTmp.length()-nTmp2-1).c_str());
		v[nNum[1]+1].d=nNum[2];//it starts from point 0, but we let it from 1, the left one is all 0 except v0.d
		v[nNum[1]+1].id=nNum[0]+1;//id+1, I think the d maybe [0
		vCons[nNum[0]+1].vid=nNum[1]+1;
		g_totalDem+=nNum[2];
	}
	/*int nCount1=0;
	for (int i=0;i<g_numVert+1;++i){
		int nTmp=0,nTmp2=0;
		sort(v[i].idEdge.begin(),v[i].idEdge.end(),[i](int pos1,int pos2)->bool{
			return g_edge[pos1].c<g_edge[pos2].c;
		});
		int nD=v[i].d;
		for (int j=0;j<v[i].idEdge.size();j++){
			nTmp+=g_edge[v[i].idEdge[j]].u;
			if (nD>=g_edge[v[i].idEdge[j]].u){
				nTmp2+=g_edge[v[i].idEdge[j]].u*g_edge[v[i].idEdge[j]].c;
				nD-=g_edge[v[i].idEdge[j]].u;
			}else{
				nTmp2+=nD*g_edge[v[i].idEdge[j]].c;
				nD=0;
				break;
			}
		}
		if (nTmp<v[i].d)
			cout<<i<<endl;
		if (nTmp2>=g_costServ){
			cout<<"GT g_costServ: "<<i<<endl;
			++nCount1;
		}
	}
	cout<<nCount1;*/
	//make an initial of the networkSimplex
	//v[0].d=INT_MAX;//sink will use v0.d
	for (int i=1;i<g_numVert+1;++i){
		g_edge[g_edgeCount].u=INT_MAX;//super point,>=5000*500
		g_edge[g_edgeCount].idBegin=0;
		g_edge[g_edgeCount].idEnd=i;
		g[0][i]=g_edgeCount;
		g_srcEdge[i]=g_edgeCount;
		v[0].idEdge.push_back(g_edgeCount);
		v[i].idEdgeFrom.push_back(g_edgeCount);
		g_edgeCount++;
		//idBegin=0;//always 0
		g_edgeUnD[g_edgeUnDCount].idEnd=i;//always i
		++g_edgeUnDCount;
	}
	/*for (int k=0;k<g_numDem+1;++k){
		int i=vCons[k].vid;
		g_edge[g_edgeCount].u=v[i].d;//add super sink
		g_edge[g_edgeCount].idBegin=i;
		g_edge[g_edgeCount].idEnd=g_numVert+1;
		g[i][g_numVert+1]=g_edgeCount;
		g_snkEdge[i]=g_edgeCount;
		v[i].idEdge.push_back(g_edgeCount);
		v[g_numVert+1].idEdgeFrom.push_back(g_edgeCount);
		g_edgeCount++;
	}
	g_edge[g_snkEdge[0]].c=g_costServ;*/
	//if (g_bUseHeap) g_zkwtree.initSize(g_edgeCount-1);
	//if (g_bUseHeap) g_zkwtree.initSize(g_edgeUnDCount);
	if (g_bUseHeap){
		g_heap.resize(g_edgeUnDCount);//heap should start from 1
		for (int i=0;i<g_edgeUnDCount;i++){
			g_heap[i].mk=i;
			g_edgeUnD[i].idHeap=i;
		}
	}
	for (int i=0;i<g_edgeCount;++i)//repaired start from 0
		g_edgeTmp[i]=g_edge[i];
	v[0].next=1;
	v[1].prec=0;
	for (int i=1;i<g_numVert+1;++i){
		if (v[i].id>0){//d maybe 0
			g_edge[g_srcEdge[i]].x=v[i].d;//the left xij is all 0, and the initial feasible tree <0,1~g_numVert> is strong for they're all away from 0
		}//initial x is 0, no need for else
		v[i].parent=0;//all point's parent is 0 && v[0].depth=0
		v[i].directToParent=false;//v[i].direction=false cannot be omitted!!!
		v[i].depth=1;//v[0].depth=0, depth is from 0
		v[i].next=i+1;//i's next dfs node is i+1 except vn.next
		v[i+1].prec=i;
	}
	v[g_numVert].next=0;//!here is important
	v[0].prec=g_numVert;
	/*for (int i=0;i<g_numVert;++i){
		for (int j=0;j<g_numVert;j++){
			cout<<g[i][j].u<<" ";
		}
		cout<<endl;
	}
	for (int i=0;i<g_numVert;++i){
		cout<<v[i].d<<" ";
	}*/
	set<int> nServerPos,nGreedyServerPos;
	int nMinCost=g_numDem*g_costServ,nMinPos[MAX_CONSUME_NUM]={0},nMinPosCount=0;
	/*int nMinCost=0,nMinPos[MAX_CONSUME_NUM]={5,18,21,23,26,37,38,43,47,50,55,57,58,61,62,67,73,78,82,86,90,98
	,104,107,109,115,124,127,129,136,138,140,148,156,159,160},nMinPosCount=36;
	for (int i=0;i<nMinPosCount;++i){
		if (nMinPos[i]>0){
			nMinPos[i]++;
		}
	}*/
	//branchAndBound(g_numVert,g_costServ,nServerPos);
	/*bitset<1010> seed,best;
    for (int i=0;i<g_numVert;++i)
        if (v[i+1].id)
            seed[i]=1;
    int z_seed=Flow(seed).val(),z_best;
    cout<<"Direct Seed: "<<z_seed<<endl;
    for(int changed=1,tm=0;changed;){
        changed=0;
        for(int i=0;i<g_numVert;++i){
            ++tm;
            seed[i].flip();
            int t=Flow(seed).val();
            if(t==~0u>>1){
                seed[i].flip();
                continue;
            }
            if(t<z_seed){
                z_seed=t;
                changed=1;
            }else{
                seed[i].flip();
            }
        }
    }
    cout<<"Greedy Seed: "<<z_seed<<endl;
	for (int i=0;i<seed.size();++i){
		nGreedyServerPos.insert(i+1);
	}*/
	if (g_numVert<200)
	for (int k=0;k<g_numDem;k++){
		int min2Cost=INT_MAX,min2Pos=0,minPos=0;
		int nCostBefore=nMinCost;
		for (int i=1;i<g_numVert+1;++i){
			if (nGreedyServerPos.find(i)==nGreedyServerPos.end()){
				nServerPos.clear();
				nServerPos.insert(nGreedyServerPos.begin(),nGreedyServerPos.end());
				nServerPos.insert(i);
				networkSimplexAlg(nServerPos,g_edge);
				int nTotalCost=0,nTotalNum=0;
				int nType=1;
				if (nType==1){
					for (int i=0;i<g_edgeCount;++i){
						g_edgeTmp[i].x=g_edge[i].x;
					}
					processNetwork(nServerPos,nTotalCost,g_edgeTmp,false);//1
					//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
					//nTotalCost=0;nTotalNum=setTmp.size();
					calcCost(nTotalCost,nTotalNum,g_edgeTmp,false);
				}else if (nType==0){
					processNetwork(nServerPos,nTotalCost,g_edge,true,0);
					networkSimplexAlg(nServerPos,g_edge);
					nTotalCost=0;nTotalNum=nServerPos.size();
					calcCost(nTotalCost,nTotalNum,g_edge,false);
				}else{
					for (int i=0;i<g_edgeCount;++i){
						g_edgeTmp[i].x=g_edge[i].x;
					}
					processNetwork(nServerPos,nTotalCost,g_edgeTmp,false,1);
					//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
					//nTotalCost=0;nTotalNum=setTmp.size();
					calcCost(nTotalCost,nTotalNum,g_edgeTmp,false);
				}
				if (nMinCost>nTotalCost){
					nMinCost=nTotalCost;
					minPos=i;
				}
				if (min2Cost>nTotalCost){
					min2Cost=nTotalCost;
					min2Pos=i;
					//cout<<"min2Pos History: "<<i<<" Cost: "<<min2Cost<<endl;
				}
			}
			if (clock()-g_tmStart>85*CLOCKS_PER_SEC)
				break;
		}
		if (minPos>0){
			nGreedyServerPos.insert(minPos);
			nMinPos[nMinPosCount]=minPos;
			nMinPosCount++;
			//cout<<nCostBefore<<" after greedy: "<<nMinCost<<" "<<"pos(in program):"<<minPos<<endl;
			//if (nCostBefore-nMinCost<g_costServ) break;
		}else{
			break;
			//nGreedyServerPos.insert(min2Pos);
		}
		if (clock()-g_tmStart>85*CLOCKS_PER_SEC)
			break;
	}
	/*nMinPosCount+=2;
	for (int i=1;i<g_numVert+1;++i){
		for (int i2=1;i2<g_numVert+1;i2++){
			if (i2!=i&&(nGreedyServerPos.find(i)==nGreedyServerPos.end()||
				nGreedyServerPos.find(i2)==nGreedyServerPos.end())){
				nServerPos.clear();
				nServerPos.insert(nMinPos,nMinPos+nMinPosCount-2);
				nServerPos.insert(i);
				nServerPos.insert(i2);
				networkSimplexAlg(g_numVert,g_costServ,nServerPos);
				int nTotalCost=0,nTotalNum=nMinPosCount;
				processNetwork(g_numVert,nServerPos,g_costServ,nTotalCost,false);
				//nTotalCost=0;
				calcCost(g_costServ,g_numVert,nTotalCost,nTotalNum);
				if (nMinCost>nTotalCost){
					nMinCost=nTotalCost;
					nMinPos[nMinPosCount-2]=i;
					nMinPos[nMinPosCount-1]=i2;
				}
			}
			if (clock()-g_tmStart>85*CLOCKS_PER_SEC)
				break;
		}
		if (clock()-g_tmStart>85*CLOCKS_PER_SEC)
			break;
	}*/
	//GA-genetic algorithm

	/*srand((int)time(0));
	int nTry=0;
	if (g_numVert>500){
        Flow best_flow=xjb_search();
        nMinCost=best_flow.cost;
        for (int i=0;i<g_numVert;i++){
            if (best_flow.modifiedservers[i])
                nServerPos.insert(i+1);
        }
        //delete &best_flow;
	}
	if (g_numVert<=500){
		int nPop,nType=1;
		if (g_numVert<200){
			nPop=160;//500;
		}else{
			nPop=200;//200;
			//nType=0;
		}
		int nMCTmp,nMCCount=0;
		do{
		++nMCCount;
		GenAlg genAlg(g_numVert,nPop,g_numDem,g_costServ,nGreedyServerPos);//20~30 chromosomes
		//genAlg.startGA(85);
		genAlg.startPSO(87,0.9*RAND_MAX,0.5*RAND_MAX,0.5*RAND_MAX,nType);
		//genAlg.startPSO(85,1,2,2);
		nMCTmp=genAlg.getMinCost();
		if (nMinCost>nMCTmp){
			nMinCost=nMCTmp;
			nServerPos=genAlg.getBestServerPos();
			nMCCount=0;
		}
		cout<<nMinCost<<endl;
		}while (nMCCount<nTry);
	}*/

	//nMinCost=0;nServerPos.clear();nServerPos.insert(nMinPos,nMinPos+nMinPosCount);
	//xjbs();

	//SA
	//startSA(nMinCost,nServerPos,nGreedyServerPos);

	//xjbs
	startXjbs(nMinCost,nServerPos,nGreedyServerPos);

	cout<<"use time: "<<(clock()-g_tmStart)*1000/CLOCKS_PER_SEC<<"ms"<<endl;

	cout<<"nMinCost="<<nMinCost<<" minPos=";
	/*ostream_iterator<int,char> out_iter(cout," ");
	copy(nMinServerPos.begin(),nMinServerPos.end(),out_iter);*/
	/*for_each(nMinServerPos.begin(),nMinServerPos.end(),[](int pos){
		cout<<" "<<pos;
	});*/
	for (auto i=nServerPos.begin();i!=nServerPos.end();++i){
		cout<<" "<<*i;
		if (v[*i].id)
			cout<<"*";
	}
	cout<<endl;
	for (int i=0;i<nMinPosCount;++i)
		cout<<" "<<nMinPos[i];
	cout<<endl;

	/*if (g_numVert==0&&g_numDem>0)//customers' need exists while no nodes, impossible
		strTopo="NA";
	else{*/
	//printTree();
	ostringstream sout;
	int nLines=0;
	if (nMinCost<g_numDem*g_costServ){
		/*nServerPos.clear();
		nServerPos.insert(nMinPos,nMinPos+nMinPosCount);*/
		networkSimplexAlg(nServerPos,g_edge);
		int nTotalCost=0,nTotalNum=nMinPosCount;
		/*nTotalNum=nServerPos.size();
		while (!calcCost(g_costServ,g_numVert,nTotalCost,nTotalNum,g)){
			processNetwork(nServerPos,nTotalCost,g_edge,true);
			nTotalNum=nServerPos.size();
			networkSimplexAlg(nServerPos,g_edge);
			nTotalCost=0;
		}*/
		for (int i=0;i<g_edgeCount;++i){
			g_edgeTmp[i].x=g_edge[i].x;
		}
		processNetwork(nServerPos,nTotalCost,g_edgeTmp,true);
		calcCost(nTotalCost,nTotalNum,g_edgeTmp);
		networkSimplexAlg(nServerPos,g_edge);

		cout<<"Final nMinCost="<<nTotalCost<<" minPos=";
		for (auto i=nServerPos.begin();i!=nServerPos.end();++i)
			cout<<" "<<*i;
		cout<<endl;

		int iMap=g_srcEdge[1]-1;
		for (int i=1;i<g_numVert+1;++i){
			++iMap;
			while (g_edge[iMap].x>0){
				sout<<i-1<<" ";
				int nFlow=g_edge[iMap].x;
				if (v[i].id>0&&v[i].d>0){
					if (nFlow>=v[i].d){
						nFlow=v[i].d;
						v[i].d=0;
						++g_leftCus;
					}else{
						v[i].d-=nFlow;
					}
					sout<<v[i].id-1<<" "<<nFlow;
				}else
					printPath(i,nFlow,sout);
				if (g_leftCus<g_numDem) sout<<endl;
				++nLines;
				g_edge[iMap].x-=nFlow;
				if (g_edge[iMap].x==0){
					g_totalCost+=g_costServ;
				}
			}
		}
	}
	if (nLines>0&&nLines<=50000){
		char cstrTmp[6]="";
		sprintf(cstrTmp,"%d",nLines);
		strTopo=string(cstrTmp)+"\n\n"+sout.str();
	}else{
		sout.str("");
		sout<<g_numDem<<endl<<endl;//the most simple ans
		for (int line=m+5;line<line_num;line++){
			strTmp=topo[line];
			nTmp=strTmp.find(' ');
			nTmp2=strTmp.find(' ',nTmp+1);
			nNum[1]=atoi(strTmp.substr(nTmp+1,nTmp2-nTmp).c_str());
			nNum[2]=atoi(strTmp.substr(nTmp2+1,strTmp.length()-nTmp2-1).c_str());
			v[nNum[1]+1].d=nNum[2];//it starts from point 0, but we let it from 1, the left one is all 0 except v0.d
		}
		nTmp=0;
		for (int i=1;i<g_numVert+1;++i){
			if (v[i].id>0){
				if (++nTmp==g_numDem)
					sout<<i-1<<" "<<v[i].id-1<<" "<<v[i].d;
				else
					sout<<i-1<<" "<<v[i].id-1<<" "<<v[i].d<<endl;
			}
		}
		strTopo=sout.str();
		g_totalCost=g_numDem*g_costServ;
	}
	cout<<"Total cost: "<<g_totalCost<<endl;
	cout<<g_count<<endl<<"simplex num: "<<g_count2<<endl;
	strTopo=strTopo;
    cin.get();

	// 直接调用输出文件的方法输出到指定文件中(ps请注意格式的正确性，如果有解，第一行只有一个数据；第二行为空；第三行开始才是具体的数据，数据之间用一个空格分隔开)
	topo_file=strTopo.c_str();
	write_result(topo_file, filename);

}

void branchAndBound(set<int>& nServerPos){

}

void netSAinit(NodeEdge g_edge[]){//0 is completely reset
	//make an initial strongly feasible tree or solution
	for (int i=0;i<g_edgeCount-g_numVert;++i){
		g_edge[i].x=0;
	}
	v[0].next=1;
	v[1].prec=0;
	/*for (int i=0;i<g_edgeCount;++i){
		g_edge[i].x=0;
	}
	g_edge[g_snkEdge[0]].x=g_totalDem;
	v[g_numVert+1].parent=0;
	v[g_numVert+1].directToParent=false;
	v[g_numVert+1].depth=1;
	v[g_numVert+1].next=0;
	v[0].prec=g_numVert+1;
	v[g_numVert+1].pi=-g_edge[g_snkEdge[0]].c;*/
	for (int i=1;i<g_numVert+1;++i){
		if (v[i].id>0){//d maybe 0
			g_edge[g_srcEdge[i]].x=v[i].d;//the left xij is all 0, and the initial feasible tree <0,1~g_numVert> is strong for they're all away from 0
		}else{
			g_edge[g_srcEdge[i]].x=0;
		}
		v[i].parent=0;//all point's parent is 0 && v[0].depth=0
		v[i].directToParent=false;//v[i].direction=false cannot be omitted!!!
		v[i].depth=1;//v[0].depth=0, depth is from 0
		v[i].next=i+1;//i's next dfs node is i+1 except vn.next
		v[i+1].prec=i;//notice overflow!!!
	}
	v[g_numVert].next=0;//!here is important
	v[0].prec=g_numVert;
	/*//calculate π[i], let π0=0 or the potential of the root is 0
	for (int i=1;i<g_numVert+1;++i){//cπij=cij-πi+πj=0;here 0/g_costServ-0+πi=0
		v[i].pi=-g_edge[g_srcEdge[i]].c;
	}*/
}
void networkSimplexAlg(set<int>& pos,NodeEdge g_edge[]){
	int nTmp,nTmp2,nTmp3;
	g_count2++;
	//make an initial strongly feasible tree or solution
	for (int i=1;i<g_numVert+1;++i){
		if (pos.find(i)!=pos.end()){
			g_edge[g_srcEdge[i]].c=0;
			//g_edge[g_srcEdge[i]].u=INT_MAX;
		}else{
			g_edge[g_srcEdge[i]].c=g_costServ;
			//g_edge[g_srcEdge[i]].u=0;
		}
	}
	int d;
	for (int i=v[0].next;i!=0;i=v[i].next){
		if (v[i].parent==0){
			d=-g_edge[g_srcEdge[i]].c-v[i].pi;
		}
		v[i].pi+=d;
	}
	//find the maximum residual reduced cost(-cπij/cπij for L/U)
	/*for (int i=0;i<g_edgeCount;++i){
		g_edge[i].cpi=g_edge[i].c-v[g_edge[i].idBegin].pi+v[g_edge[i].idEnd].pi;
		if (g_bUseHeap) zkwtreeModify(i);
	}*/
	for (int i=0;i<g_edgeUnDCount-g_numVert;++i){
		changeEdgeUnD(i,g_edge);
	}
	int iMapEdgeUnD=g_edgeUnDCount-g_numVert;
	for (int i=1;i<g_numVert+1;++i){//only one direction & don't modify the idBegin or End
		if (g_edge[g_srcEdge[i]].x==0)//here notice!!!
			g_edgeUnD[iMapEdgeUnD].cpi=-(g_edge[g_srcEdge[i]].c+v[i].pi);
		else
			g_edgeUnD[iMapEdgeUnD].cpi=g_edge[g_srcEdge[i]].c+v[i].pi;//idBegin=0,End is i;
		if (g_bUseHeap) zkwtreeModify(iMapEdgeUnD);
		++iMapEdgeUnD;
	}
	int max;
	do{
		int p,q;
		max=0;
		/*for (int i=0;i<g_numVert+1;++i){//maybe cache can not save it leads to its slow speed!!!
			for (int k=0;k<v[i].edge.size();k++){
				int j=v[i].edge[k];
				//if (!(!v[j].directToParent&&v[j].parent==i||v[i].directToParent&&v[i].parent==j)){//if the arc exists && it !∈ T;i!=j belongs to .u>0
					g[i][j].cpi=g[i][j].c-v[i].pi+v[j].pi;
					if (!g[i][j].x){//belongs to L
						if (g[i][j].cpi<-max){
							max=-g[i][j].cpi;
							p=i;q=j;
						}
					}else{//belongs to U
						if (g[i][j].cpi>max){
							max=g[i][j].cpi;
							p=i;q=j;
						}
					}
				//}//if don't use this if, the speed will be boosted
			}
		}*/	
		/*for (int j=0;j<g_numVert+1;++j){
			for (int k=0;k<v[j].idEdge.size();++k){
				int i=v[j].idEdge[k];
				int ni=g_edge[i].idBegin;
				int nj=g_edge[i].idEnd;
				//int nCpiij=g_edge[i].c-v[ni].pi+v[nj].pi;
				int nCpiij=g_edge[i].cpi;
				if (g_edge[i].x==0){
					if (nCpiij<-max){
						max=-nCpiij;
						p=ni;q=nj;
						//break;
					}
				}else{
					if (nCpiij>max){
						max=nCpiij;
						p=ni;q=nj;
						//break;
					}
				}
			}
		}*/
		//int nTmTmp=clock();
		if (g_bUseHeap){
			/*int maxEdge=g_zkwtree.top_pos()-1;
			max=g_zkwtree.pop();*/
			int maxEdge=g_heap[0].mk;
			max=g_heap[0].val;
			//cout<<max<<" "<<maxEdge<<endl;
			//p=g_edge[maxEdge].idBegin;q=g_edge[maxEdge].idEnd;
			p=g_edgeUnD[maxEdge].idBegin;q=g_edgeUnD[maxEdge].idEnd;
		}else{
			/*int nCpiij;
			for (int i=0;i<g_edgeCount;++i){
				//if (g_edge[i].u){
				if (g_edge[i].x==0){
					nCpiij=-g_edge[i].cpi;
					if (nCpiij>max){
						max=nCpiij;
						p=g_edge[i].idBegin;q=g_edge[i].idEnd;
						//break;
					}
				}else{
					nCpiij=g_edge[i].cpi;
					if (nCpiij>max){
						max=nCpiij;
						p=g_edge[i].idBegin;q=g_edge[i].idEnd;
						//break;
					}
				}
				//}
			}*/
			for (int i=0;i<g_edgeUnDCount;++i){
				if (g_edgeUnD[i].cpi>max){//this is a modified cpi!~
					max=g_edgeUnD[i].cpi;
					p=g_edgeUnD[i].idBegin;q=g_edgeUnD[i].idEnd;
				}
			}
		}
		//g_count+=clock()-nTmTmp;
		if (max!=0){//try entering the arc(p,q)-><p,q> & take out the arc<k,l>->(k,l)
			//find the outgoing arc<k,l>
			int k=p,l=q;//<p,q>∈W+ for L && ∈W- for U
			bool outPTop=true;//suppose <k,l> is in the way from p to top intersection (true includes <p,q> itself!)
			int topP=p,topQ=q,min=g_edge[g[p][q]].u;//<p,q>∈W+ for L,residual flow is u-x,here x=0;<p,q>∈W- for U,residual flow is x-0,here x=u;
			if (g_edge[g[p][q]].x==0){//L,W+ is <p,q>
				while (topP!=topQ){
					if (v[topP].depth>v[topQ].depth){//go from P to top intersection
						if (v[topP].directToParent){//∈W-
							findOutgoing(k,l,outPTop,true,min,topP,v[topP].parent,true);//residual flow is x-0;= means the first min choice along the W+ from top intersection
						}else{//∈W+
							findOutgoing(k,l,outPTop,true,min,v[topP].parent,topP,true,true);//residual flow is u-x;= means the first min choice along the W+
						}
						topP=v[topP].parent;
					}else if (v[topP].depth<v[topQ].depth){
						if (v[topQ].directToParent){//∈W+
							findOutgoing(k,l,outPTop,false,min,topQ,v[topQ].parent,false,true);//residual flow is u-x;!= means the first min choice along the W+
						}else{//∈W-
							findOutgoing(k,l,outPTop,false,min,v[topQ].parent,topQ);//residual flow is x;!= means the first min choice along the W+
						}
						topQ=v[topQ].parent;
					}else{//v[topP].depth==v[topQ].depth
						//topP up to its parent
						if (v[topP].directToParent){//∈W-
							findOutgoing(k,l,outPTop,true,min,topP,v[topP].parent,true);//residual flow is x-0;= means the first min choice along the W+ from top intersection
						}else{//∈W+
							findOutgoing(k,l,outPTop,true,min,v[topP].parent,topP,true,true);//residual flow is u-x;= means the first min choice along the W+
						}
						topP=v[topP].parent;
						//topQ up to its parent
						if (v[topQ].directToParent){//∈W+
							findOutgoing(k,l,outPTop,false,min,topQ,v[topQ].parent,false,true);//residual flow is u-x;!= means the first min choice along the W+
						}else{//∈W-
							findOutgoing(k,l,outPTop,false,min,v[topQ].parent,topQ);//residual flow is x;!= means the first min choice along the W+
						}
						topQ=v[topQ].parent;
					}
				}
			}else{//U,W+ is <q,p>
				while (topP!=topQ){
					if (v[topP].depth>v[topQ].depth){
						if (v[topP].directToParent){//∈W+
							findOutgoing(k,l,outPTop,true,min,topP,v[topP].parent,false,true);
						}else{//∈W-
							findOutgoing(k,l,outPTop,true,min,v[topP].parent,topP,false);
						}
						topP=v[topP].parent;
					}else if (v[topP].depth<v[topQ].depth){//go from Q to top intersection
						if (v[topQ].directToParent){//∈W-
							findOutgoing(k,l,outPTop,false,min,topQ,v[topQ].parent,true);
						}else{//∈W+
							findOutgoing(k,l,outPTop,false,min,v[topQ].parent,topQ,true,true);
						}
						topQ=v[topQ].parent;
					}else{//v[topP].depth==v[topQ].depth
						//topP up to its parent
						if (v[topP].directToParent){//∈W+
							findOutgoing(k,l,outPTop,true,min,topP,v[topP].parent,false,true);
						}else{//∈W-
							findOutgoing(k,l,outPTop,true,min,v[topP].parent,topP,false);
						}
						topP=v[topP].parent;
						//topQ up to its parent
						if (v[topQ].directToParent){//∈W-
							findOutgoing(k,l,outPTop,false,min,topQ,v[topQ].parent,true);
						}else{//∈W+
							findOutgoing(k,l,outPTop,false,min,v[topQ].parent,topQ,true,true);
						}
						topQ=v[topQ].parent;
					}
				}
			}
			//update xij: traverse the loop
			bool bXpqL=true;//true means xpq==0
			if (min==0){
				if (g_edge[g[p][q]].x==0){//L,W+ is <p,q>
				}else{
					bXpqL=false;//update bXpqL for later updating πi
				}
			}else{//min>0
				topP=p,topQ=q;
				if (g_edge[g[p][q]].x==0){//L,W+ is <p,q>
					while (topP!=topQ){
						if (v[topP].depth>v[topQ].depth){//go from P to top intersection
							if (v[topP].directToParent){//∈W-
								g_edge[g[topP][v[topP].parent]].x-=min;
							}else{//∈W+
								g_edge[g[v[topP].parent][topP]].x+=min;
							}
							topP=v[topP].parent;
						}else if (v[topP].depth<v[topQ].depth){
							if (v[topQ].directToParent){//∈W+
								g_edge[g[topQ][v[topQ].parent]].x+=min;
							}else{//∈W-
								g_edge[g[v[topQ].parent][topQ]].x-=min;
							}
							topQ=v[topQ].parent;
						}else{//v[topP].depth==v[topQ].depth
							//topP up to its parent
							if (v[topP].directToParent){//∈W-
								g_edge[g[topP][v[topP].parent]].x-=min;
							}else{//∈W+
								g_edge[g[v[topP].parent][topP]].x+=min;
							}
							topP=v[topP].parent;
							//topQ up to its parent
							if (v[topQ].directToParent){//∈W+
								g_edge[g[topQ][v[topQ].parent]].x+=min;
							}else{//∈W-
								g_edge[g[v[topQ].parent][topQ]].x-=min;
							}
							topQ=v[topQ].parent;
						}
					}
					g_edge[g[p][q]].x+=min;//update xpq
				}else{//U,W+ is <q,p>
					while (topP!=topQ){
						if (v[topP].depth>v[topQ].depth){
							if (v[topP].directToParent){//∈W+
								g_edge[g[topP][v[topP].parent]].x+=min;
							}else{//∈W-
								g_edge[g[v[topP].parent][topP]].x-=min;
							}
							topP=v[topP].parent;
						}else if (v[topP].depth<v[topQ].depth){//go from Q to top intersection
							if (v[topQ].directToParent){//∈W-
								g_edge[g[topQ][v[topQ].parent]].x-=min;
							}else{//∈W+
								g_edge[g[v[topQ].parent][topQ]].x+=min;
							}
							topQ=v[topQ].parent;
						}else{//v[topP].depth==v[topQ].depth
							//topP up to its parent
							if (v[topP].directToParent){//∈W+
								g_edge[g[topP][v[topP].parent]].x+=min;
							}else{//∈W-
								g_edge[g[v[topP].parent][topP]].x-=min;
							}
							topP=v[topP].parent;
							//topQ up to its parent
							if (v[topQ].directToParent){//∈W-
								g_edge[g[topQ][v[topQ].parent]].x-=min;
							}else{//∈W+
								g_edge[g[v[topQ].parent][topQ]].x+=min;
							}
							topQ=v[topQ].parent;
						}
					}
					g_edge[g[p][q]].x-=min;//update xpq
					bXpqL=false;//update bXpqL for later updating πi
				}
			}
			//update the tree T=T1+<p,q>or<q,p>+T2
			if (k==p&&l==q){//don't update the tree structure&&πi;it's important to classify this situation for there's no concept of a higher node
				changeEdgeUnD(g[p][q]>>1,g_edge);//0i cannot enter here!
			}else{//(including the case of entering <p,q>&&deleting <q,p>)
				int nH,nL,nPQH,nPQL;//nH means the higher node id between k&&l;nPQH means the higer node id between p&&q
				if (v[k].depth<v[l].depth){
					nH=k;nL=l;
				}else{//v[k].depth>v[l].depth
					nH=l;nL=k;
				}
				if (outPTop){//if the outgoing arc is between p && top intersection
					nPQH=q;nPQL=p;//the higher one must be q
				}else{//vice versa
					nPQH=p;nPQL=q;
				}
		
				//enter <p,q>or<q,p>;only enter
				/*if (1||p==2&&q==5){
					int pCheck=0,num=1;
					while (v[pCheck].next!=0){
						if (pCheck!=0&&v[pCheck].depth!=v[v[pCheck].parent].depth+1)
							while (true){
								cout<<"ErrorDepth!";
							}
						int nTmp=pCheck;
						pCheck=v[pCheck].next;num++;
						if (v[pCheck].parent==nTmp||v[pCheck].parent!=nTmp&&v[pCheck].depth<=v[nTmp].depth){
						}else{
							while (true){
								cout<<"ErrorParent!";
							}
						}
					}
					if (num!=g_numVert+1+1){
						cout<<"ErrorNum!";
					}
					//p=2;
				}*/
				int pSearch=nPQH;//pSearch start from nPQH
				nTmp=v[pSearch].next;//save it for later T2 last dps node points to it;but it may be nL if nH==nTopP
				v[pSearch].next=nPQL;//update next
				int nTmpNLPrec=v[nPQL].prec;
				v[nPQL].prec=pSearch;
				nTmp2=v[nPQL].parent;//save it for later use,first enter&&go out processed point
				v[nPQL].parent=nPQH;//upadate parent
				int nTmpDirect=v[nPQL].directToParent;//save for later nTmp4.direct use!!!
				v[nPQL].directToParent=nPQL==p?true:false;//update direction for entering
				//delete <k,l>or<l,k>;update T2 tree structure
				pSearch=nPQL;//the same as pSearch=v[pSearch].next for we don't want to find the same depth with the same point
				int nTmp4=nTmp2;
				nTmp2=nPQL;
				while (true){
					//enter;no need to search from nTmp2 again
					while (v[v[pSearch].next].depth>v[nTmp2].depth){//find at most the same depth
						pSearch=v[pSearch].next;
					}
					if (nTmp2==nL){//end condition;don't use depth to judge or it will lead a big problem!
						break;
					}
					nTmp3=v[pSearch].next;//save it for later use;can use temporary variables
					v[pSearch].next=nTmp4;//update next
					nTmpNLPrec=v[nTmp4].prec;//when nTmp4 is nL, its prec is reserved
					v[nTmp4].prec=pSearch;
					int nTmp5=v[nTmp4].parent;//save it for later shift
					v[nTmp4].parent=nTmp2;//upadate parent;nTmp2 is released
					int nTmpDirect2=v[nTmp4].directToParent;//save for later shift
					v[nTmp4].directToParent=!nTmpDirect;//update direction for changed parent;nTmpDirect is released
					nTmpDirect=nTmpDirect2;//shift;nTmpDirect2 is released
					//go out
					pSearch=nTmp4;//it's the same as pSearch=v[pSearch].next;
					//the jump reason is both we don't want to find the same depth with the same point in the next cycle
					//and nTmp4!=v[nTmp4].parent is always true except root, but pSearch.next won't be 0 for its depth>1>0 so we can also delete this sentence
					while (v[pSearch].next!=v[nTmp4].parent){//now parent is nonexistent
						pSearch=v[pSearch].next;
					}
					v[pSearch].next=nTmp3;//nTmp3 is released
					v[nTmp3].prec=pSearch;
					//shift nTmp4,nTmp5->nTmp2,nTmp4 then nTmp5 is released
					nTmp2=nTmp4;nTmp4=nTmp5;
				}
				nTmp3=v[pSearch].next;//save it for final update next
				//int nT2Final=pSearch;
				if (nTmp!=nL){//if nTmp is nL, it may be nonexistent
					v[pSearch].next=nTmp;//next is nTmp
					v[nTmp].prec=pSearch;
					//only old tree from the root nL goes out
					/*if (nH==nPQH){
						//pSearch=nTmp is wrong for it may loop the tree again!
					}else
						pSearch=nH;*/
					if (nTmpNLPrec!=nPQH)//use nLPrec to boost up!
						pSearch=nTmpNLPrec;
					/*while (v[pSearch].next!=nL){//now <nH,nL> is nonexistent; this while may waste a lot of time
						if (v[pSearch].next==nPQH){//if encounter the <nPQH,nPQL>+T2 then jump it
							pSearch=nT2Final;
						}else
							pSearch=v[pSearch].next;
					}*/
					v[pSearch].next=nTmp3;//nTmp3 is released
					v[nTmp3].prec=pSearch;
				}else{//if (nTmp==nL) then topP==nH! if opposite the condition it's not always true!
					//v[pSearch].next=nTmp3;//nTmp3 is released && don't need to update anything including v[nH].next....next=nL for it has already been updated to nPQL
					nTmp=nTmp3;//correct the T2 final node's pointing place
				}
				//update tree depth&&πi
				//int nTmTmp=clock();
				pSearch=nPQL;
				v[pSearch].depth=v[v[pSearch].parent].depth+1;
				if (nPQH==p&&bXpqL||nPQH==q&&!bXpqL){
					v[pSearch].pi+=max;//-=-max because cπnPQHnPQL=cπpq for L=-cπpq for U<0 && max always>0
				}else{
					v[pSearch].pi-=max;//vice versa
				}
				/*for (int k=0;k<(int)v[pSearch].idEdge.size();++k){
					int i=v[pSearch].idEdge[k];
					g_edge[i].cpi=g_edge[i].c-v[pSearch].pi+v[g_edge[i].idEnd].pi;
					if (g_bUseHeap) zkwtreeModify(i);
				}
				for (int k=0;k<(int)v[pSearch].idEdgeFrom.size();++k){
					int i=v[pSearch].idEdgeFrom[k];
					g_edge[i].cpi=g_edge[i].c-v[g_edge[i].idBegin].pi+v[pSearch].pi;
					if (g_bUseHeap) zkwtreeModify(i);
				}*/
				for (int k=0;k<(int)v[pSearch].idEdge.size();++k){//pSearch cannot be 0
					int i=v[pSearch].idEdge[k]>>1;//it's the same as using idEdgeUnD if both are from 0
					changeEdgeUnD(i,g_edge);
				}
				if (g_edge[g_srcEdge[pSearch]].x==0)
					g_edgeUnD[g_edgeUnDCount-g_numVert+pSearch-1].cpi=-(g_edge[g_srcEdge[pSearch]].c+v[pSearch].pi);
				else
					g_edgeUnD[g_edgeUnDCount-g_numVert+pSearch-1].cpi=g_edge[g_srcEdge[pSearch]].c+v[pSearch].pi;//idBegin=0,End is pSearch;
				if (g_bUseHeap) zkwtreeModify(g_edgeUnDCount-g_numVert+pSearch-1);
				while (v[pSearch].next!=nTmp){//traverse the T2;nTmp is released here for label
					pSearch=v[pSearch].next;
					v[pSearch].depth=v[v[pSearch].parent].depth+1;
					if (nPQH==p&&bXpqL||nPQH==q&&!bXpqL){
						v[pSearch].pi+=max;//-=-max because cπnPQHnPQL=cπpq for L=-cπpq for U<0 && max always>0
					}else{
						v[pSearch].pi-=max;//vice versa
					}
					for (int k=0;k<(int)v[pSearch].idEdge.size();++k){//pSearch cannot be 0
						int i=v[pSearch].idEdge[k]>>1;//it's the same as using idEdgeUnD if both are from 0
						changeEdgeUnD(i,g_edge);
					}
					if (g_edge[g_srcEdge[pSearch]].x==0)
						g_edgeUnD[g_edgeUnDCount-g_numVert+pSearch-1].cpi=-(g_edge[g_srcEdge[pSearch]].c+v[pSearch].pi);
					else
						g_edgeUnD[g_edgeUnDCount-g_numVert+pSearch-1].cpi=g_edge[g_srcEdge[pSearch]].c+v[pSearch].pi;//idBegin=0,End is pSearch;
					if (g_bUseHeap) zkwtreeModify(g_edgeUnDCount-g_numVert+pSearch-1);
				}
				//g_count+=clock()-nTmTmp;
			}
		}
		//g_count+=clock()-nTmTmp;
	}while (max!=0);
}

void changeEdgeUnD(int i,NodeEdge g_edge[]){
	int ki=g_edgeUnD[i].idBegin,kj=g_edgeUnD[i].idEnd;
	int ke=g[ki][kj];
	if (g_edge[ke].x>0){//cpi maybe 0(x<u) or +cπij(x==u)
		g_edgeUnD[i].cpi=g_edge[ke].c-v[ki].pi+v[kj].pi;
		//the opposite way must be x==0&& cpi=-cπji=-(c-pij+pii)=pij-pii-c,so it's impossible greater than cpi now
	}else if (g_edge[ke^1].x>0){//the opposite edge!~
		g_edgeUnD[i].cpi=g_edge[ke].c-v[kj].pi+v[ki].pi;//the same reason & notice edge[ke^1].c is the same
		g_edgeUnD[i].idBegin=kj;g_edgeUnD[i].idEnd=ki;//exchange the end points
	}else{//both x=0 so both cpi=-cπij so take the smaller -pii+pij to make cpi larger
		if (v[ki].pi>=v[kj].pi){
			g_edgeUnD[i].cpi=-(g_edge[ke].c-v[ki].pi+v[kj].pi);//idBegin=ki;idEnd=kj;
		}else{
			g_edgeUnD[i].cpi=-(g_edge[ke].c-v[kj].pi+v[ki].pi);
			g_edgeUnD[i].idBegin=kj;g_edgeUnD[i].idEnd=ki;//exchange the end points
		}
	}
	if (g_bUseHeap) zkwtreeModify(i);
}
void zkwtreeModify(int i){
	int _pos=g_edgeUnD[i].idHeap;
	g_heap[_pos].val=g_edgeUnD[i].cpi;
	//go down
	int _secondChild=_pos*2+2;
	while (_secondChild<g_edgeUnDCount){
		if (g_heap[_secondChild]<g_heap[_secondChild-1])
			--_secondChild;
		if (g_heap[_secondChild].val<=g_edgeUnD[i].cpi)
			break;
		g_heap[_pos]=g_heap[_secondChild];
		g_edgeUnD[g_heap[_pos].mk].idHeap=_pos;
		_pos=_secondChild;
		_secondChild=_pos*2+2;
	}
	if (_secondChild==g_edgeUnDCount){
		//if (g_edgeUnD[i].cpi<g_heap[_secondChild-1].val){
			g_heap[_pos]=g_heap[_secondChild-1];
			g_edgeUnD[g_heap[_pos].mk].idHeap=_pos;
			_pos=_secondChild-1;
		//}
	}
	//go up
	int _parent=(_pos-1)/2;
	while (_pos>0&&g_heap[_parent].val<g_edgeUnD[i].cpi){
		g_heap[_pos]=g_heap[_parent];
		g_edgeUnD[g_heap[_pos].mk].idHeap=_pos;
		_pos=_parent;
		_parent=(_pos-1)/2;
	}
	g_heap[_pos].val=g_edgeUnD[i].cpi;g_heap[_pos].mk=i;//g_heap[_pos]=_val;
	g_edgeUnD[i].idHeap=_pos;//_val.mk==i,and after= we have val.mk==_pos.mk
	//g_zkwtree.modify(i+1,g_edgeUnD[i].cpi);
	/*if (g_edge[i].x)
		g_zkwtree.modify(i,g_edge[i].cpi);
	else
		g_zkwtree.modify(i,-g_edge[i].cpi);*/
}
void findOutgoing(int& k,int& l,bool& outPTop,bool bResult,
				  int& min,int p,int q,bool bEqual,bool directPlus){
	if (directPlus){
		if (bEqual){
			if (min>=g_edge[g[p][q]].u-g_edge[g[p][q]].x){
				k=p;l=q;outPTop=bResult;
				min=g_edge[g[p][q]].u-g_edge[g[p][q]].x;
			}
		}else{
			if (min>g_edge[g[p][q]].u-g_edge[g[p][q]].x){
				k=p;l=q;outPTop=bResult;
				min=g_edge[g[p][q]].u-g_edge[g[p][q]].x;
			}
		}
	}else{
		if (bEqual){
			if (min>=g_edge[g[p][q]].x){
				k=p;l=q;outPTop=bResult;
				min=g_edge[g[p][q]].x;
			}
		}else{
			if (min>g_edge[g[p][q]].x){
				k=p;l=q;outPTop=bResult;
				min=g_edge[g[p][q]].x;
			}
		}
	}
	/*if (min<0)
		cout<<"ErrorMin";*/
}

bool calcCost(int& totalCost,int& totalNum,NodeEdge g_edge[],bool bTest){
	int tmpNum=0;
	for (int i=1;i<g_numVert+1;++i){
		if (g_edge[g_srcEdge[i]].x>0){
			totalCost+=g_costServ;
			tmpNum++;
		}
	}
	if (bTest&&tmpNum!=totalNum)
		return false;
	else{
		for (int i=0;i<g_edgeCount-g_numVert;++i){//0 is empty;*2-1
			if (g_edge[i].x>0){
				totalCost+=g_edge[i].x*g_edge[i].c;
			}
		}
		return true;
	}
}
void processNetwork(set<int>& pos,int& totalCost,NodeEdge g_edge[],bool bMakeRoute,int bBug){
	typedef pair<int,int> Pair;
	map<int,int> mapFlowID;
	if (bBug>0)
	for (int k=1;k<g_numDem+1;k++){
		int i=vCons[k].vid;
		if (pos.find(i)==pos.end()){//v[i].d>0
			int iMap=g_srcEdge[i];
			if (g_edge[iMap].x>0&&g_edge[iMap].x<v[i].d){
				int nFlow,nFlowLeft=v[i].d-g_edge[iMap].x;
				g_edge[iMap].x=v[i].d;
				do{
					nFlow=nFlowLeft;
					deleteFromNet(i,nFlow,pos,g_edge);
					nFlowLeft-=nFlow;
				}while(nFlowLeft!=0);
			}
			if (g_edge[iMap].x==0&&bBug==2){
				int nFlowLeft=v[i].d,nFlowCost=0,nFlow,nSrcID;
				mapFlowID.clear();
				do{
					nFlow=nFlowLeft;
					deleteAndCalcFromNet(i,nFlow,pos,nFlowCost,nSrcID,g_edge);
					nFlowLeft-=nFlow;
					auto tmpIter=mapFlowID.find(nSrcID);
					if (tmpIter==mapFlowID.end())
						mapFlowID.insert(Pair(nSrcID,nFlow));
					else
						(*tmpIter).second+=nFlow;
				}while (nFlowLeft!=0);
				if (nFlowCost<g_costServ){
					/*int nRealCost=nFlowCost;
					for (auto i=mapFlowID.begin();i!=mapFlowID.end();++i){
						if (g_edge[g_srcEdge[i->first]].x==i->second){
							nRealCost+=g_costServ;
							break;
						}
					}
					if (nRealCost>=g_costServ){
						g_edge[iMap].x=v[i].d;
						for (auto i=mapFlowID.begin();i!=mapFlowID.end();++i){
							g_edge[g_srcEdge[i->first]].x-=i->second;
						}
					}else*/
					totalCost+=nFlowCost;
				}else{
					g_edge[iMap].x=v[i].d;
					for (auto i=mapFlowID.begin();i!=mapFlowID.end();++i){
						g_edge[g_srcEdge[i->first]].x-=i->second;
					}
				}
			}
		}
	}
	if (bMakeRoute){
		pos.clear();
		for (int i=1;i<g_numVert+1;++i){
			if (g_edge[g_srcEdge[i]].x>0){
				pos.insert(g_edge[g_srcEdge[i]].idEnd);
			}
		}
	}
}
void deleteFromNet(int j,int& flow,set<int>& pos,NodeEdge g_edge[]){
	//for (int k=(int)v[j].idEdgeFrom.size()-2;k>=0;--k){//-1 means no 0j
	for (int k=0;k<(int)v[j].idEdgeFrom.size()-1;k++){//-1 means no 0j
		int ijMap=v[j].idEdgeFrom[k];
		if (g_edge[ijMap].x>0){
			int i=g_edge[ijMap].idBegin;
			if (flow>g_edge[ijMap].x)
				flow=g_edge[ijMap].x;
			if (pos.find(i)!=pos.end()){
				g_edge[g_srcEdge[i]].x-=flow;
			}else{
				deleteFromNet(i,flow,pos,g_edge);
			}
			g_edge[ijMap].x-=flow;
			break;
		}
	}
}
void deleteAndCalcFromNet(int j,int& flow,set<int>& pos,int& flowCost,int& srcID,NodeEdge g_edge[]){
	for (int k=0;k<(int)v[j].idEdgeFrom.size()-1;k++){//without 0j
		int ijMap=v[j].idEdgeFrom[k];
		if (g_edge[ijMap].x>0){
			int i=g_edge[ijMap].idBegin;
			if (flow>g_edge[ijMap].x)
				flow=g_edge[ijMap].x;
			if (pos.find(i)!=pos.end()){
				srcID=i;
			}else{
				deleteAndCalcFromNet(i,flow,pos,flowCost,srcID,g_edge);
			}
			g_edge[ijMap].x-=flow;
			flowCost+=g_edge[ijMap].c*flow;
			break;
		}
	}
}

void printPath(int i,int& flow,ostream& sout){
	for (int k=0;k<(int)v[i].idEdge.size();k++){
		int ijMap=v[i].idEdge[k];
		if (g_edge[ijMap].x>0){//&&g_edge[ijMap].u>0
			int j=g_edge[ijMap].idEnd;
			if (flow>g_edge[ijMap].x)
				flow=g_edge[ijMap].x;
			if (v[j].id>0&&v[j].d>0){
				if (flow>=v[j].d){
					flow=v[j].d;
					v[j].d=0;
					++g_leftCus;
				}else{
					v[j].d-=flow;
				}
				sout<<j-1<<" "<<v[j].id-1<<" "<<flow;
			}else{
				sout<<j-1<<" ";
				//int nTmp=g_edge[ijMap].u;
				//g_edge[ijMap].u=0;
				printPath(j,flow,sout);
				//g_edge[ijMap].u=nTmp;
			}
			g_edge[ijMap].x-=flow;
			g_totalCost+=g_edge[ijMap].c*flow;
			break;
		}
	}
}

void printTree(){
	int pSearch=0;
	do{
		cout<<pSearch<<" ;parent: "<<v[pSearch].parent<<" ;depth: "<<v[pSearch].depth;
		if (v[pSearch].directToParent){
			cout<<" X toParent="<<g_edge[g[pSearch][v[pSearch].parent]].x;
			cout<<" X fromParent="<<g_edge[g[v[pSearch].parent][pSearch]].x<<endl;
		}else{
			cout<<" X fromParent="<<g_edge[g[v[pSearch].parent][pSearch]].x;
			cout<<" X toParent="<<g_edge[g[pSearch][v[pSearch].parent]].x<<endl;
		}
		pSearch=v[pSearch].next;
	}
	while (pSearch!=0);
}

GenAlg::GenAlg(int bitSize,int popSize,int g_numDem,int g_costServ,set<int> greedyPos):m_bestFit(0),
	m_mutationRate((int)(RAND_MAX*0.2)),m_crossoverRate((int)(RAND_MAX*0.9)),//70%?80~95%?
	m_chromoBitSize(bitSize),//0.05~0.3?0.5~1%?
	m_cServer(g_costServ),m_maxFit(g_numDem*g_costServ),
	m_conCMax(10){//?
	m_popSize=0;
	/*int nSumD=bitSize;
	for (int i=1;i<g_numDem+1;++i){
		nSumD+=v[vCons[i].vid].d;
	}
	cout<<nSumD<<endl;*/
	m_pop.push_back(Genome(greedyPos));
	m_popSize++;
	while (m_popSize<popSize){
		set<int> setTmp;
		/*int k=rand();//*nSumD
		for (int i=1;i<bitSize+1;++i){
			//if (v[i].id>0){
			//	if (rand()*(v[i].d+1)>=k)
			//		setTmp.insert(i);
			if (rand()>=k)
				setTmp.insert(i);
			//if (greedyPos.find(i)!=greedyPos.end())
			//	if (rand()>=k)
			//		setTmp.insert(i);
		}*/
		for (int i=1;i<rand()%(bitSize+1)+1;++i){
			setTmp.insert(rand()%bitSize+1);
		}
		/*for (int i=1;i<rand()%(g_numDem+1)+1;++i){//rand()*g_numDem/RAND_MAX+1
			setTmp.insert(vCons[rand()%g_numDem+1].vid);
		}*/
		setTmp.insert(sFixedServerPos.begin(),sFixedServerPos.end());
		Genome tmpGenome(setTmp);
		m_pop.push_back(tmpGenome);
		m_popSize++;
		//tmpGenome.print();
	}
}
void GenAlg::calcFit(int timeS){
	m_totalFit=0;
	for (int i=0;i<m_popSize;++i){
		set<int> setTmp=m_pop[i].decodeToSet();
		networkSimplexAlg(setTmp,g_edge);
		int nTotalCost=0,nTotalNum=0;
		if (0){
			for (int i=0;i<g_edgeCount;++i){
				g_edgeTmp[i].x=g_edge[i].x;
			}
			processNetwork(setTmp,nTotalCost,g_edgeTmp,true);
			//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
			//nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(nTotalCost,nTotalNum,g_edgeTmp);//false
		}else{
			processNetwork(setTmp,nTotalCost,g_edge,true,0);
			networkSimplexAlg(setTmp,g_edge);
			nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(nTotalCost,nTotalNum,g_edge,false);
		}
		if (nTotalCost<m_maxFit){
			m_pop[i]=Genome(setTmp,m_maxFit-nTotalCost);
			//m_pop[i].m_fitness=m_maxFit-nTotalCost;
			if (m_pop[i].m_fitness>m_bestGenome.m_fitness){
				m_bestFit=m_pop[i].m_fitness;
				m_bestGenome=m_pop[i];
				m_convergCount=0;
			}
		}else
			m_pop[i].m_fitness=0;
		m_totalFit+=m_pop[i].m_fitness;
		if (clock()-g_tmStart>=timeS*CLOCKS_PER_SEC)
			break;
	}
}
Genome& GenAlg::getChromoRoulette(){
	double dSlice=rand()*m_totalFit/RAND_MAX;
	//Genome gChosen;
	double dNowFitSum=0;
	for (int i=0;i<(int)m_pop.size();++i){
		dNowFitSum+=m_pop[i].m_fitness;
		if (dNowFitSum>=dSlice){
			//gChosen=m_pop[i];
			return m_pop[i];
			//break;
		}
	}
}
void GenAlg::crossover(vector<int>& chromo1,vector<int>& chromo2,double crossoverRate,int type){
	if (rand()<crossoverRate){
		int pos=rand()%m_chromoBitSize+1;
		int pos2=rand()%m_chromoBitSize+1;
		if (type==0)
			pos2=0;
		if (pos<pos2){
			int nTmp=pos;
			pos=pos2;
			pos2=nTmp;
		}
		if ((int)chromo1.size()<pos/Genome::m_intBitNS+1)
			chromo1.resize(pos/Genome::m_intBitNS+1,0);
		if ((int)chromo2.size()<pos/Genome::m_intBitNS+1)
			chromo2.resize(pos/Genome::m_intBitNS+1,0);
		int i=pos2/Genome::m_intBitNS,nTmp;
		if (i<pos/Genome::m_intBitNS){
			nTmp=chromo1[i]&(0x1<<Genome::m_intBitNS)-(0x1<<pos2%Genome::m_intBitNS);
			chromo1[i]&=~((0x1<<Genome::m_intBitNS)-(0x1<<pos2%Genome::m_intBitNS));
			chromo1[i]|=chromo2[i]&(0x1<<Genome::m_intBitNS)-(0x1<<pos2%Genome::m_intBitNS);
			chromo2[i]&=~((0x1<<Genome::m_intBitNS)-(0x1<<pos2%Genome::m_intBitNS));
			chromo2[i]|=nTmp;
			for (i=i+1;i<pos/Genome::m_intBitNS;++i){
				int nTmp=chromo1[i];
				chromo1[i]=chromo2[i];
				chromo2[i]=nTmp;
			}
			nTmp=chromo1[i]&(0x1<<pos%Genome::m_intBitNS+1)-1;
			chromo1[i]&=~((0x1<<pos%Genome::m_intBitNS+1)-1);
			chromo1[i]|=chromo2[i]&(0x1<<pos%Genome::m_intBitNS+1)-1;
			chromo2[i]&=~((0x1<<pos%Genome::m_intBitNS+1)-1);
			chromo2[i]|=nTmp;
		}else{
			nTmp=chromo1[i]&(0x1<<pos%Genome::m_intBitNS+1)-(0x1<<pos2%Genome::m_intBitNS);
			chromo1[i]&=~((0x1<<pos%Genome::m_intBitNS+1)-(0x1<<pos2%Genome::m_intBitNS));
			chromo1[i]|=chromo2[i]&(0x1<<pos%Genome::m_intBitNS+1)-(0x1<<pos2%Genome::m_intBitNS);
			chromo2[i]&=~((0x1<<pos%Genome::m_intBitNS+1)-(0x1<<pos2%Genome::m_intBitNS));
			chromo2[i]|=nTmp;
		}
	}
}
void GenAlg::mutate(vector<int>& chromo){
	int i=rand()%m_chromoBitSize+1;
	if (rand()<m_mutationRate){
		if ((int)chromo.size()<i/Genome::m_intBitNS+1)
			chromo.resize(i/Genome::m_intBitNS+1,0);
		chromo[i/Genome::m_intBitNS]^=0x1<<i%Genome::m_intBitNS;
	}
}
void GenAlg::startGA(int timeS){
	vector<Genome> popNew;
	m_convergCount=0;
	while (m_convergCount<m_conCMax){
		popNew.clear();
		m_convergCount++;
		cout<<m_convergCount<<endl;
		calcFit(timeS);
		if (clock()-g_tmStart>=timeS*CLOCKS_PER_SEC)
			break;
		int i=0;
		while(i<m_popSize-1){
			Genome genomeF=getChromoRoulette();
			Genome genomeM=getChromoRoulette();
			crossover(genomeF.m_genome,genomeM.m_genome,m_crossoverRate);
			mutate(genomeF.m_genome);
			mutate(genomeM.m_genome);
			popNew.push_back(genomeF);
			popNew.push_back(genomeM);
			i+=2;
		}
		if (i==m_popSize-1){
			Genome genomeF=getChromoRoulette();
			mutate(genomeF.m_genome);
			popNew.push_back(genomeF);
		}
		for (i=0;i<m_popSize;++i){
			m_pop[i]=popNew[i];
		}
	}
}
void GenAlg::startPSO(int timeS,int w,int c1,int c2,int type){
	m_convergCount=0;
	/*for (int i=0;i<m_popSize;++i){
		if (m_pop[i].m_genome.size()<m_chromoBitSize/Genome::m_intBitNS+1)
			m_pop[i].m_genome.resize(m_chromoBitSize/Genome::m_intBitNS+1,0);
		vector<int> nV0;
		int k=0,nTmp=0;
		for (int j=1;j<m_chromoBitSize+1;j++){
			if (j/Genome::m_intBitNS>k){
				k=j/Genome::m_intBitNS;
				nV0.push_back(nTmp);
				nTmp=0;
			}
			nTmp|=rand()%2<<j%Genome::m_intBitNS;
		}
		nV0.push_back(nTmp);
		m_v.push_back(Genome(nV0));
	}*/
	m_pBest=m_pop;
	if (g_numVert>200)
		m_conCMax=200;
	while (m_convergCount<m_conCMax){
		m_convergCount++;
		cout<<m_convergCount<<endl;
		calcPGBest(timeS,type);
		if (clock()-g_tmStart>=timeS*CLOCKS_PER_SEC)
			break;
		int i=0;
		while(i<m_popSize){
			Genome genomeP=m_pBest[i],genomeG=m_bestGenome;
			if (rand()<w){
				int pos=rand()%m_chromoBitSize+1,pos2=rand()%m_chromoBitSize+1;
				if (pos<pos2){
					int nTmp=pos;
					pos=pos2;
					pos2=nTmp;
				}
				if ((int)m_pop[i].m_genome.size()<pos/Genome::m_intBitNS+1)
					m_pop[i].m_genome.resize(pos/Genome::m_intBitNS+1,0);
				int nTmp=m_pop[i].m_genome[pos/Genome::m_intBitNS]&0x1<<pos%Genome::m_intBitNS;
				m_pop[i].m_genome[pos/Genome::m_intBitNS]&=~(0x1<<pos%Genome::m_intBitNS);
				m_pop[i].m_genome[pos/Genome::m_intBitNS]|=m_pop[i].m_genome[pos2/Genome::m_intBitNS]&0x1<<pos2%Genome::m_intBitNS;
				m_pop[i].m_genome[pos2/Genome::m_intBitNS]&=~(0x1<<pos2%Genome::m_intBitNS);
				m_pop[i].m_genome[pos2/Genome::m_intBitNS]|=nTmp;
			}
			crossover(genomeP.m_genome,m_pop[i].m_genome,c1,0);
			if (rand()%2==0)
				m_pop[i]=genomeP;
			crossover(genomeG.m_genome,m_pop[i].m_genome,c2);
			if (rand()%2==0)
				m_pop[i]=genomeG;
			/*for (int j=0;j<m_chromoBitSize/Genome::m_intBitNS+1;j++){
				m_v[i].m_genome[j/Genome::m_intBitNS]=w*m_v[i].m_genome[j/Genome::m_intBitNS]+
					c1*rand()*1.0/RAND_MAX*(m_pBest[i].m_genome[j/Genome::m_intBitNS]-m_pop[i].m_genome[j/Genome::m_intBitNS])
					+c2*rand()*1.0/RAND_MAX*(m_bestGenome.m_genome[j/Genome::m_intBitNS]-m_pop[i].m_genome[j/Genome::m_intBitNS]);
				m_pop[i].m_genome[j/Genome::m_intBitNS]+=m_v[i].m_genome[j/Genome::m_intBitNS]*1;
			}
			m_pop[i].m_genome[m_chromoBitSize/Genome::m_intBitNS]&=~((0x1<<Genome::m_intBitNS)-(0x1<<m_chromoBitSize%Genome::m_intBitNS+1));*/
			++i;
		}
		//type=3;
		int loop=0;
		Genome genomeLS=m_bestGenome;
		do{
			Genome genomeL=genomeLS;
			int pos=rand()%m_chromoBitSize+1,pos2=rand()%m_chromoBitSize+1;
			if (pos<pos2){
				int nTmp=pos;
				pos=pos2;
				pos2=nTmp;
			}
			if (genomeL.m_genome.size()<pos/Genome::m_intBitNS+1)
				genomeL.m_genome.resize(pos/Genome::m_intBitNS+1,0);
			genomeL.m_genome[pos/Genome::m_intBitNS]^=0x1<<pos%Genome::m_intBitNS;
			genomeL.m_genome[pos2/Genome::m_intBitNS]^=0x1<<pos2%Genome::m_intBitNS;
			set<int> setTmp=genomeL.decodeToSet();
			int nTmTmp=clock();
			networkSimplexAlg(setTmp,g_edge);
			int nTotalCost=0,nTotalNum=0;
			if (type==1){
				for (int i=0;i<g_edgeCount;++i){
					g_edgeTmp[i].x=g_edge[i].x;
				}
				processNetwork(setTmp,nTotalCost,g_edgeTmp,true);
				g_count+=clock()-nTmTmp;
				calcCost(nTotalCost,nTotalNum,g_edgeTmp);//false
			}else{
				for (int k=1;k<g_numDem+1;k++){
					int i=vCons[k].vid;
					if (setTmp.find(i)==setTmp.end()){//v[i].d>0
						int iMap=g_srcEdge[i];
						if (g_edge[iMap].x>0){
							nTotalCost=~0u>>1;
							break;
						}
					}
				}
				if (nTotalCost!=~0u>>1){
					nTotalCost+=setTmp.size()*g_costServ;
					for (int i=0;i<g_edgeCount-g_numVert;++i){//0 is empty;*2-1
						if (g_edge[i].x>0){
							nTotalCost+=g_edge[i].x*g_edge[i].c;
						}
					}
				}
			}
			if (m_maxFit-nTotalCost>=genomeLS.m_fitness){
				genomeLS=Genome(setTmp,m_maxFit-nTotalCost);
			}else
				loop++;
		}
		while (loop<m_chromoBitSize);
		if (genomeLS.m_fitness>=m_bestGenome.m_fitness){
			if (genomeLS!=m_bestGenome){
				m_convergCount=0;
				m_bestGenome=genomeLS;
				m_bestFit=m_bestGenome.m_fitness;
			}
		}
		if (m_bestFit==m_maxFit-67623)
			break;
	}
}
void GenAlg::calcPGBest(int timeS,int type){
	for (int i=0;i<m_popSize;++i){
		set<int> setTmp=m_pop[i].decodeToSet();
		int nTmTmp=clock();
		networkSimplexAlg(setTmp,g_edge);
		int nTotalCost=0,nTotalNum=0;
		if (type==1){
			for (int i=0;i<g_edgeCount;++i){
				g_edgeTmp[i].x=g_edge[i].x;
			}
			processNetwork(setTmp,nTotalCost,g_edgeTmp,true);
			//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
			g_count+=clock()-nTmTmp;
			//nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(nTotalCost,nTotalNum,g_edgeTmp);//false
		}else if (type==0){
			processNetwork(setTmp,nTotalCost,g_edge,true,0);
			networkSimplexAlg(setTmp,g_edge);
			g_count+=clock()-nTmTmp;
			nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(nTotalCost,nTotalNum,g_edge,false);
		}else if (type==2){
			for (int i=0;i<g_edgeCount;++i){
				g_edgeTmp[i].x=g_edge[i].x;
			}
			processNetwork(setTmp,nTotalCost,g_edgeTmp,true,1);
			//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
			g_count+=clock()-nTmTmp;
			//nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(nTotalCost,nTotalNum,g_edgeTmp,false);
		}else{
			for (int k=1;k<g_numDem+1;k++){
				int i=vCons[k].vid;
				if (setTmp.find(i)==setTmp.end()){//v[i].d>0
					int iMap=g_srcEdge[i];
					if (g_edge[iMap].x>0){
						nTotalCost=~0u>>1;
						break;
					}
				}
			}
			if (nTotalCost!=~0u>>1){
				//calcCost(nTotalCost,nTotalNum,g_edge,false);
				nTotalCost+=setTmp.size()*g_costServ;
				for (int i=0;i<g_edgeCount-g_numVert;++i){//0 is empty;*2-1
					if (g_edge[i].x>0){
						nTotalCost+=g_edge[i].x*g_edge[i].c;
					}
				}
			}
		}
		if (nTotalCost<m_maxFit){
			m_pop[i]=Genome(setTmp,m_maxFit-nTotalCost);
			//m_pop[i].m_fitness=m_maxFit-nTotalCost;
			if (m_pop[i].m_fitness>m_pBest[i].m_fitness){
				m_pBest[i]=m_pop[i];
				if (m_pop[i].m_fitness>m_bestGenome.m_fitness){
					m_bestFit=m_pop[i].m_fitness;
					m_bestGenome=m_pop[i];
					m_convergCount=0;
				}
			}
		}else
			m_pop[i].m_fitness=0;
		if (clock()-g_tmStart>=timeS*CLOCKS_PER_SEC)
			break;
	}
}
void xjbs(){
	clock_t tmXjbStart=clock();
	set<int> setPos;
	int count=0;
	random_device rd;
	mt19937 mt(rd());
	uniform_int_distribution<int> dist(1,g_numVert);
	auto dice=bind(dist,mt);
	for (int i=0;i<1000;++i){
		int N=dice();
		setPos.clear();
		for (int j=0;j<N;j++){
			setPos.insert(dice());
		}
		//printf("xjbs: %d\n",i);
		netSAinit(g_edge);
		networkSimplexAlg(setPos,g_edge);
		//if (clock()-tmXjbStart>30*CLOCKS_PER_SEC)
		//	break;
	}
	cout<<"xjbs used time: "<<clock()-tmXjbStart<<endl;
}
Flow& randomwalk(){
	bitset<1010> dir_servers;
	for (int i=0;i<g_numVert;++i){
		if (v[i+1].id){
			dir_servers[i]=1;
		}
	}
	vector<dude> dudes;
	int mx_dude=7;//7;
	static int record[10000010];
	Flow* best_flow=new Flow;
	for (int i=0;1;++i){
		if (i==0){
			for (int i=0;i<mx_dude;++i){
				dude d;
				bitset<1010> ts=dir_servers;
				if (i!=0){
					ts[rand()%g_numVert].flip();
				}
				d.flow=new Flow(ts);
				dudes.push_back(d);
			}
		}
		for (int j=dudes.size()-1;j>=0;--j){
			dude d;
			bitset<1010> ts=dudes[j].flow->servers;
			int pos=rand()%g_numVert;
			while(ts[pos]==0&&rand()%3)
				pos=rand()%g_numVert;
			ts[pos].flip();
			d.flow=new Flow(ts);
			dudes.push_back(d);
		}
		sort(dudes.begin(),dudes.end());
		/*auto dupItr=unique(dudes.begin(),dudes.end());
		int delSize=dudes.end()-dupItr;
		cout<<delSize<<endl;
		while (delSize>0){
			delete dudes.back().flow;
			dudes.pop_back();
			--delSize;
		}*/
		while ((int)dudes.size()>mx_dude){
			delete dudes.back().flow;
			dudes.pop_back();
		}
		if (i%int(20000.0/g_numVert+1)==0){
			cout<<"Iteration "<<i+1<<": ";
            for(int i=0;i<min((int)dudes.size(),5);++i)
                cout<<dudes[i].flow->val()<<","<<dudes[i].flow->servers.count()<<" ";
            cout<<endl;
		}
		record[i]=dudes[0].flow->val();
		if (best_flow->val()>dudes[0].flow->val()){
			*best_flow=*dudes[0].flow;
			//cout<<best_flow->val()<<endl;
		}
		int gap=512;
		if (i-gap>=0&&record[i-gap]==record[i]){
			cout<<best_flow->val()<<endl;
			i=-1;
			while ((int)dudes.size()>0){
				delete dudes.back().flow;
				dudes.pop_back();
			}
			netSAinit(g_edge);
			//break;
		}
		if (clock()-g_tmStart>89*CLOCKS_PER_SEC){
			break;
		}
	}
	cout<<"Best solution: "<<best_flow->val()<<endl;
	for (int i=0;i<g_numVert;i++){
		if (best_flow->servers[i])
			cout<<" "<<i+1;
	}
	cout<<endl;
	return *best_flow;
}

typedef bitset<1010>facility;
struct xjb_dude{
    xjb_dude(facility f_){
        f=f_;
        v=Flow(f).val();
        age=0;
        contri=0;
    }
    facility f;
    int v;
    int age;
    xjb_dude*pr;
    int contri;
};
struct xjb_set{
    xjb_set(facility f_,int v_){
        f=f_;
        v=v_;
    }
    facility f;
    int v;
};

bool operator<(const xjb_set&a,const xjb_set&b){
    if(a.v!=b.v)
        return a.v<b.v;
    for(int i=0;i<g_numVert;++i)
        if(a.f[i]!=b.f[i])
            return a.f[i]<b.f[i];
    return false;
}

bool operator<(const xjb_dude&a,const xjb_dude&b){
    if(a.v!=b.v)
        return a.v<b.v;
    for(int i=0;i<g_numVert;++i)
        if(a.f[i]!=b.f[i])
            return a.f[i]<b.f[i];
    if(a.age!=b.age)
        return a.age>b.age;
    return false;
}

bool cmp(xjb_dude*a,xjb_dude*b){
    return *a<*b;
}

bool operator==(const xjb_dude&a,const xjb_dude&b){
    return a.f==b.f&&a.v==b.v;
}

bool operator!=(const xjb_dude&a,const xjb_dude&b){
    return !(a==b);
}

Flow xjb_search(){
    facility seed,best;
    set<xjb_set>hash;
    for (int i=0;i<g_numVert;++i)
        if (v[i+1].id)
            seed[i]=1;
    int z_seed=Flow(seed).val(),z_best,z_seed_before=z_seed;
    cout<<"Direct Seed: "<<z_seed<<endl;
    for(int changed=1,tm=0;changed;){
        changed=0;
        for(int i=0;i<g_numVert;++i){
            ++tm;
            seed[i].flip();
            int t=Flow(seed).val();
            if(t==~0u>>1){
                seed[i].flip();
                continue;
            }
            if(t<z_seed){
                z_seed=t;
                changed=1;
            }else{
                seed[i].flip();
            }
        }
    }
    cout<<"Greedy Seed: "<<z_seed<<endl;
    vector<xjb_dude*>dudes;
    dudes.push_back(new xjb_dude(seed));
    best=seed;
    z_best=z_seed;
    int max_qu=2;
    for(int it=0;;++it){
        for(int i=dudes.size()-1;i>=0;--i){
            facility f=dudes[i]->f;
            ++dudes[i]->age;
            int pos=rand()%g_numVert;
			while(f[pos]==0&&rand()%3){
                pos=rand()%g_numVert;
            }
            f[pos].flip();
            auto dude=new xjb_dude(f);
            if(!hash.count(xjb_set(dude->f,dude->v))){
                dudes.push_back(dude);
                dudes.back()->pr=dudes[i];
            }else{
                delete dude;
            }
        }
        sort(dudes.begin(),dudes.end(),cmp);
        vector<xjb_dude*>new_dudes;
        for(int i=0;i<dudes.size();++i){
            if(i==0||*dudes[i]!=*dudes[i-1])
                new_dudes.push_back(dudes[i]);
            else{
                delete dudes[i];
			}
        }
        for(int i=0;i<new_dudes.size()&&i<max_qu;++i)
            if(new_dudes[i]->age==0)
                ++new_dudes[i]->pr->contri;
        dudes.clear();
        for(int i=0;i<new_dudes.size();++i){
            if(new_dudes[i]->age>80){
                hash.insert(xjb_set(new_dudes[i]->f,new_dudes[i]->v));
                delete new_dudes[i];
            }else{
                dudes.push_back(new_dudes[i]);
            }
        }
        while(dudes.size()>max_qu){
            delete dudes.back();
            dudes.pop_back();
        }
        if(z_best>dudes[0]->v){
            z_best=dudes[0]->v;
            best=dudes[0]->f;
        }
        if (it%int(40000.0/g_numVert+1)==0){
            cout<<"Iteration "<<it+1<<": ";
            for(int i=0;i<min((int)dudes.size(),5);++i)
                cout<<dudes[i]->v<<","<<dudes[i]->f.count()<<","<<dudes[i]->age<<","<<dudes[i]->contri<<" ";
            cout<<endl;
			//cout<<hash.size()<<endl;
            //cout<<endl;
            //cout<<"Best solution: "<<z_best<<endl<<endl;
        }
        if (clock()-g_tmStart>87*CLOCKS_PER_SEC){
            break;
        }
		if (z_best==67623)
			break;
    }
    return Flow(best);
}
bool startSA(int nMinCost,set<int> nMinPos,set<int> nGreedyServerPos){

	return true;
}

bool startXjbs(int nMinCost,set<int> nMinPos,set<int> nGreedyServerPos){
	int edgeCost[MAX_NODE_NUM+1];
	for (int i=1;i<g_numVert+1;++i){
		if (v[i].id){
			edgeCost[i]=g_costServ/1;
		}
	}
	for (int i=1;i<g_numVert+1;++i){
		if (nMinPos.find(i)!=nMinPos.end()){
			g_edge[g_srcEdge[i]].c=0;
			//g_edge[g_srcEdge[i]].u=INT_MAX;
		}else{
			g_edge[g_srcEdge[i]].c=g_costServ;
			//g_edge[g_srcEdge[i]].u=0;
		}
	}
	networkSimplexAlg(nMinPos,g_edge);
	return true;
}