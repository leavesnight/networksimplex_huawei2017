#include "deploy.h"
#include <stdio.h>
#include <sstream>
#include <string>
#include <limits.h>
#include <ctime>
#include <map>
#include <algorithm>
//#include <iterator>
using namespace std;

const int MAX_NODE_NUM=1000,MAX_LINES_NUM=50000,MAX_CONSUME_NUM=500;
int g_count,g_leftCus,g_totalCost,g_count2;
clock_t g_tmStart=clock();

struct Node{
	int u;//upper bound, uij=[0,100];lower bound is lij=0
	int c;//cost, cij=[0,100]
	int x;//flow, xij
	int cpi;//Cπij, 4 answers from pi[2]&pj[2] but we can just use 2 types
	//[1]:cij-πi0+πj0 for LorU && [0]:cij-πi0+πjk(<0 then in) for L;[0]:cij-πik+πj0(>0 then in) for U
	//though [0] is smaller for L or bigger for U, it may cause a contradiction that x0j/x0i for L/U doesn't change from >0 to 0
	int idEdge;
};
struct NodeEdge{
	int u;//upper bound, uij=[0,100];lower bound is lij=0
	int c;//cost, cij=[0,100]
	int x;//flow, xij
	//int cpi;//Cπij
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
	vector<int> edge;
};
struct NodeConsumer{
	int vid;//record the id in the network,+1
};
Node g[MAX_NODE_NUM+1][MAX_NODE_NUM+1];//0 is the super point/ultimate source
NodeEdge g_edge[MAX_NODE_NUM*(40+1)];//max 20 edges connecting to one point, and add 0i;
int g_edgeCount;
Node gTmp[MAX_NODE_NUM+1][MAX_NODE_NUM+1];
NodeVertex v[MAX_NODE_NUM+1];
NodeConsumer vCons[MAX_CONSUME_NUM+1];
set<int> sFixedServerPos;


void networkSimplexAlg(int n,int cServer,set<int>& pos,Node g[][MAX_NODE_NUM+1],NodeVertex v[]);
void findOutgoing(int& k,int& l,bool& outPTop,bool bResult,
				  int& min,int p,int q,bool bEqual=false,bool directPlus=false);
void processNetwork(int n,set<int>& pos,int cServer,int& totalCost,Node g[][MAX_NODE_NUM+1],NodeVertex v[],
					bool bMakeRoute=false,bool bBug=true);
void deleteFromNet(int j,int& flow,int n,set<int>& pos,Node g[][MAX_NODE_NUM+1]);
void deleteAndCalcFromNet(int j,int& flow,int n,set<int>& pos,int& flowCost,int& srcID,Node g[][MAX_NODE_NUM+1]);
bool calcCost(int cServer,int n,int& totalCost,int& totalNum,Node g[][MAX_NODE_NUM+1],bool bTest=true);
void printPath(int i,int& flow,int n,ostream& sout);
void printTree();

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
	int n,m,nd,nTmp,nTmp2;
	string strTmp(topo[0]);
	nTmp=strTmp.find(' ');
	n=atoi(strTmp.substr(0,nTmp).c_str());//[0,1000]
	nTmp2=strTmp.find(' ',nTmp+1);
	m=atoi(strTmp.substr(nTmp+1,nTmp2-nTmp).c_str());//[0,1000*20]
	nd=atoi(strTmp.substr(nTmp2+1,strTmp.length()-nTmp2-1).c_str());//[0,500]
	//cout<<n<<endl<<m<<endl<<nd<<endl;
	int cServer=atoi(string(topo[2]).c_str());//[0,5000]

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
		if (g[nNum[0]+1][nNum[1]+1].u==0){
			g[nNum[0]+1][nNum[1]+1].u=nNum[2];//it starts from point 0, but we let it from 1
			g[nNum[1]+1][nNum[0]+1].u=nNum[2];//bidirectional
			v[nNum[0]+1].edge.push_back(nNum[1]+1);
			v[nNum[1]+1].edge.push_back(nNum[0]+1);
			g[nNum[0]+1][nNum[1]+1].c=nNum[3];
			g[nNum[1]+1][nNum[0]+1].c=nNum[3];
			g_edge[g_edgeCount].u=nNum[2];
			g_edge[g_edgeCount].c=nNum[3];
			g_edge[g_edgeCount].idBegin=nNum[0]+1;
			g_edge[g_edgeCount].idEnd=nNum[1]+1;
			g[nNum[0]+1][nNum[1]+1].idEdge=g_edgeCount;
			g_edgeCount++;
			g_edge[g_edgeCount].u=nNum[2];
			g_edge[g_edgeCount].c=nNum[3];
			g_edge[g_edgeCount].idBegin=nNum[1]+1;
			g_edge[g_edgeCount].idEnd=nNum[0]+1;
			g[nNum[1]+1][nNum[0]+1].idEdge=g_edgeCount;
			g_edgeCount++;
		}
	}
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
	}
	/*for (int i=0;i<n+1;i++){
		int nTmp=0,nTmp2=0;
		sort(v[i].edge.begin(),v[i].edge.end(),[i](int pos1,int pos2)->bool{
			if (g[i][pos1].c<g[i][pos2].c){
				return true;
			}else
				return false;
		});
		int nD=v[i].d;
		for (int j=0;j<v[i].edge.size();j++){
			nTmp+=g[i][v[i].edge[j]].u;
			if (nD>=g[i][v[i].edge[j]].u){
				nTmp2+=g[i][v[i].edge[j]].u*g[i][v[i].edge[j]].c;
				nD-=g[i][v[i].edge[j]].u;
			}else{
				nTmp2+=nD*g[i][v[i].edge[j]].c;
				nD=0;
			}
		}
		if (nTmp<v[i].d)
			cout<<i<<endl;
		if (nTmp2>=cServer){
			cout<<"GT cServer: "<<i<<endl;
			/*sFixedServerPos.insert(i);
			for (int j=0;j<v[i].edge.size();j++){
				g[i][v[i].edge[j]].u=0;
				g[v[i].edge[j]][i].u=0;
				for (int k=0;k<v[v[i].edge[j]].edge.size();k++){
					if (v[v[i].edge[j]].edge[k]==i){
						v[v[i].edge[j]].edge.erase(v[v[i].edge[j]].edge.begin()+k);
					}
				}
			}
			v[i].edge.clear();
		}
	}*/
	//make an initial of the networkSimplex
	//v[0].d=INT_MAX;//may not use v0.d
	for (int i=1;i<n+1;i++){
		g[0][i].u=INT_MAX;//super point,>=5000*500
		v[0].edge.push_back(i);
		g_edge[g_edgeCount].u=INT_MAX;
		g_edge[g_edgeCount].idBegin=0;
		g_edge[g_edgeCount].idEnd=i;
		g[0][i].idEdge=g_edgeCount;
		g_edgeCount++;
	}
	v[0].next=1;
	v[1].prec=0;
	for (int i=1;i<n+1;i++){
		if (v[i].id>0){//d maybe 0
			g[0][i].x=v[i].d;//the left xij is all 0, and the initial feasible tree <0,1~n> is strong for they're all away from 0
			g_edge[g_edgeCount-1-n+i].x=v[i].d;
		}//initial x is 0, no need for else
		v[i].parent=0;//all point's parent is 0 && v[0].depth=0
		v[i].directToParent=false;//v[i].direction=false cannot be omitted!!!
		v[i].depth=1;//v[0].depth=0, depth is from 0
		v[i].next=i+1;//i's next dfs node is i+1 except vn.next
		v[i+1].prec=i;
	}
	v[n].next=0;//!here is important
	v[0].prec=n;
	/*for (int i=0;i<n;i++){
		for (int j=0;j<n;j++){
			cout<<g[i][j].u<<" ";
		}
		cout<<endl;
	}
	for (int i=0;i<n;i++){
		cout<<v[i].d<<" ";
	}*/
	set<int> nServerPos,nGreedyServerPos;
	int nMinCost=nd*cServer,nMinPos[MAX_CONSUME_NUM]={0},nMinPosCount=0;
	/*int nMinCost=0,nMinPos[MAX_CONSUME_NUM]={106, 131, 165, 185, 276, 305, 310, 390, 391, 401, 428, 575, 585, 631, 655, 740},nMinPosCount=16;
	for (int i=0;i<nMinPosCount;i++){
		if (nMinPos[i]>0){
			nMinPos[i]++;
		}
	}
	//branchAndBound(n,cServer,nServerPos);
	nServerPos.clear();
	nServerPos.insert(nMinPos,nMinPos+nMinPosCount);
	int gmfCount=0;
	while (clock()-g_tmStart<30*CLOCKS_PER_SEC){
		networkSimplexAlg(n,cServer,nServerPos,g,v);
		gmfCount++;
	}
	cout<<"Oh My god! "<<gmfCount<<endl;*/
	/*for (int k=0;k<nd;k++){
		int min2Cost=INT_MAX,min2Pos=0,minPos=0;
		int nCostBefore=nMinCost;
		for (int i=1;i<n+1;i++){
			if (nGreedyServerPos.find(i)==nGreedyServerPos.end()){
				nServerPos.clear();
				nServerPos.insert(nGreedyServerPos.begin(),nGreedyServerPos.end());
				nServerPos.insert(i);
				networkSimplexAlg(n,cServer,nServerPos);
				int nTotalCost=0,nTotalNum=nMinPosCount;
				processNetwork(n,nServerPos,cServer,nTotalCost,false);
				//nTotalCost=0;
				calcCost(cServer,n,nTotalCost,nTotalNum);
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
			if (clock()-g_tmStart>40*CLOCKS_PER_SEC)
				break;
		}
		if (minPos>0){
			nGreedyServerPos.insert(minPos);
			nMinPos[nMinPosCount]=minPos;
			nMinPosCount++;
			cout<<nCostBefore<<" after greedy: "<<nMinCost<<endl;
			if (nCostBefore-nMinCost<cServer) break;
		}else{
			break;
			//nGreedyServerPos.insert(min2Pos);
		}
		if (clock()-g_tmStart>40*CLOCKS_PER_SEC)
			break;
	}
	/*nMinPosCount+=2;
	for (int i=1;i<n+1;i++){
		for (int i2=1;i2<n+1;i2++){
			if (i2!=i&&(nGreedyServerPos.find(i)==nGreedyServerPos.end()||
				nGreedyServerPos.find(i2)==nGreedyServerPos.end())){
				nServerPos.clear();
				nServerPos.insert(nMinPos,nMinPos+nMinPosCount-2);
				nServerPos.insert(i);
				nServerPos.insert(i2);
				networkSimplexAlg(n,cServer,nServerPos);
				int nTotalCost=0,nTotalNum=nMinPosCount;
				processNetwork(n,nServerPos,cServer,nTotalCost,false);
				//nTotalCost=0;
				calcCost(cServer,n,nTotalCost,nTotalNum);
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
	
	srand((int)time(0));
	int nPop,nType=1;
	if (n<500)
		nPop=500;
	else{
		nPop=200;
		nType=0;
	}
	GenAlg genAlg(n,nPop,nd,cServer,nGreedyServerPos);//20~30 chromosomes
	//genAlg.startGA(85);
	genAlg.startPSO(85,0.9*RAND_MAX,0.5*RAND_MAX,0.5*RAND_MAX,nType);
	//genAlg.startPSO(85,1,2,2);
	nMinCost=genAlg.getMinCost();
	nServerPos=genAlg.getBestServerPos();

	cout<<"use time: "<<(clock()-g_tmStart)*1000/CLOCKS_PER_SEC<<"ms"<<endl;

	cout<<"nMinCost="<<nMinCost<<" minPos=";
	/*ostream_iterator<int,char> out_iter(cout," ");
	copy(nMinServerPos.begin(),nMinServerPos.end(),out_iter);*/
	/*for_each(nMinServerPos.begin(),nMinServerPos.end(),[](int pos){
		cout<<" "<<pos;
	});*/
	for (auto i=nServerPos.begin();i!=nServerPos.end();i++){
		cout<<" "<<*i;
	}
	cout<<endl;
	for (int i=0;i<nMinPosCount;i++)
		cout<<" "<<nMinPos[i];
	cout<<endl;

	/*if (n==0&&nd>0)//customers' need exists while no nodes, impossible
		strTopo="NA";
	else{*/
	//printTree();
	ostringstream sout;
	int nLines=0;
	if (nMinCost<nd*cServer){
		/*nServerPos.clear();
		nServerPos.insert(nMinPos,nMinPos+nMinPosCount);*/
		networkSimplexAlg(n,cServer,nServerPos,g,v);
		int nTotalCost=0,nTotalNum=nMinPosCount;
		/*nTotalNum=nServerPos.size();
		while (!calcCost(cServer,n,nTotalCost,nTotalNum,g)){
			processNetwork(n,nServerPos,cServer,nTotalCost,g,v,true);
			nTotalNum=nServerPos.size();
			networkSimplexAlg(n,cServer,nServerPos,g,v);
			nTotalCost=0;
		}*/
		for (int i=0;i<n+1;i++){
			for (int j=0;j<n+1;j++){
				gTmp[i][j]=g[i][j];
			}
		}
		processNetwork(n,nServerPos,cServer,nTotalCost,gTmp,v,true);
		calcCost(cServer,n,nTotalCost,nTotalNum,gTmp);
		networkSimplexAlg(n,cServer,nServerPos,g,v);

		cout<<"Final nMinCost="<<nTotalCost<<" minPos=";
		for (auto i=nServerPos.begin();i!=nServerPos.end();i++)
			cout<<" "<<*i;
		cout<<endl;

		for (int i=1;i<n+1;i++){
			while (g[0][i].x>0){
				sout<<i-1<<" ";
				int nFlow=g[0][i].x;
				if (v[i].id>0&&v[i].d>0){
					if (nFlow>=v[i].d){
						nFlow=v[i].d;
						v[i].d=0;
						g_leftCus++;
					}else{
						v[i].d-=nFlow;
					}
					sout<<v[i].id-1<<" "<<nFlow;
				}else
					printPath(i,nFlow,n,sout);
				if (g_leftCus<nd) sout<<endl;
				nLines++;
				g[0][i].x-=nFlow;
				if (g[0][i].x==0){
					g_totalCost+=cServer;
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
		sout<<nd<<endl<<endl;//the most simple ans
		for (int line=m+5;line<line_num;line++){
			strTmp=topo[line];
			nTmp=strTmp.find(' ');
			nTmp2=strTmp.find(' ',nTmp+1);
			nNum[1]=atoi(strTmp.substr(nTmp+1,nTmp2-nTmp).c_str());
			nNum[2]=atoi(strTmp.substr(nTmp2+1,strTmp.length()-nTmp2-1).c_str());
			v[nNum[1]+1].d=nNum[2];//it starts from point 0, but we let it from 1, the left one is all 0 except v0.d
		}
		nTmp=0;
		for (int i=1;i<n+1;i++){
			if (v[i].id>0){
				if (++nTmp==nd)
					sout<<i-1<<" "<<v[i].id-1<<" "<<v[i].d;
				else
					sout<<i-1<<" "<<v[i].id-1<<" "<<v[i].d<<endl;
			}
		}
		strTopo=sout.str();
		g_totalCost=nd*cServer;
	}
	cout<<"Total cost: "<<g_totalCost<<endl;
	cout<<g_count<<endl<<"simplex num: "<<g_count2<<endl;
	strTopo=strTopo;
	cin.get();

	// 直接调用输出文件的方法输出到指定文件中(ps请注意格式的正确性，如果有解，第一行只有一个数据；第二行为空；第三行开始才是具体的数据，数据之间用一个空格分隔开)
	topo_file=strTopo.c_str();
	write_result(topo_file, filename);

}

void branchAndBound(int n,int cServer,set<int>& nServerPos){

}

void networkSimplexAlg(int n,int cServer,set<int>& pos,Node g[][MAX_NODE_NUM+1],NodeVertex v[]){
	int nTmp,nTmp2,nTmp3;
	g_count2++;
	//make an initial strongly feasible tree or solution
	/*for (int i=1;i<n+1;i++){
		for (int k=0;k<v[i].edge.size();k++){
			g[i][v[i].edge[k]].x=0;
		}
	}
	for (int i=0;i<g_edgeCount-n;i++){
		g_edge[i].x=0;
	}
	v[0].next=1;
	v[1].prec=0;*/
	for (int i=1;i<n+1;i++){
		if (pos.find(i)!=pos.end()){
			g[0][i].c=0;
			g_edge[g_edgeCount-1-n+i].c=0;
		}else{
			g[0][i].c=cServer;
			g_edge[g_edgeCount-1-n+i].c=cServer;
		}
		/*if (v[i].id>0){//d maybe 0
			g[0][i].x=v[i].d;//the left xij is all 0, and the initial feasible tree <0,1~n> is strong for they're all away from 0
			g_edge[g_edgeCount-1-n+i].x=v[i].d;
		}else{
			g[0][i].x=0;
			g_edge[g_edgeCount-1-n+i].x=0;
		}
		v[i].parent=0;//all point's parent is 0 && v[0].depth=0
		v[i].directToParent=false;//v[i].direction=false cannot be omitted!!!
		v[i].depth=1;//v[0].depth=0, depth is from 0
		v[i].next=i+1;//i's next dfs node is i+1 except vn.next
		v[i+1].prec=i;*/
	}
	/*v[n].next=0;//!here is important
	v[0].prec=n;
	//calculate π[i], let π0=0 or the potential of the root is 0
	for (int i=1;i<n+1;i++){//cπij=cij-πi+πj=0;here 0/cServer-0+πi=0
		v[i].pi=-g[0][i].c;
	}*/
	int d;
	for (int i=v[0].next;i!=0;i=v[i].next){
		if (v[i].parent==0){
			d=-g_edge[g_edgeCount-1-n+i].c-v[i].pi;//or g[0][i]
		}
		v[i].pi+=d;
	}
	//find the maximum residual reduced cost(-cπij/cπij for L/U)
	int max;
	/*for (int i=0;i<n+1;i++){
		for (int k=0;k<v[i].edge.size();k++){
			int j=v[i].edge[k];
			g[i][j].cpi=g[i][j].c-v[i].pi+v[j].pi;
		}
	}*/
	do{
		int p,q;
		max=0;
		/*for (int i=0;i<n+1;i++){//maybe cache can not save it leads to its slow speed!!!
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
		for (int i=0;i<g_edgeCount;i++){
			int ni=g_edge[i].idBegin;
			int nj=g_edge[i].idEnd;
			int nCpiij=g_edge[i].c-v[ni].pi+v[nj].pi;
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
		if (max!=0){//try entering the arc(p,q)-><p,q> & take out the arc<k,l>->(k,l)
			//find the outgoing arc<k,l>
			int k=p,l=q;//<p,q>∈W+ for L && ∈W- for U
			bool outPTop=true;//suppose <k,l> is in the way from p to top intersection (true includes <p,q> itself!)
			int topP=p,topQ=q,min=g[p][q].u;//<p,q>∈W+ for L,residual flow is u-x,here x=0;<p,q>∈W- for U,residual flow is x-0,here x=u;
			if (g[p][q].x==0){//L,W+ is <p,q>
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
				if (g[p][q].x==0){//L,W+ is <p,q>
				}else{
					bXpqL=false;//update bXpqL for later updating πi
				}
			}else{//min>0
				topP=p,topQ=q;
				if (g[p][q].x==0){//L,W+ is <p,q>
					while (topP!=topQ){
						if (v[topP].depth>v[topQ].depth){//go from P to top intersection
							if (v[topP].directToParent){//∈W-
								g[topP][v[topP].parent].x-=min;
								g_edge[g[topP][v[topP].parent].idEdge].x-=min;
							}else{//∈W+
								g[v[topP].parent][topP].x+=min;
								g_edge[g[v[topP].parent][topP].idEdge].x+=min;
							}
							topP=v[topP].parent;
						}else if (v[topP].depth<v[topQ].depth){
							if (v[topQ].directToParent){//∈W+
								g[topQ][v[topQ].parent].x+=min;
								g_edge[g[topQ][v[topQ].parent].idEdge].x+=min;
							}else{//∈W-
								g[v[topQ].parent][topQ].x-=min;
								g_edge[g[v[topQ].parent][topQ].idEdge].x-=min;
							}
							topQ=v[topQ].parent;
						}else{//v[topP].depth==v[topQ].depth
							//topP up to its parent
							if (v[topP].directToParent){//∈W-
								g[topP][v[topP].parent].x-=min;
								g_edge[g[topP][v[topP].parent].idEdge].x-=min;
							}else{//∈W+
								g[v[topP].parent][topP].x+=min;
								g_edge[g[v[topP].parent][topP].idEdge].x+=min;
							}
							topP=v[topP].parent;
							//topQ up to its parent
							if (v[topQ].directToParent){//∈W+
								g[topQ][v[topQ].parent].x+=min;
								g_edge[g[topQ][v[topQ].parent].idEdge].x+=min;
							}else{//∈W-
								g[v[topQ].parent][topQ].x-=min;
								g_edge[g[v[topQ].parent][topQ].idEdge].x-=min;
							}
							topQ=v[topQ].parent;
						}
					}
					g[p][q].x+=min;//update xpq
					g_edge[g[p][q].idEdge].x+=min;//update xpq
				}else{//U,W+ is <q,p>
					while (topP!=topQ){
						if (v[topP].depth>v[topQ].depth){
							if (v[topP].directToParent){//∈W+
								g[topP][v[topP].parent].x+=min;
								g_edge[g[topP][v[topP].parent].idEdge].x+=min;
							}else{//∈W-
								g[v[topP].parent][topP].x-=min;
								g_edge[g[v[topP].parent][topP].idEdge].x-=min;
							}
							topP=v[topP].parent;
						}else if (v[topP].depth<v[topQ].depth){//go from Q to top intersection
							if (v[topQ].directToParent){//∈W-
								g[topQ][v[topQ].parent].x-=min;
								g_edge[g[topQ][v[topQ].parent].idEdge].x-=min;
							}else{//∈W+
								g[v[topQ].parent][topQ].x+=min;
								g_edge[g[v[topQ].parent][topQ].idEdge].x+=min;
							}
							topQ=v[topQ].parent;
						}else{//v[topP].depth==v[topQ].depth
							//topP up to its parent
							if (v[topP].directToParent){//∈W+
								g[topP][v[topP].parent].x+=min;
								g_edge[g[topP][v[topP].parent].idEdge].x+=min;
							}else{//∈W-
								g[v[topP].parent][topP].x-=min;
								g_edge[g[v[topP].parent][topP].idEdge].x-=min;
							}
							topP=v[topP].parent;
							//topQ up to its parent
							if (v[topQ].directToParent){//∈W-
								g[topQ][v[topQ].parent].x-=min;
								g_edge[g[topQ][v[topQ].parent].idEdge].x-=min;
							}else{//∈W+
								g[v[topQ].parent][topQ].x+=min;
								g_edge[g[v[topQ].parent][topQ].idEdge].x+=min;
							}
							topQ=v[topQ].parent;
						}
					}
					g[p][q].x-=min;//update xpq
					g_edge[g[p][q].idEdge].x-=min;//update xpq
					bXpqL=false;//update bXpqL for later updating πi
				}
			}
			//update the tree T=T1+<p,q>or<q,p>+T2
			if (k==p&&l==q){//don't update the tree structure&&πi;it's important to classify this situation for there's no concept of a higher node
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
					if (num!=n+1){
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
				pSearch=nPQL;
				v[pSearch].depth=v[v[pSearch].parent].depth+1;
				if (nPQH==p&&bXpqL||nPQH==q&&!bXpqL){
					v[pSearch].pi+=max;//-=-max because cπnPQHnPQL=cπpq for L=-cπpq for U<0 && max always>0
				}else{
					v[pSearch].pi-=max;//vice versa
				}
				/*for (int i=0;i<v[pSearch].edge.size();i++){
					int j=v[pSearch].edge[i];
					g[pSearch][j].cpi=g[pSearch][j].c-v[pSearch].pi+v[j].pi;
					g[j][pSearch].cpi=g[j][pSearch].c-v[j].pi+v[pSearch].pi;
				}
				g[0][pSearch].cpi=g[0][pSearch].c-v[0].pi+v[pSearch].pi;*/
				while (v[pSearch].next!=nTmp){//traverse the T2;nTmp is released here for label
					pSearch=v[pSearch].next;
					v[pSearch].depth=v[v[pSearch].parent].depth+1;
					if (nPQH==p&&bXpqL||nPQH==q&&!bXpqL){
						v[pSearch].pi+=max;//-=-max because cπnPQHnPQL=cπpq for L=-cπpq for U<0 && max always>0
					}else{
						v[pSearch].pi-=max;//vice versa
					}
					/*for (int i=0;i<v[pSearch].edge.size();i++){
						int j=v[pSearch].edge[i];
						g[pSearch][j].cpi=g[pSearch][j].c-v[pSearch].pi+v[j].pi;
						g[j][pSearch].cpi=g[j][pSearch].c-v[j].pi+v[pSearch].pi;
					}
					g[0][pSearch].cpi=g[0][pSearch].c-v[0].pi+v[pSearch].pi;*/
				}
			}
		}
	}while (max!=0);
}

void findOutgoing(int& k,int& l,bool& outPTop,bool bResult,
				  int& min,int p,int q,bool bEqual,bool directPlus){
	if (directPlus){
		if (bEqual){
			if (min>=g[p][q].u-g[p][q].x){
				k=p;l=q;outPTop=bResult;
				min=g[p][q].u-g[p][q].x;
			}
		}else{
			if (min>g[p][q].u-g[p][q].x){
				k=p;l=q;outPTop=bResult;
				min=g[p][q].u-g[p][q].x;
			}
		}
	}else{
		if (bEqual){
			if (min>=g[p][q].x){
				k=p;l=q;outPTop=bResult;
				min=g[k][l].x;
			}
		}else{
			if (min>g[p][q].x){
				k=p;l=q;outPTop=bResult;
				min=g[k][l].x;
			}
		}
	}
	/*if (min<0)
		cout<<"ErrorMin";*/
}

bool calcCost(int cServer,int n,int& totalCost,int& totalNum,Node g[][MAX_NODE_NUM+1],bool bTest){
	int tmpNum=0;
	for (int i=1;i<n+1;i++){
		if (g[0][i].x>0){
			totalCost+=cServer;
			tmpNum++;
		}
	}
	if (bTest&&tmpNum!=totalNum)
		return false;
	else{
		for (int i=1;i<n+1;i++){
			for (int j=1;j<n+1;j++){
				if (g[i][j].x>0){
					totalCost+=g[i][j].x*g[i][j].c;
				}
			}
		}
		return true;
	}
}

void processNetwork(int n,set<int>& pos,int cServer,int& totalCost,Node g[][MAX_NODE_NUM+1],NodeVertex v[],
					bool bMakeRoute,bool bBug){
	typedef pair<int,int> Pair;
	map<int,int> mapFlowID;
	if (bBug)
	for (int i=1;i<n+1;i++){
		if (v[i].d>0&&pos.find(i)==pos.end()){
			if (g[0][i].x>0&&g[0][i].x<v[i].d){
				int nFlow,nFlowLeft=v[i].d-g[0][i].x;
				g[0][i].x=v[i].d;
				do{
					nFlow=nFlowLeft;
					deleteFromNet(i,nFlow,n,pos,g);
					nFlowLeft-=nFlow;
				}while(nFlowLeft!=0);
			}
			if (g[0][i].x==0&&bBug){
				int nFlowLeft=v[i].d,nFlowCost=0,nFlow,nSrcID;
				mapFlowID.clear();
				do{
					nFlow=nFlowLeft;
					deleteAndCalcFromNet(i,nFlow,n,pos,nFlowCost,nSrcID,g);
					nFlowLeft-=nFlow;
					auto tmpIter=mapFlowID.find(nSrcID);
					if (tmpIter==mapFlowID.end())
						mapFlowID.insert(Pair(nSrcID,nFlow));
					else
						(*tmpIter).second+=nFlow;
				}while (nFlowLeft!=0);
				if (nFlowCost<cServer){
					totalCost+=nFlowCost;
				}else{
					g[0][i].x=v[i].d;
					for (auto i=mapFlowID.begin();i!=mapFlowID.end();i++){
						g[0][i->first].x-=i->second;
					}
				}
			}
		}
	}
	if (bMakeRoute){
		pos.clear();
		for (int i=1;i<n+1;i++){
			if (g[0][i].x>0){
				pos.insert(i);
			}
		}
	}
}
void deleteFromNet(int j,int& flow,int n,set<int>& pos,Node g[][MAX_NODE_NUM+1]){
	for (int i=1;i<n+1;i++){
		if (g[i][j].x>0){
			if (flow>g[i][j].x)
				flow=g[i][j].x;
			if (pos.find(i)!=pos.end()){
				g[0][i].x-=flow;
			}else{
				deleteFromNet(i,flow,n,pos,g);
			}
			g[i][j].x-=flow;
			break;
		}
	}
}
void deleteAndCalcFromNet(int j,int& flow,int n,set<int>& pos,int& flowCost,int& srcID,Node g[][MAX_NODE_NUM+1]){
	for (int i=1;i<n+1;i++){
		if (g[i][j].x>0){
			if (flow>g[i][j].x)
				flow=g[i][j].x;
			if (pos.find(i)!=pos.end()){
				//g[0][i].x-=flow;
				srcID=i;
			}else{
				deleteAndCalcFromNet(i,flow,n,pos,flowCost,srcID,g);
			}
			g[i][j].x-=flow;
			flowCost+=g[i][j].c*flow;
			break;
		}
	}
}

void printPath(int i,int& flow,int n,ostream& sout){
	for (int j=1;j<n+1;j++){
		if (g[i][j].x>0){//&&g[i][j].u>0
			if (flow>g[i][j].x)
				flow=g[i][j].x;
			if (v[j].id>0&&v[j].d>0){
				if (flow>=v[j].d){
					flow=v[j].d;
					v[j].d=0;
					g_leftCus++;
				}else{
					v[j].d-=flow;
				}
				sout<<j-1<<" "<<v[j].id-1<<" "<<flow;
			}else{
				sout<<j-1<<" ";
				//int nTmp=g[i][j].u;
				//g[i][j].u=0;
				printPath(j,flow,n,sout);
				//g[i][j].u=nTmp;
			}
			g[i][j].x-=flow;
			g_totalCost+=g[i][j].c*flow;
			break;
		}
	}
}

void printTree(){
	int pSearch=0;
	do{
		cout<<pSearch<<" ;parent: "<<v[pSearch].parent<<" ;depth: "<<v[pSearch].depth;
		if (v[pSearch].directToParent){
			cout<<" X toParent="<<g[pSearch][v[pSearch].parent].x;
			cout<<" X fromParent="<<g[v[pSearch].parent][pSearch].x<<endl;
		}else{
			cout<<" X fromParent="<<g[v[pSearch].parent][pSearch].x;
			cout<<" X toParent="<<g[pSearch][v[pSearch].parent].x<<endl;
		}
		pSearch=v[pSearch].next;
	}
	while (v[pSearch].next!=0);
}

GenAlg::GenAlg(int bitSize,int popSize,int nd,int cServer,set<int> greedyPos):m_bestFit(0),
	m_mutationRate(RAND_MAX*0.2),m_crossoverRate(RAND_MAX*0.9),//70%?80~95%?
	m_chromoBitSize(bitSize),//0.05~0.3?0.5~1%?
	m_cServer(cServer),m_maxFit(nd*cServer),
	m_conCMax(10){//?
	m_popSize=0;
	/*int nSumD=bitSize;
	for (int i=1;i<nd+1;i++){
		nSumD+=v[vCons[i].vid].d;
	}
	cout<<nSumD<<endl;*/
	while (m_popSize<popSize){
		set<int> setTmp;
		/*int k=rand();//*nSumD
		for (int i=1;i<bitSize+1;i++){
			//if (v[i].id>0){
			//	if (rand()*(v[i].d+1)>=k)
			//		setTmp.insert(i);
			if (rand()>=k)
				setTmp.insert(i);
			//if (greedyPos.find(i)!=greedyPos.end())
			//	if (rand()>=k)
			//		setTmp.insert(i);
		}*/
		for (int i=1;i<rand()%(bitSize+1)+1;i++){
			setTmp.insert(rand()%bitSize+1);
		}
		/*for (int i=1;i<rand()%(nd+1)+1;i++){//rand()*nd/RAND_MAX+1
			setTmp.insert(vCons[rand()%nd+1].vid);
		}*/
		setTmp.insert(sFixedServerPos.begin(),sFixedServerPos.end());
		Genome tmpGenome(setTmp);
		m_pop.push_back(tmpGenome);
		m_popSize++;
		//tmpGenome.print();
	}
	//m_pop.push_back(Genome(greedyPos));
	//m_popSize++;
}
void GenAlg::calcFit(int timeS){
	m_totalFit=0;
	for (int i=0;i<m_popSize;i++){
		set<int> setTmp=m_pop[i].decodeToSet();
		networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
		int nTotalCost=0,nTotalNum=0;
		processNetwork(m_chromoBitSize,setTmp,m_cServer,nTotalCost,g,v,true);
		//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp);
		//nTotalCost=0;nTotalNum=setTmp.size();
		calcCost(m_cServer,m_chromoBitSize,nTotalCost,nTotalNum,g);//,false);
		if (nTotalCost<m_maxFit){
			m_pop[i]=Genome(setTmp,m_maxFit-nTotalCost);
			//m_pop[i].m_fitness=m_maxFit-nTotalCost;
			if (m_pop[i].m_fitness>m_bestFit){
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
	for (int i=0;i<m_pop.size();i++){
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
		if (chromo1.size()<pos/Genome::m_intBitNS+1)
			chromo1.resize(pos/Genome::m_intBitNS+1,0);
		if (chromo2.size()<pos/Genome::m_intBitNS+1)
			chromo2.resize(pos/Genome::m_intBitNS+1,0);
		int i=pos2/Genome::m_intBitNS,nTmp;
		if (i<pos/Genome::m_intBitNS){
			nTmp=chromo1[i]&(0x1<<Genome::m_intBitNS)-(0x1<<pos2%Genome::m_intBitNS);
			chromo1[i]&=~((0x1<<Genome::m_intBitNS)-(0x1<<pos2%Genome::m_intBitNS));
			chromo1[i]|=chromo2[i]&(0x1<<Genome::m_intBitNS)-(0x1<<pos2%Genome::m_intBitNS);
			chromo2[i]&=~((0x1<<Genome::m_intBitNS)-(0x1<<pos2%Genome::m_intBitNS));
			chromo2[i]|=nTmp;
			for (i=i+1;i<pos/Genome::m_intBitNS;i++){
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
		if (chromo.size()<i/Genome::m_intBitNS+1)
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
		for (i=0;i<m_popSize;i++){
			m_pop[i]=popNew[i];
		}
	}
}
void GenAlg::startPSO(int timeS,int w,int c1,int c2,int type){
	m_convergCount=0;
	/*for (int i=0;i<m_popSize;i++){
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
				if (m_pop[i].m_genome.size()<pos/Genome::m_intBitNS+1)
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
			i++;
		}
		/*int loop=0;
		Genome genomeLS=m_bestGenome;
		do{
			int pos=rand()%m_chromoBitSize+1,pos2=rand()%m_chromoBitSize+1;
			if (pos<pos2){
				int nTmp=pos;
				pos=pos2;
				pos2=nTmp;
			}
			if (genomeLS.m_genome.size()<pos/Genome::m_intBitNS+1)
				genomeLS.m_genome.resize(pos/Genome::m_intBitNS+1,0);
			genomeLS.m_genome[pos/Genome::m_intBitNS]^=0x1<<pos%Genome::m_intBitNS;
			genomeLS.m_genome[pos/Genome::m_intBitNS]^=0x1<<pos2%Genome::m_intBitNS;
			set<int> setTmp=genomeLS.decodeToSet();
			networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp);
			int nTotalCost=0,nTotalNum=0;
			processNetwork(m_chromoBitSize,setTmp,m_cServer,nTotalCost,true);
			calcCost(m_cServer,m_chromoBitSize,nTotalCost,nTotalNum);
			if (m_maxFit-nTotalCost>genomeLS.m_fitness){
				genomeLS=Genome(setTmp,m_maxFit-nTotalCost);
			}else
				loop++;
		}
		while (loop<m_convergCount/2);
		if (genomeLS.m_fitness>m_bestGenome.m_fitness){
			m_bestGenome=genomeLS;
		}*/
	}
}
void GenAlg::calcPGBest(int timeS,int type){
	for (int i=0;i<m_popSize;i++){
		set<int> setTmp=m_pop[i].decodeToSet();
		int nTmTmp=clock();
		networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
		int nTotalCost=0,nTotalNum=0;
		if (type==1){
			for (int i=0;i<m_chromoBitSize+1;i++){
				for (int j=0;j<m_chromoBitSize+1;j++){
					gTmp[i][j]=g[i][j];
				}
			}
			processNetwork(m_chromoBitSize,setTmp,m_cServer,nTotalCost,gTmp,v,true);
			//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
			g_count+=clock()-nTmTmp;
			//nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(m_cServer,m_chromoBitSize,nTotalCost,nTotalNum,gTmp);//false
		}else{
			processNetwork(m_chromoBitSize,setTmp,m_cServer,nTotalCost,g,v,true,false);
			networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
			g_count+=clock()-nTmTmp;
			nTotalCost=0;nTotalNum=setTmp.size();
			calcCost(m_cServer,m_chromoBitSize,nTotalCost,nTotalNum,g,false);
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