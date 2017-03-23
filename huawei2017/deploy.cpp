#include "deploy.h"
#include <stdio.h>
#include <sstream>
#include <string>
#include <limits.h>
#include <ctime>
#include <map>
//#include <algorithm>
//#include <iterator>
using namespace std;

const int MAX_NODE_NUM=1000,MAX_LINES_NUM=50000,MAX_CONSUME_NUM=500;
int g_count,g_leftCus,g_totalCost;
clock_t g_tmStart=clock();

struct Node{
	int u;//upper bound, uij=[0,100];lower bound is lij=0
	int c;//cost, cij=[0,100]
	int x;//flow, xij
	int cpi;//Cπij, 4 answers from pi[2]&pj[2] but we can just use 2 types
	//[1]:cij-πi0+πj0 for LorU && [0]:cij-πi0+πjk(<0 then in) for L;[0]:cij-πik+πj0(>0 then in) for U
	//though [0] is smaller for L or bigger for U, it may cause a contradiction that x0j/x0i for L/U doesn't change from >0 to 0
};
struct NodeVertex{//can store the NodeTree data
	int d;//demand, di=[0,5000]
	int id;//customer's id, πid
	//NodeTree
	int parent;bool directToParent;//true means the arc is pointing to the parent else it's pointing from the parent
	int depth;
	int next;//deep first search
	int pi;//node potential//two types for the step function from the ultimate source, πi[2]
	vector<int> edge;
};
struct NodeConsumer{
	int vid;//record the id in the network,+1
};
Node g[MAX_NODE_NUM+1][MAX_NODE_NUM+1];//0 is the super point/ultimate source
NodeVertex v[MAX_NODE_NUM+1];
NodeConsumer vCons[MAX_CONSUME_NUM+1];


void networkSimplexAlg(int n,int cServer,set<int>& pos);
void findOutgoing(int& k,int& l,bool& outPTop,bool bResult,
				  int& min,int p,int q,bool bEqual=false,bool directPlus=false);
void processNetwork(int n,set<int>& pos,int cServer,int& totalCost,bool bMakeRoute=false,bool bBug=true);
void deleteFromNet(int j,int& flow,int n,set<int>& pos);
void deleteAndCalcFromNet(int j,int& flow,int n,set<int>& pos,int& flowCost,int& srcID);
bool calcCost(int cServer,int n,int& totalCost,int& totalNum,bool bTest=true);
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
		g[nNum[0]+1][nNum[1]+1].u=nNum[2];//it starts from point 0, but we let it from 1
		g[nNum[1]+1][nNum[0]+1].u=nNum[2];//bidirectional
		v[nNum[0]+1].edge.push_back(nNum[1]+1);
		v[nNum[1]+1].edge.push_back(nNum[0]+1);
		g[nNum[0]+1][nNum[1]+1].c=nNum[3];
		g[nNum[1]+1][nNum[0]+1].c=nNum[3];
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
	//make an initial of the networkSimplex
	//v[0].d=INT_MAX;//may not use v0.d
	for (int i=1;i<n+1;i++){
		g[0][i].u=INT_MAX;//super point,>=5000*500
		v[0].edge.push_back(i);
	}
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
	//branchAndBound(n,cServer,nServerPos);
	/*for (int k=0;k<nd;k++){
		int min2Cost=INT_MAX,min2Pos=0,minPos=0;
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
			if (clock()-g_tmStart>85*CLOCKS_PER_SEC)
				break;
		}
		if (minPos>0){
			nGreedyServerPos.insert(minPos);
			nMinPos[nMinPosCount]=minPos;
			nMinPosCount++;
		}else{
			break;
			//nGreedyServerPos.insert(min2Pos);
		}
		if (clock()-g_tmStart>85*CLOCKS_PER_SEC)
			break;
	}
	nMinPosCount+=2;
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
	int nPop;
	if (n<500)
		nPop=n*2;
	else
		nPop=n*2;
	GenAlg genAlg(n,nPop,nd,cServer);//20~30 chromosomes
	genAlg.startGA(85);
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
		networkSimplexAlg(n,cServer,nServerPos);
		int nTotalCost=0,nTotalNum=nMinPosCount;
		/*nTotalNum=nServerPos.size();
		while (!calcCost(cServer,n,nTotalCost,nTotalNum)){
			processNetwork(n,nServerPos,cServer,nTotalCost,true);
			nTotalNum=nServerPos.size();
			networkSimplexAlg(n,cServer,nServerPos);
			nTotalCost=0;
		}*/
		processNetwork(n,nServerPos,cServer,nTotalCost,true);
		calcCost(cServer,n,nTotalCost,nTotalNum);
		networkSimplexAlg(n,cServer,nServerPos);

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
	//cout<<g_count<<endl;
	strTopo=strTopo;
	cin.get();

	// 直接调用输出文件的方法输出到指定文件中(ps请注意格式的正确性，如果有解，第一行只有一个数据；第二行为空；第三行开始才是具体的数据，数据之间用一个空格分隔开)
	topo_file=strTopo.c_str();
	write_result(topo_file, filename);

}

void branchAndBound(int n,int cServer,set<int>& nServerPos){

}

void networkSimplexAlg(int n,int cServer,set<int>& pos){
	int nTmp,nTmp2,nTmp3;
	//g_count++;
	//make an initial strongly feasible tree or solution
	for (int i=0;i<n+1;i++)
		for (int k=0;k<v[i].edge.size();k++){
			g[i][v[i].edge[k]].x=0;
		}
	v[0].next=1;//may not use v0.d
	for (int i=1;i<n+1;i++){
		if (pos.find(i)!=pos.end())
			g[0][i].c=0;
		else
			g[0][i].c=cServer;
		if (v[i].id>0)//d maybe 0
			g[0][i].x=v[i].d;//the left xij is all 0, and the initial feasible tree <0,1~n> is strong for they're all away from 0
		else
			g[0][i].x=0;
		v[i].parent=0;//all point's parent is 0 && v[0].depth=0
		v[i].directToParent=false;//v[i].direction=false cannot be omitted!!!
		v[i].depth=1;//v[0].depth=0, depth is from 0
		v[i].next=i+1;//i's next dfs node is i+1 except vn.next
	}
	v[n].next=0;//!here is important

	//calculate π[i], let π0=0 or the potential of the root is 0
	for (int i=1;i<n+1;i++){//cπij=cij-πi+πj=0;here 0/cServer-0+πi=0
		v[i].pi=-g[0][i].c;
	}
	//find the maximum residual reduced cost(-cπij/cπij for L/U)
	int max;
	/*for (int i=0;i<n+1;i++){
		for (int k=0;k<v[i].edge.size();k++){
			int j=v[i].edge[k];
			g[i][j].cpi=g[i][j].c-v[i].pi+v[j].pi;
		}
	}*/
	unsigned char bMayWrong=0;//if cπij uses one πi1 then if x<0,πIDi>!=0 will be a wrong trial;0false 1true 2may be correct
	do{
		int p,q;
		max=0;
		for (int i=0;i<n+1;i++){
			for (int k=0;k<v[i].edge.size();k++){
				int j=v[i].edge[k];
				if (!(!v[j].directToParent&&v[j].parent==i||v[i].directToParent&&v[i].parent==j)){//if the arc exists && it !∈ T;i!=j belongs to .u>0
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
							}else{//∈W+
								g[v[topP].parent][topP].x+=min;
							}
							topP=v[topP].parent;
						}else if (v[topP].depth<v[topQ].depth){
							if (v[topQ].directToParent){//∈W+
								g[topQ][v[topQ].parent].x+=min;
							}else{//∈W-
								g[v[topQ].parent][topQ].x-=min;
							}
							topQ=v[topQ].parent;
						}else{//v[topP].depth==v[topQ].depth
							//topP up to its parent
							if (v[topP].directToParent){//∈W-
								g[topP][v[topP].parent].x-=min;
							}else{//∈W+
								g[v[topP].parent][topP].x+=min;
							}
							topP=v[topP].parent;
							//topQ up to its parent
							if (v[topQ].directToParent){//∈W+
								g[topQ][v[topQ].parent].x+=min;
							}else{//∈W-
								g[v[topQ].parent][topQ].x-=min;
							}
							topQ=v[topQ].parent;
						}
					}
					g[p][q].x+=min;//update xpq
				}else{//U,W+ is <q,p>
					while (topP!=topQ){
						if (v[topP].depth>v[topQ].depth){
							if (v[topP].directToParent){//∈W+
								g[topP][v[topP].parent].x+=min;
							}else{//∈W-
								g[v[topP].parent][topP].x-=min;
							}
							topP=v[topP].parent;
						}else if (v[topP].depth<v[topQ].depth){//go from Q to top intersection
							if (v[topQ].directToParent){//∈W-
								g[topQ][v[topQ].parent].x-=min;
							}else{//∈W+
								g[v[topQ].parent][topQ].x+=min;
							}
							topQ=v[topQ].parent;
						}else{//v[topP].depth==v[topQ].depth
							//topP up to its parent
							if (v[topP].directToParent){//∈W+
								g[topP][v[topP].parent].x+=min;
							}else{//∈W-
								g[v[topP].parent][topP].x-=min;
							}
							topP=v[topP].parent;
							//topQ up to its parent
							if (v[topQ].directToParent){//∈W-
								g[topQ][v[topQ].parent].x-=min;
							}else{//∈W+
								g[v[topQ].parent][topQ].x+=min;
							}
							topQ=v[topQ].parent;
						}
					}
					g[p][q].x-=min;//update xpq
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
					//shift nTmp4,nTmp5->nTmp2,nTmp4 then nTmp5 is released
					nTmp2=nTmp4;nTmp4=nTmp5;
				}
				nTmp3=v[pSearch].next;//save it for final update next
				if (nTmp!=nL){
					v[pSearch].next=nTmp;//next is nTmp
					//only old tree from the root nL goes out
					if (nH==nPQH) 
						pSearch=nTmp;
					else
						pSearch=nH;
					while (v[pSearch].next!=nL){//now <nH,nL> is nonexistent
						if (v[pSearch].next==nPQH)//if encounter the <nPQH,nPQL>+T2 then jump it
							pSearch=nTmp;
						else
							pSearch=v[pSearch].next;
					}
					v[pSearch].next=nTmp3;//nTmp3 is released
				}else{//if (nTmp==nL) then topP==nH! if opposite the condition it's not always true!
					//v[pSearch].next=nTmp3;//nTmp3 is released && don't need to update anything including v[nH].next....next=nL for it has already been updated to nPQL
					nTmp=nTmp3;//correct the T2 final node's pointing place
				}
				/*int pCheck=0,num=1;
				while (v[pCheck].next!=0){
					pCheck=v[pCheck].next;
					num++;
				}
				if (num!=n+1){
					cout<<"ErrorNum!";
				}else{
					cout<<"NumRight "<<g_count<<endl;
				}*/
				//g_count++;
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

bool calcCost(int cServer,int n,int& totalCost,int& totalNum,bool bTest){
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

void processNetwork(int n,set<int>& pos,int cServer,int& totalCost,bool bMakeRoute,bool bBug){
	typedef pair<int,int> Pair;
	map<int,int> mapFlowID;
	for (int i=1;i<n+1;i++){
		if (v[i].d>0&&pos.find(i)==pos.end()){
			if (g[0][i].x>0&&g[0][i].x<v[i].d){
				int nFlow,nFlowLeft=v[i].d-g[0][i].x;
				g[0][i].x=v[i].d;
				do{
					nFlow=nFlowLeft;
					deleteFromNet(i,nFlow,n,pos);
					nFlowLeft-=nFlow;
				}while(nFlowLeft!=0);
			}
			if (g[0][i].x==0&&bBug){
				int nFlowLeft=v[i].d,nFlowCost=0,nFlow,nSrcID;
				mapFlowID.clear();
				do{
					nFlow=nFlowLeft;
					deleteAndCalcFromNet(i,nFlow,n,pos,nFlowCost,nSrcID);
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
void deleteFromNet(int j,int& flow,int n,set<int>& pos){
	for (int i=1;i<n+1;i++){
		if (g[i][j].x>0){
			if (flow>g[i][j].x)
				flow=g[i][j].x;
			if (pos.find(i)!=pos.end()){
				g[0][i].x-=flow;
			}else{
				deleteFromNet(i,flow,n,pos);
			}
			g[i][j].x-=flow;
			break;
		}
	}
}
void deleteAndCalcFromNet(int j,int& flow,int n,set<int>& pos,int& flowCost,int& srcID){
	for (int i=1;i<n+1;i++){
		if (g[i][j].x>0){
			if (flow>g[i][j].x)
				flow=g[i][j].x;
			if (pos.find(i)!=pos.end()){
				//g[0][i].x-=flow;
				srcID=i;
			}else{
				deleteAndCalcFromNet(i,flow,n,pos,flowCost,srcID);
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

GenAlg::GenAlg(int bitSize,int popSize,int nd,int cServer):m_bestFit(0),
	m_mutationRate(RAND_MAX*0.2),m_crossoverRate(RAND_MAX*0.9),//70%?80~95%?
	m_chromoBitSize(bitSize),//0.05~0.3?0.5~1%?
	m_cServer(cServer),m_maxFit(nd*cServer),
	m_conCMax(10){//?
	m_popSize=0;
	int nSumD=bitSize;
	for (int i=1;i<nd+1;i++){
		nSumD+=v[vCons[i].vid].d;
	}
	cout<<nSumD<<endl;
	while (m_popSize<popSize){
		set<int> setTmp;
		int k=rand();//*nSumD
		/*for (int i=1;i<bitSize+1;i++){
			//if (v[i].id>0){
			//	if (rand()*(v[i].d+1)>=k)
			//		setTmp.insert(i);
			if (rand()>=k)
				setTmp.insert(i);
		}*/
		for (int i=1;i<rand()%(bitSize+1)+1;i++){
			setTmp.insert(rand()%bitSize+1);
		}
		/*for (int i=1;i<rand()%(nd+1)+1;i++){//rand()*nd/RAND_MAX+1
			setTmp.insert(vCons[rand()%nd+1].vid);
		}*/
		Genome tmpGenome(setTmp);
		m_pop.push_back(tmpGenome);
		m_popSize++;
		//tmpGenome.print();
	}
}
void GenAlg::calcFit(int timeS){
	m_totalFit=0;
	for (int i=0;i<m_popSize;i++){
		set<int> setTmp=m_pop[i].decodeToSet();
		networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp);
		int nTotalCost=0,nTotalNum=0;
		processNetwork(m_chromoBitSize,setTmp,m_cServer,nTotalCost,true);
		//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp);
		//nTotalCost=0;nTotalNum=setTmp.size();
		calcCost(m_cServer,m_chromoBitSize,nTotalCost,nTotalNum);//,false);
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
void GenAlg::crossover(vector<int>& chromo1,vector<int>& chromo2){
	if (rand()<m_crossoverRate){
		int pos=rand()%m_chromoBitSize+1;
		int pos2=rand()%m_chromoBitSize+1;
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
			nTmp=chromo1[i]&(0x1<<Genome::m_intBitNS+1)-(0x1<<pos2%Genome::m_intBitNS);
			chromo1[i]&=~((0x1<<Genome::m_intBitNS+1)-(0x1<<pos2%Genome::m_intBitNS));
			chromo1[i]|=chromo2[i]&(0x1<<Genome::m_intBitNS+1)-(0x1<<pos2%Genome::m_intBitNS);
			chromo2[i]&=~((0x1<<Genome::m_intBitNS+1)-(0x1<<pos2%Genome::m_intBitNS));
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
			crossover(genomeF.m_genome,genomeM.m_genome);
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