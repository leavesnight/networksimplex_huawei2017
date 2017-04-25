#include "deploy_solver.h"
#include <stdio.h>
#include <algorithm>

#include <iostream>
#include <random>
#include <functional>

ServerSelectionSolver::~ServerSelectionSolver(){
	delete []g_edge;
	delete []g_edgeTmp;
	delete []g_edgeUnD;
	for (int i=0;i<g_numVert+m_numSS+2;++i){
		delete []g[i];
	}
	delete []g;
	delete []vCons;
	delete []v;
	delete []m_maxout;
	delete []m_lagn;
	delete []m_pnumSS;
	for (int i=0;i<g_numVert;++i){
		delete []g_srcEdge[i];
	}
	delete []g_srcEdge;
}
ServerSelectionSolver::ServerSelectionSolver(char* topo[MAX_EDGE_NUM],int line_num,int fast_mode):m_minCost(~0u>>1),g_bUseHeap(false),
	m_copy(false),m_fast_mode(fast_mode){
	//read parameters from topo
	sscanf(topo[0],"%d %d %d",&g_numVert,&g_m,&g_numDem);
	//if (g_numVert>500)
	//	m_fast_mode=1;
	int topo_line=1;
	NodeServer server;
	int max_extra_cost=0;
	if (m_fast_mode==2){
		while (sscanf(topo[++topo_line],"%d",&server.cost)==1){
			server.id=0;server.cap=~0u>>1;
			g_pServFast.push_back(server);
		}
		m_numSSFast=m_numSS=g_pServFast.size();
		m_root=g_numVert+m_numSS+1;
		g_pExtraCost=new int[g_numVert];//start from 0
		for (int i=0;i<g_numVert;++i)
			g_pExtraCost[i]=0;
	}else{
		while (sscanf(topo[++topo_line],"%d %d %d",&server.id,&server.cap,&server.cost)==3)
			g_pServFast.push_back(server);
		m_numSSFast=g_pServFast.size();
		if (m_fast_mode==1)
			m_numSS=1;
		else
			m_numSS=m_numSSFast;
		m_root=g_numVert+m_numSS+1;
		g_pExtraCost=new int[g_numVert];//start from 0
		int i,extra_cost;
		while (sscanf(topo[++topo_line],"%d %d",&i,&extra_cost)==2){
			g_pExtraCost[i]=extra_cost;
			if (max_extra_cost<extra_cost) max_extra_cost=extra_cost;
		}
	}
	m_pos_serv.resize(g_numVert,m_numSSFast-1);
	int nNum[4],nTmp3;
	g=new int*[g_numVert+m_numSS+2];//one for big super point,one for big M point
	for (int i=0;i<g_numVert+m_numSS+2;++i){
		g[i]=new int[g_numVert+m_numSS+2];
		for (int j=0;j<g_numVert+m_numSS+2;++j)
			g[i][j]=-1;//-1 means no corresponding edge
	}
	g_edge=new NodeEdge[g_m*2+(g_numVert+1)*m_numSS+g_numDem+1];//1*m_numSS for big super point edges,g_numDem+1 is for big M point edges
	g_edgeUnD=new NodeEdgeUnD[g_m+(g_numVert+1)*m_numSS+g_numDem+1];//the same as upper
	g_edgeCount=g_edgeUnDCount=0;
	v=new NodeVertex[g_numVert+m_numSS+2];//1 for big super point,1 for artificial big M point
	while (sscanf(topo[++topo_line],"%d %d %d %d",nNum,nNum+1,nNum+2,nNum+3)==4)
		if (g[nNum[0]][nNum[1]]==-1){
			g_edge[g_edgeCount].u=nNum[2];
			g_edge[g_edgeCount].c=nNum[3];
			g_edge[g_edgeCount].idBegin=nNum[0];//it starts from point 0, we also let it from 0
			g_edge[g_edgeCount].idEnd=nNum[1];
			g[nNum[0]][nNum[1]]=g_edgeCount;
			v[nNum[0]].idEdge.push_back(g_edgeCount);
			v[nNum[1]].idEdgeFrom.push_back(g_edgeCount);
			++g_edgeCount;
			g_edge[g_edgeCount].u=nNum[2];
			g_edge[g_edgeCount].c=nNum[3];
			g_edge[g_edgeCount].idBegin=nNum[1];//bidirectional
			g_edge[g_edgeCount].idEnd=nNum[0];
			g[nNum[1]][nNum[0]]=g_edgeCount;
			v[nNum[1]].idEdge.push_back(g_edgeCount);
			v[nNum[0]].idEdgeFrom.push_back(g_edgeCount);
			++g_edgeCount;
			g_edgeUnD[g_edgeUnDCount].idBegin=nNum[0];
			g_edgeUnD[g_edgeUnDCount].idEnd=nNum[1];
			++g_edgeUnDCount;
		}
	for (int i=0;i<g_numVert;++i) sort(v[i].idEdgeFrom.begin(),v[i].idEdgeFrom.end(),[this](const int a,const int b){return g_edge[a].c<g_edge[b].c;});
	--line_num;
	vCons=new int[g_numVert+1];//id==0 means not a consumer
	for (int i=0;i<g_numVert+m_numSS+2;++i)
		v[i].d=v[i].id=0;
	for (int i=0;i<g_numVert+1;++i)
		vCons[i]=0;
	g_totalDem=0;
	while (topo_line<line_num&&sscanf(topo[++topo_line],"%d %d %d",nNum,nNum+1,nNum+2)==3){
		v[nNum[1]].d=nNum[2];//it starts from point 0, but we let it from 1, the left one is all 0 except vBSS.d&&vSSk.d
		v[nNum[1]].id=nNum[0]+1;//id+1, I think the d maybe [0
		vCons[nNum[0]+1]=nNum[1];
		g_totalDem+=nNum[2];
	}
	m_maxout=new int[g_numVert];
	m_lagn=new double[g_numVert];
	m_maxCS=0;
	for (int i=0;i<g_numVert;++i){
		m_maxout[i]=v[i].d;
		for (int j=0;j<v[i].idEdge.size();++j){
			m_maxout[i]+=g_edge[v[i].idEdge[j]].u;
		}
		if (m_maxout[i]>g_totalDem)
			m_maxout[i]=g_totalDem;
		if (m_maxout[i]>g_pServFast[m_numSSFast-1].cap)
			m_maxout[i]=g_pServFast[m_numSSFast-1].cap;
		//cout<<i<<" maxout: "<<m_maxout[i]<<" totalDem:"<<g_totalDem<<" di:"<<v[i].d<<endl;
		m_lagn[i]=1.0*g_pServFast[0].cost/g_totalDem/2;//m_maxout[i]/2;
		for (int k=0;k<m_numSSFast-1;++k){//no need to judge if the last one for the initial!
			if (m_maxout[i]<=g_pServFast[k].cap){
				m_pos_serv[i]=k;
				/*cout<<g_pServFast[k].cap-m_maxout[i]<<" ";
				if (k>0&&(g_pServFast[k].cap-m_maxout[i]<(g_pServFast[k].cap-g_pServFast[k-1].cap)/2)){
					--m_pos_serv[i];
					//cout<<g_pServFast[k].cap-m_maxout[i]<<" ";
				}*/
				break;
			}
		}
		//if (m_pos_serv[i]>4)
		//	m_pos_serv[i]=4;
		//cout<<m_pos_serv[i]<<" ";
		if (m_maxCS<g_pServFast[m_pos_serv[i]].cost+g_pExtraCost[i])
			m_maxCS=g_pServFast[m_pos_serv[i]].cost+g_pExtraCost[i];
	}
	m_pnumSS=new int[g_numVert];
	if (m_fast_mode==1)
		for (int i=0;i<g_numVert;++i) m_pnumSS[i]=1;
	else
		for (int i=0;i<g_numVert;++i) m_pnumSS[i]=m_pos_serv[i]+1;
	//make an initial of the networkSimplex
	//v[BSS].d=INT_MAX?g_totalDem;//sink will use vBSS.d?
	g_srcEdge=new int*[g_numVert];
	for (int i=0;i<g_numVert;++i)
		g_srcEdge[i]=new int[m_pnumSS[i]];
	for (int i=0;i<g_numVert;++i){
		for (int k=0;k<m_pnumSS[i];++k){
			int ik=k+g_numVert;
			if (m_fast_mode==1){
				g_edge[g_edgeCount].u=g_pServFast[m_pos_serv[i]].cap;
			}else{
				if (k==0)
					g_edge[g_edgeCount].u=g_pServFast[k].cap;
				else
					g_edge[g_edgeCount].u=g_pServFast[k].cap-g_pServFast[k-1].cap;//super point,>=5000*500
			}
			g_edge[g_edgeCount].idBegin=ik;
			g_edge[g_edgeCount].idEnd=i;
			g[ik][i]=g_edgeCount;
			g_srcEdge[i][k]=g_edgeCount;
			v[ik].idEdge.push_back(g_edgeCount);
			v[i].idEdgeFrom.push_back(g_edgeCount);
			g_edgeCount++;
			g_edgeUnD[g_edgeUnDCount].idBegin=ik;//always ik
			g_edgeUnD[g_edgeUnDCount].idEnd=i;//always i
			++g_edgeUnDCount;
		}
	}
	g_edgeCoreCount=g_edgeCount;
	g_edgeCoreUnDCount=g_edgeUnDCount;
	//make big super point && M edges
	int bigM=0;
	for (int k=0;k<m_numSSFast;++k){
		if (bigM<g_pServFast[k].cost)
			bigM=g_pServFast[k].cost;
	}
	for (int k=0;k<m_numSS;++k){
		int ik=k+g_numVert,i=g_numVert+m_numSS;
		g_edge[g_edgeCount].u=~0u>>1;//g_edge[g_edgeCount].u*g_numVert;//if use ~0u>>1 please notice x=l-u
		g_edge[g_edgeCount].c=0;
		g_edge[g_edgeCount].idBegin=i;
		g_edge[g_edgeCount].idEnd=ik;
		g[i][ik]=g_edgeCount;
		v[i].idEdge.push_back(g_edgeCount);
		v[ik].idEdgeFrom.push_back(g_edgeCount);
		g_edgeCount++;
		g_edgeUnD[g_edgeUnDCount].idBegin=i;//always i
		g_edgeUnD[g_edgeUnDCount].idEnd=ik;//always ik
		++g_edgeUnDCount;
	}
	bigM+=max_extra_cost+(g_numVert-1)*100;
	bigM=bigM/2+1;//2*bigM should be more expensive than any route from super sources to the consumer points
	for (int j=1;j<g_numDem+1;++j){
		int ik=m_root,i=vCons[j];
		g_edge[g_edgeCount].u=~0u>>1;//no limit for M point edge
		g_edge[g_edgeCount].c=bigM;
		g_edge[g_edgeCount].idBegin=ik;
		g_edge[g_edgeCount].idEnd=i;
		g[ik][i]=g_edgeCount;
		v[ik].idEdge.push_back(g_edgeCount);
		v[i].idEdgeFrom.push_back(g_edgeCount);
		g_edgeCount++;
		g_edgeUnD[g_edgeUnDCount].idBegin=ik;//always ik
		g_edgeUnD[g_edgeUnDCount].idEnd=i;//always i
		++g_edgeUnDCount;
	}
	{//g_totalDem should always>0
	int ik=g_numVert+m_numSS,i=m_root;
	g_edge[g_edgeCount].u=~0u>>1;
	g_edge[g_edgeCount].c=bigM;
	g_edge[g_edgeCount].idBegin=ik;
	g_edge[g_edgeCount].idEnd=i;
	g[ik][i]=g_edgeCount;
	v[ik].idEdge.push_back(g_edgeCount);
	v[i].idEdgeFrom.push_back(g_edgeCount);
	g_edgeCount++;
	g_edgeUnD[g_edgeUnDCount].idBegin=ik;//always ik
	g_edgeUnD[g_edgeUnDCount].idEnd=i;//always i
	++g_edgeUnDCount;
	}
	//for (int i=0;i<g_numVert+m_numSS+2;++i)
	v[m_root].pi=0;//!!!very important initial
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
	g_edgeTmp=new NodeEdge[g_m*2+(g_numVert+1)*m_numSS+g_numDem+1];
	for (int i=0;i<g_edgeCount;++i)//repaired start from 0
		g_edgeTmp[i]=g_edge[i];
	/*int nCount1=0;
	for (int i=0;i<g_numVert;++i){
		int nTmp=0,nTmp2=0;
		sort(v[i].idEdge.begin(),v[i].idEdge.end(),[this](int pos1,int pos2)->bool{
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
		if (nTmp2>=g_pExtraCost[i]-500){
			if (nTmp2==0)
				cout<<"500 "<<i<<endl;
			else
				cout<<"GT g_costServ: "<<i<<endl;
			++nCount1;
		}
	}
	cout<<nCount1;
	cout<<endl;*/
};
const char* ServerSelectionSolver::getResult(){
	ostringstream sout;
	int nLines=g_totalCost=0;
	if (m_minCost<~0u>>1){
		if (0&&m_fast_mode==1){
			for (int i=0;i<g_numVert;++i){
				if (m_bestServerPos[i]<m_numSSFast){
					m_pos_serv[i]=m_bestServerPos[i];
					g_edge[g_srcEdge[i][0]].u=g_pServFast[m_bestServerPos[i]].cap;
				}
			}
			netSAinit(g_edge);
		}
		networkSimplexAlg(m_bestServerPos,g_edge);
		int nTotalCost=0;
		/*nTotalNum=nServerPos.size();
		while (!calcCost(g_costServ,g_numVert,nTotalCost,nTotalNum,g)){
			processNetwork(nServerPos,nTotalCost,g_edge,true);
			nTotalNum=nServerPos.size();
			networkSimplexAlg(nServerPos,g_edge);
			nTotalCost=0;
		}*/
		for (int i=0;i<g_edgeCoreCount;++i){
			g_edgeTmp[i].x=g_edge[i].x;
		}
		processNetwork(m_bestServerPos,nTotalCost,g_edgeTmp,true);
		calcCost(nTotalCost,g_edgeTmp);
		networkSimplexAlg(m_bestServerPos,g_edge);

		cout<<"Final minCost="<<nTotalCost<<" bestPos=";
		for (auto i=m_bestServerPos.begin();i!=m_bestServerPos.end();++i)
			cout<<" "<<(int)*i;
		cout<<endl;

		int nFlowCheck=g_totalDem;
		g_leftCus=0;
		for (int i=0;i<g_numVert;++i){
			if (g_edge[g_srcEdge[i][0]].x>0){
				int kServ;
				for (kServ=m_pnumSS[i]-1;kServ>=0;--kServ)
					if (g_edge[g_srcEdge[i][kServ]].x>0) break;
				int k=kServ;
				if (m_fast_mode==1){
					for (kServ=m_numSSFast-1;kServ>=0;--kServ)
						if (g_edge[g_srcEdge[i][0]].x>g_pServFast[kServ].cap) break;
					++kServ;
				}
				while (g_edge[g_srcEdge[i][0]].x>0){
					sout<<i<<" ";
					int nFlow=g_edge[g_srcEdge[i][k]].x;//0;
					//for (int j=k;j>=0;--j) nFlow+=g_edge[g_srcEdge[i][j]].x;
					if (k>0) nFlow+=g_pServFast[k-1].cap;
					if (v[i].id>0&&v[i].d>0){//cannot omit .d>0!!!
						if (nFlow>=v[i].d){
							nFlow=v[i].d;
							v[i].d=0;
							++g_leftCus;
						}else{
							v[i].d-=nFlow;
						}
						sout<<v[i].id-1<<" "<<nFlow<<" "<<g_pServFast[kServ].id;
					}else
						printPath(i,nFlow,sout,kServ);
					if (g_leftCus<g_numDem) sout<<endl;
					++nLines;
					nFlowCheck-=nFlow;
					while (nFlow>g_edge[g_srcEdge[i][k]].x){
						nFlow-=g_edge[g_srcEdge[i][k]].x;
						g_edge[g_srcEdge[i][k]].x=0;
						--k;
					}
					if (!(g_edge[g_srcEdge[i][k]].x-=nFlow)) --k;
					if (g_edge[g_srcEdge[i][0]].x==0){
						g_totalCost+=g_pServFast[kServ].cost+g_pExtraCost[i];
					}
				}
			}
		}
		if (nFlowCheck)
			cout<<"total flow!=demand! please check the program!"<<endl;
	}
	if (nLines>0&&nLines<=300000){
		char cstrTmp[7]="";
		sprintf(cstrTmp,"%d",nLines);
		strTopo=string(cstrTmp)+"\n\n"+sout.str();
	}else{
		return "NA";
		/*sout.str("");
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
		g_totalCost=g_numDem*g_costServ;*/
	}
	cout<<"Real total cost: "<<g_totalCost<<endl;
	return strTopo.c_str();
}
void ServerSelectionSolver::printPath(int i,int& flow,ostream& sout,int kserv){
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
				sout<<j<<" "<<v[j].id-1<<" "<<flow<<" "<<g_pServFast[kserv].id;
			}else{
				sout<<j<<" ";
				//int nTmp=g_edge[ijMap].u;
				//g_edge[ijMap].u=0;
				printPath(j,flow,sout,kserv);
				//g_edge[ijMap].u=nTmp;
			}
			g_edge[ijMap].x-=flow;
			g_totalCost+=g_edge[ijMap].c*flow;
			break;
		}
	}
}
void ServerSelectionSolver::printTree(){//just for test
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
void ServerSelectionSolver::xjbs(){
	clock_t tmXjbStart=clock();
	vector<unsigned char> vecPos(g_numVert);
	random_device rd;
	mt19937 mt(rd());
	uniform_int_distribution<int> dist(0,g_numVert);
	auto dice=bind(dist,mt);
	for (int i=0;i<1000;++i){
		int N=dice();
		for (int j=0;j<g_numVert;++j)
			vecPos[j]=m_numSSFast;
		for (int j=0;j<N;++j){
			vecPos[dice()%g_numVert]=dice()%m_numSSFast;
		}
		//printf("xjbs: %d\n",i);
		netSAinit(g_edge);
		//int nTmTmp=clock();
		networkSimplexAlg(vecPos,g_edge);
		//g_count+=clock()-nTmTmp;
		//if (clock()-tmXjbStart>30*CLOCKS_PER_SEC)
		//	break;
	}
	cout<<"xjbs used time: "<<clock()-tmXjbStart<<endl;
}
