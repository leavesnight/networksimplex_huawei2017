#include "deploy_solver.h"
#include "deploy.h"
#include <map>

#include <algorithm>

#include <iostream>
using namespace std;

void ServerSelectionSolver::startSolver(int timeS){
	bool time_up=false;
	vector<unsigned char> nServerPos(g_numVert,m_numSSFast),nGreedyServerPos(g_numVert,m_numSSFast);
	/*int nMinCost=0,nMinPos[MAX_CONSUME_NUM]={5,18,21,23,26,37,38,43,47,50,55,57,58,61,62,67,73,78,82,86,90,98
	,104,107,109,115,124,127,129,136,138,140,148,156,159,160},nMinPosCount=36;
	for (int i=0;i<nMinPosCount;++i){
		if (nMinPos[i]>0){
			nMinPos[i]++;
		}
	}*/
	
	/*if (g_numVert>500){
		for (int i=0;i<g_numVert;++i)
			if (v[i].id)
				nServerPos[i]=m_numSSFast-1;
		int z_seed=Flow(nServerPos,this).val(),z_best;
		cout<<"Direct Seed: "<<z_seed<<endl;
		for(int changed=1,tm=0;changed;){
			changed=0;
			for(int i=0;i<g_numVert;++i){
				bool change_state=false;
				int old_state=nServerPos[i];
				nServerPos[i]=m_pnumSS[i];
				while (!change_state){
					++tm;
					if (old_state<m_numSSFast){
						nServerPos[i]=m_numSSFast;
						change_state=true;
					}else{
						--nServerPos[i];
						//nServerPos[i]=rand()%m_numSS;
						//if (nServerPos[i]==0)
							change_state=true;
					}
					int t=Flow(nServerPos,this).val();
					if(t<z_seed){
						z_seed=t;
						changed=1;
						//old_state=nServerPos[i];
						m_bestServerPos=nServerPos;
						if (nServerPos[i]<m_numSSFast&&nServerPos[i]>0)
							change_state=false;
						cout<<z_seed<<" "<<i<<" "<<(int)nServerPos[i]<<endl;
					}
					if (change_state){
						nServerPos[i]=old_state;
					}
					if (clock()-g_tmStart>timeS*CLOCKS_PER_SEC){
						time_up=true;break;
					}
				}
				if (time_up) break;
			}
			if (time_up) break;
		}
		cout<<"Greedy Seed: "<<z_seed<<endl;
		nGreedyServerPos=nServerPos;
		m_minCost=z_seed;
		cout<<g_count2<<endl;
	}
	//if (g_numVert>5000)
	for (int k=0;k<g_numDem;k++){
		int minPosk=0,minPos=g_numVert;
		int nCostBefore=m_minCost;
		nServerPos=nGreedyServerPos;
		for (int i=0;i<g_numVert;++i){
			int old_state=nServerPos[i];
			for (int k=0;k<=m_numSSFast;++k){
				if (k==m_pnumSS[i])
					k=m_numSSFast;
				nServerPos[i]=k;
				int nTotalCost=networkSimplexAlg(nServerPos,g_edge);
				//for (int i=0;i<g_edgeCoreCount;++i){
				//	g_edgeTmp[i].x=g_edge[i].x;
				//}
				//processNetwork(nServerPos,nTotalCost,g_edgeTmp,false,2);
				//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
				//nTotalCost=0;nTotalNum=setTmp.size();
				//calcCost(nTotalCost,g_edgeTmp);
				calcCost(nTotalCost,g_edge);
				if (m_minCost>nTotalCost){
					m_minCost=nTotalCost;
					minPos=i;minPosk=k;//old_state=k;
				}
				if (clock()-g_tmStart>timeS*CLOCKS_PER_SEC){
					time_up=true;break;
				}
			}
			nServerPos[i]=old_state;
			if (time_up) break;
		}
		if (minPos<g_numVert){
			//nGreedyServerPos=nServerPos;
			nGreedyServerPos[minPos]=minPosk;
			cout<<nCostBefore<<" after greedy: "<<m_minCost<<" "<<"pos(in program):"<<minPos<<" "<<minPosk<<endl;
			//if (nCostBefore-nMinCost<g_costServ) break;
		}else{
			break;
		}
		if (time_up) break;
	}
	cout<<g_count2<<endl;
	m_bestServerPos=nGreedyServerPos;
	/*nServerPos=m_bestServerPos;
	int minPos[2]={-1,-1},minPosk[2];
	for (int i=0;i<g_numVert-1;++i){
		for (int i2=i+1;i2<g_numVert;++i2){
			if (i2!=i&&nGreedyServerPos[i]<m_numSS&&nGreedyServerPos[i2]<m_numSS){
				for (int k=0;k<m_numSS;++k){
					for (int k2=0;k2<m_numSS;++k2){
						nServerPos[i]=k;
						nServerPos[i2]=k2;
						networkSimplexAlg(nServerPos,g_edge);
						int nTotalCost=0;
						for (int i=0;i<g_edgeCoreCount;++i){
							g_edgeTmp[i].x=g_edge[i].x;
						}
						//processNetwork(nServerPos,nTotalCost,g_edgeTmp,false,2);
						calcCost(nTotalCost,g_edgeTmp);
						if (m_minCost>nTotalCost){
							m_minCost=nTotalCost;
							minPos[0]=i;minPos[1]=i2;
							minPosk[0]=k;minPosk[1]=k2;
						}
						nServerPos[i]=nServerPos[i2]=m_numSS;
						if (clock()-g_tmStart>timeS*CLOCKS_PER_SEC)
							break;
					}
					if (clock()-g_tmStart>timeS*CLOCKS_PER_SEC){
						time_up=true;break;
					}
				}
				if (time_up) break;
			}
		}
		if (time_up) break;
	}
	if (minPos[0]>=0){
		m_bestServerPos[minPos[0]]=minPosk[0];
		m_bestServerPos[minPos[1]]=minPosk[1];
	}*/
	//GA-genetic algorithm
	/*for (int i=0;i<g_numVert;++i){
		m_pos_serv[i]=3;
		g_edge[g_srcEdge[i][0]].u=g_pServFast[3].cap;
	}
	if (isapNA()){
		for (int i=0;i<g_numVert;++i){
			g_edge[g_srcEdge[i][0]].u=g_pServFast[m_numSSFast-1].cap;
		}
		return;
	}*/
	//cout<<g_edgeUnDCount<<endl;
	m_fixedServerPos.resize(g_numVert,m_numSSFast+1);
	makejude(m_judgePos);
	//0.67/0.87/0.77 0.19
	int dmax=1;
	for (int i=1;i<g_numDem;++i){
		if (dmax<v[vCons[i]].d) dmax=v[vCons[i]].d;
	}
	m_unfixednum=0;
	for (int i=0;i<g_numVert;++i){
		if (m_judgePos[i]*1.0/RAND_MAX>=0.90)
			;//m_fixedServerPos[i]=m_numSSFast+2;
		else if (m_judgePos[i]*1.0/RAND_MAX<=0.20)
			;//m_fixedServerPos[i]=m_numSSFast;
		else
			++m_unfixednum;
	}
	cout<<m_unfixednum<<endl;
	if (g_numVert>10000){
		vector<unsigned char> best(g_numVert,m_numSSFast);
		for (int i=0;i<g_numVert;++i)
			if (v[i].id)
				best[i]=m_numSSFast-1;
		int z_best=Flow(best,this).val();
		for (int changed=1;changed;){
			changed=0;
			for (int i=0;i<g_numVert;++i){
				if (clock()-g_tmStart>timeS*CLOCKS_PER_SEC){
					break;
				}
				vector<unsigned char> t_best=best;
				int t_z=z_best,j=0;
				j=0;
				while (true){
					int tmp=best[i];
					best[i]=j;
					int t=Flow(best,this).val();
					if (t<t_z){
						t_z=t;
						t_best=best;
						best[i]=tmp;
					}else
						best[i]=tmp;
					if (j<m_pnumSS[i]) ++j;
					else if (j!=m_numSSFast) j=m_numSSFast;
					else break;
				}
				if (t_z<z_best){
					changed=1;
					best=t_best;
					z_best=t_z;
				}
			}
			cout<<z_best<<endl;
		}
		m_minCost=z_best;
		m_bestServerPos=best;
		nGreedyServerPos=best;
	}
	int nTry=0;
	if (g_numVert<000){
        xjb_search(timeS);
	}
	if (g_numVert<000){
		int nPop,nType=1;
		if (g_numVert<1000){
			nPop=10;//200;
			//nType=0;
		}else{
			nPop=10;
		}
		int nMCTmp,nMCCount=0;
		do{
		++nMCCount;
		GenAlg genAlg(nPop,this,nGreedyServerPos);//20~30 chromosomes
		//genAlg.startGA(85);
		genAlg.startPSO(timeS,0.9*RAND_MAX,0.5*RAND_MAX,0.5*RAND_MAX,nType);
		//genAlg.startPSO(85,1,2,2);
		nMCTmp=genAlg.getMinCost();
		if (m_minCost>nMCTmp){
			cout<<"pso min: "<<nMCTmp<<" from "<<m_minCost<<endl;
			m_minCost=nMCTmp;
			m_bestServerPos=genAlg.getBestServerPos();
			nMCCount=0;
		}
		cout<<m_minCost<<endl;
		}while (nMCCount<nTry);
	}

	//branchbound(88);

	//SA-method
	if (g_numVert<11000){
		startSA(timeS);
	}

	cout<<"use time: "<<(clock()-g_tmStart)*1000/CLOCKS_PER_SEC<<"ms"<<endl;

	cout<<"nMinCost="<<m_minCost<<" minPos=";
	/*ostream_iterator<int,char> out_iter(cout," ");
	copy(nMinServerPos.begin(),nMinServerPos.end(),out_iter);*/
	/*for_each(nMinServerPos.begin(),nMinServerPos.end(),[](int pos){
		cout<<" "<<pos;
	});*/

	int sum=0,min=~0u>>1;
	int numServer=0;
	for (int  i=0;i<g_numVert;++i){
		if (m_bestServerPos[i]<m_numSSFast){
			cout<<" "<<i;
			if (v[i].id)
				cout<<"*"<<v[i].d;
			/*cout<<" max"<<m_pos_serv[i]<<" use"<<(int)m_bestServerPos[i]<<" judgePos="<<m_judgePos[i]*1.0/RAND_MAX<<endl;
			cout<<v[i].d<<" "<<m_maxout[i]<<" "<<g_pExtraCost[i]<<" "<<v[i].idEdge.size()<<endl;
			sum+=m_judgePos[i];
			if (v[i].d&&min>m_judgePos[i]) min=m_judgePos[i];
			++numServer;*/
		}
	}
	//cout<<"Server num="<<numServer<<endl;
	/*int num2000=0,numl2=0,num2d=0,numl2d=0;
	for (int i=0;i<g_numVert;++i){
		if (g_pExtraCost[i]==2000){
			++num2000;
			if (v[i].id)
				++num2d;
		}else{
			++numl2;
			if (v[i].id)
				++numl2d;
		}
	}
	cout<<num2000<<" d"<<num2d<<" "<<numl2<<" d"<<numl2d<<endl;
	cout<<sum*1.0/m_judgesum<<endl;
	int max=0;
	for (int  i=0;i<g_numVert;++i){
		if (m_bestServerPos[i]==m_numSSFast){
			//cout<<" "<<m_judgePos[i]*1.0/RAND_MAX;
			if (v[i].id&&m_judgePos[i]>max) max=m_judgePos[i];
		}
	}
	cout<<endl<<max*1.0/RAND_MAX<<" "<<min*1.0/RAND_MAX<<endl;*/
	cout<<endl;
}

int ServerSelectionSolver::getRawCost(vector<unsigned char>& pos){
	/*for (int k=1;k<g_numDem+1;k++){
		int i=vCons[k];
		if (pos[i]==m_numSSFast){//v[i].d>0
			if (g_edge[g_srcEdge[i][0]].x>0){
				return ~0u>>1;
			}
		}
	}*/
	for (int i=0;i<g_numVert;++i){
		if (pos[i]==m_numSSFast){//v[i].d>0
			if (g_edge[g_srcEdge[i][0]].x>0){
				return ~0u>>1;
			}
		}else{
			/*for (int k=m_pnumSS[i]-1;k>=0;--k)
				if (g_edge[g_srcEdge[i][k]].x>0){
					int tmp=0;
					if (k>0) tmp=g_pServFast[k-1].cap;
					if (g_edge[g_srcEdge[i][k]].x+tmp>g_pServFast[pos[i]].cap)
						return ~0u>>1;
				}*/
		}
	}
	int cost=0;
	calcCost(cost,g_edge);
	return cost;
	for (int i=0;i<g_numVert;++i){
		if (pos[i]<m_numSSFast){
			cost+=g_pServFast[pos[i]].cost+g_pExtraCost[i];
		}
	}
	for (int i=0;i<g_m*2;++i){
		if (g_edge[i].x>0)
			cost+=g_edge[i].x*g_edge[i].c;
	}
	return cost;
}
bool ServerSelectionSolver::calcCost(int& totalCost,NodeEdge g_edge[],bool bCheck){
	if (totalCost==~0u>>1) return false;//NA
	for (int i=0;i<g_numVert;++i){
		//calc the server cost by real flow network
		if (m_fast_mode&&g_edge[g_srcEdge[i][0]].x>0){
			int k;
			for (k=m_numSSFast-1;k>=0;--k)
				if (g_pServFast[k].cap<g_edge[g_srcEdge[i][0]].x) break;
			totalCost+=g_pServFast[k+1].cost+g_pExtraCost[i];
		}else
			for (int k=m_pnumSS[i]-1;k>=0;--k){
				if (g_edge[g_srcEdge[i][k]].x>0){
					totalCost+=g_pServFast[k].cost+g_pExtraCost[i];
					//totalCost+=(3000+g_pExtraCost[i])*g_edge[g_srcEdge[i][k]].x;
					break;
				}
			}
	}
	if (!bCheck)
		return false;
	for (int i=0;i<g_m*2;++i){
		if (g_edge[i].x>0)
			totalCost+=g_edge[i].x*g_edge[i].c;
	}
	return true;
}
void ServerSelectionSolver::processNetwork(vector<unsigned char>& pos,int& totalCost,NodeEdge g_edge[],bool bMakeRoute,int bBug){
	typedef pair<int,int> Pair;
	map<int,int> mapFlowID,mapIDFlowAdd;
	/*if (g_count2==3135){
		int j=38;
		for (int k=0;k<v[j].idEdgeFrom.size();++k)
			cout<<v[j].idEdgeFrom[k]<<" "<<g_edge[v[j].idEdgeFrom[k]].x<<" "<<g_edge[v[j].idEdgeFrom[k]].idBegin<<endl;
		cout<<(int)pos[j]<<endl;
	}*/
	if (totalCost>0) return;//NA
	if (bBug>0)
	for (int k=1;k<g_numDem+1;k++){
		int i=vCons[k];
		if (pos[i]==m_numSSFast){//v[i].d>0
			int iMap,kMap=m_numSSFast;
			for (int j=m_pnumSS[i]-1;j>=0;--j)
				if (g_edge[g_srcEdge[i][j]].x>0){
					kMap=j;
					iMap=g_srcEdge[i][j];
					break;
				}
			if (kMap<m_numSSFast){
				int di_left=v[i].d;
				if (kMap>0) di_left-=g_pServFast[kMap-1].cap;//uncomplete di_left
				if (g_edge[iMap].x<di_left){
					int nFlow,nFlowLeft;
					if (di_left>g_edge[iMap].u){
						nFlowLeft=g_edge[iMap].u-g_edge[iMap].x;
						g_edge[iMap].x=g_edge[iMap].u;
					}else{
						nFlowLeft=di_left-g_edge[iMap].x;
						g_edge[iMap].x=di_left;
					}
					do{
						nFlow=nFlowLeft;
						deleteFromNet(i,nFlow,pos,g_edge);
						nFlowLeft-=nFlow;
					}while(nFlowLeft!=0);
				}
			}
			if (kMap==m_numSSFast&&bBug==2){
				int nFlowLeft,nFlowCost=0,nFlow,nSrcID=-1,nFlowInit;
				if (v[i].d>g_pServFast[0].cap)
					nFlowLeft=g_pServFast[0].cap;
				else
					nFlowLeft=v[i].d;
				nFlowInit=nFlowLeft;
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
				if (nFlowCost<g_pServFast[0].cost+g_pExtraCost[i]){
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
					for (auto i=mapFlowID.begin();i!=mapFlowID.end();++i){
						auto tmpIter=mapIDFlowAdd.find(i->first);
						if (tmpIter==mapIDFlowAdd.end())
							mapIDFlowAdd.insert(*i);
						else
							(*tmpIter).second+=i->second;
					}
				}else{
					mapIDFlowAdd.insert(Pair(i,nFlowInit));
				}
			}
		}
	}
	if (bBug==2){
		for (auto i=mapIDFlowAdd.begin();i!=mapIDFlowAdd.end();++i){
			int l=0;
			while (g_edge[g_srcEdge[i->first][l]].u<i->second+g_edge[g_srcEdge[i->first][l]].x){
				i->second-=g_edge[g_srcEdge[i->first][l]].u-g_edge[g_srcEdge[i->first][l]].x;
				g_edge[g_srcEdge[i->first][l]].x=g_edge[g_srcEdge[i->first][l]].u;
				++l;
			}
			g_edge[g_srcEdge[i->first][l]].x+=i->second;
		}
	}
	if (bMakeRoute){
		for (int i=0;i<g_numVert;++i){
			if (m_fast_mode==1){
				if (g_edge[g_srcEdge[i][0]].x>0){
					int k;
					for (k=m_numSSFast-1;k>=0;--k)
						if (g_edge[g_srcEdge[i][0]].x>g_pServFast[k].cap) break;
					pos[i]=k+1;
				}else pos[i]=m_numSSFast;
			}else{
				pos[i]=m_numSSFast;
				for (int k=m_pnumSS[i]-1;k>=0;--k){
					if (g_edge[g_srcEdge[i][k]].x>0){
						pos[i]=k;
						break;
					}
				}
			}
		}
	}
}
void ServerSelectionSolver::deleteFromNet(int j,int& flow,vector<unsigned char>& pos,NodeEdge g_edge[]){
	int k_max=v[j].idEdgeFrom.size()-m_pnumSS[j]-1;
	if (v[j].id)
		--k_max;//-m_numSS means no kj, and -1(bigM) is only for it's from vCons[i]!!!
	for (int k=k_max;k>=0;--k){
	//for (int k=0;k<k_max;++k){
		int ijMap=v[j].idEdgeFrom[k];
		if (g_edge[ijMap].x>0){
			int i=g_edge[ijMap].idBegin;
			if (flow>g_edge[ijMap].x)
				flow=g_edge[ijMap].x;
			if (g_edge[g_srcEdge[i][0]].x>0){
				for (int l=m_pnumSS[i]-1;l>=0;--l)
					if (g_edge[g_srcEdge[i][l]].x>0){
						int tmp=g_edge[g_srcEdge[i][l]].x;
						if (l>0) tmp+=g_pServFast[l-1].cap;
						if (tmp>flow)
							tmp=flow;
						else
							flow=tmp;
						while (tmp>g_edge[g_srcEdge[i][l]].x){
							tmp-=g_edge[g_srcEdge[i][l]].x;
							g_edge[g_srcEdge[i][l]].x=0;
							--l;
						}
						g_edge[g_srcEdge[i][l]].x-=tmp;
						break;
					}
			}else{
				deleteFromNet(i,flow,pos,g_edge);
			}
			g_edge[ijMap].x-=flow;
			break;
		}
	}
}
void ServerSelectionSolver::deleteAndCalcFromNet(int j,int& flow,vector<unsigned char>& pos,int& flowCost,int& srcID,NodeEdge g_edge[]){
	int k_max=v[j].idEdgeFrom.size()-m_pnumSS[j];
	if (v[j].id)
		--k_max;//-m_numSS means no kj, and -1(bigM) is only for it's from vCons[i]!!!
	for (int k=0;k<k_max;++k){
		int ijMap=v[j].idEdgeFrom[k];
		if (g_edge[ijMap].x>0){
			int i=g_edge[ijMap].idBegin;
			if (flow>g_edge[ijMap].x)
				flow=g_edge[ijMap].x;
			if (g_edge[g_srcEdge[i][0]].x>0){
				for (int l=m_pnumSS[i]-1;l>=0;--l)
					if (g_edge[g_srcEdge[i][l]].x>0){
						int tmp=g_edge[g_srcEdge[i][l]].x;
						if (l>0) tmp+=g_pServFast[l-1].cap;
						if (tmp>flow)
							tmp=flow;
						else
							flow=tmp;
						while (tmp>g_edge[g_srcEdge[i][l]].x){
							tmp-=g_edge[g_srcEdge[i][l]].x;
							g_edge[g_srcEdge[i][l]].x=0;
							--l;
						}
						g_edge[g_srcEdge[i][l]].x-=tmp;
						break;
					}
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

void ServerSelectionSolver::branchbound(int timeS){
	cout<<"BB method starts: "<<clock()<<endl;
	vector<unsigned char> pos(g_numVert,m_numSS+1);//undetermined one
	int pos_branch=-1;
	struct NodeBB{
		vector<unsigned char> pos;
		int cost_bound,pos_branch;
		//NodeBB(){}
		NodeBB(vector<unsigned char>& _pos,int _cost,int _pos_branch):
			pos(_pos),cost_bound(_cost),pos_branch(_pos_branch){}
		bool operator <(const NodeBB& nb) const{ return cost_bound>nb.cost_bound;}
	} posBB(pos,calBBbound(pos,pos_branch),pos_branch);
	vector<priority_queue<NodeBB>> vecqu(g_numVert+1);
	
	/*for (int i=1;i<g_numDem+1;++i){
		int j=vCons[i];
		pos[j]=0;
	}*/
	/*for (int k1=0;k1<g_numVert;++k1) if (v[k1].d==0){
	for (int i=0;i<g_numVert;++i){
		pos[i]=m_numSS+1;
	}
	pos[7]=pos[13]=pos[22]=pos[37]=pos[43]=0;pos[38]=1;pos[15]=1;
	pos[44]=0;pos[34]=0;
	//pos[k1]=0;
	//pos[k1]=pos[k2]=0;
	if (calBBbound(pos,pos_branch)<2340)
		cout<<k1<<" "<<endl;
	}
	cin.get();

	return;*/
	int num=0;
	int find_local;
	vecqu[num].push(posBB);
	if (posBB.pos_branch>-1&&posBB.cost_bound<m_minCost)
		while (true){
			find_local=0;
			posBB=vecqu[num].top();vecqu[num].pop();
			int i=posBB.pos_branch;
			posBB.pos[i]=0;
			pos_branch=-1;
			NodeBB posBB_tmp(posBB.pos,calBBbound(posBB.pos,pos_branch),pos_branch);
			if (posBB_tmp.pos_branch>-1&&num<g_numVert&&posBB.cost_bound<m_minCost){
				vecqu[num+1].push(posBB_tmp);
			}else if (posBB_tmp.pos_branch<=-1||num==g_numVert) find_local=2;
			else ++find_local;
			posBB.pos[i]=m_numSS;
			pos_branch=-1;
			NodeBB posBB_tmp2(posBB.pos,calBBbound(posBB.pos,pos_branch),pos_branch);
			if (posBB_tmp2.pos_branch>-1&&num<g_numVert&&posBB.cost_bound<m_minCost){
				vecqu[num+1].push(posBB_tmp2);
			}else if (posBB_tmp.pos_branch<=-1||num==g_numVert) find_local=2;
			else ++find_local;
			if (find_local>=2){
				//cout<<num<<endl;
				//cout<<g_count2<<endl;
				//cin.get();
				int next_best=m_minCost,next_num;
				for (int i=0;i<vecqu.size();++i){
					if (!vecqu[i].empty())
						if (next_best>vecqu[i].top().cost_bound){
							next_best=vecqu[i].top().cost_bound;
							next_num=i;
						}
				}
				if (next_best==m_minCost) break;
				else num=next_num;
			}else ++num;
		}
}
int ServerSelectionSolver::calBBbound(vector<unsigned char> pos,int& posBranch){
	vector<int> f(g_numVert,g_pServFast[0].cost);
	vector<double> lambda(g_numVert,0);//>=0
	int step,step_max=15;
	vector<unsigned char> y(g_numVert);//unsure
	int old_mode=m_fast_mode;
	m_fast_mode=3;

	//pos[38]=pos[15]=pos[37]=pos[13]=pos[22]=m_numSS;
	//pos[1]=pos[2]=pos[3]=0;
	//cout<<1.0*f[0]/g_totalDem<<endl;

	//for (int i=0;i<g_numVert;++i)
	//	lambda[i]=1.0*f[0]/m_maxout[i];
	for (int i=0;i<g_numVert;++i)
		if (pos[i]==m_numSS)
			lambda[i]=g_pServFast[0].cost;//very large
		else if (pos[i]<m_numSS)
			lambda[i]=0;
	double cost,bestcost=-1000000;
	step=1;
	do{
		cost=0;
		for (int i=0;i<g_numVert;++i){
			if (pos[i]==m_numSS+1){
				if (f[i]-m_maxout[i]*lambda[i]<0){
					y[i]=1;
					cost+=f[i]-m_maxout[i]*lambda[i];
				}else{
					y[i]=0;//cost+=0;
				}
			}else if (pos[i]==m_numSS){
				y[i]=0;
			}else{
				y[i]=1;
				cost+=f[i]-m_maxout[i]*lambda[i];
			}
		}
		for (int i=0;i<g_numVert;++i){
			g_edge[g_srcEdge[i][0]].c=lambda[i];
		}
		networkSimplexAlg(pos,g_edge);
		for (int i=0;i<g_numVert;++i){
			cost+=g_edge[g_srcEdge[i][0]].x*lambda[i];
		}
		for (int i=0;i<g_m*2;++i){
			if (g_edge[i].x>0)
				cost+=g_edge[i].x*g_edge[i].c;
		}
		for (int i=0;i<g_numVert;++i){
			if (pos[i]==m_numSS+1){
				if (g_edge[g_srcEdge[i][0]].x-m_maxout[i]*y[i]>0)//x0i-D*yi>0
					lambda[i]+=m_lagn[i]/step;
				else if (g_edge[g_srcEdge[i][0]].x-m_maxout[i]*y[i]<0){
					lambda[i]-=m_lagn[i]/step;
					if (lambda[i]<0) lambda[i]=0;
				}else{
					//lambda[i]+=1-rand()%3;
				}
			}
		}
		//cout<<step<<": "<<cost<<endl;
		if (cost>bestcost)
			bestcost=cost;
		else
			++step;
	}while (step<step_max);
	//for (int i=0;i<g_numVert;++i)
	//	cout<<lambda[i]<<" ";
	double max_lambda=0;
	for (int i=0;i<g_numVert;++i)
		if (pos[i]>m_numSS){
			if (lambda[i]>max_lambda){
				max_lambda=lambda[i];
				posBranch=i;
				break;
			}
		}
	for (int i=0;i<g_numVert;++i)
		pos[i]=1-y[i];
	int i;
	for (i=0;i<g_numVert;++i)
		if (lambda[i]==0||g_edge[g_srcEdge[i][0]].x-m_maxout[i]==0);
		else break;
	if (i==g_numVert) posBranch=-2;
	m_fast_mode=old_mode;
	int totalcost=networkSimplexAlg(pos,g_edge);
	calcCost(totalcost,g_edge);
	if (totalcost<m_minCost){
		m_minCost=totalcost;
		m_bestServerPos=pos;
	}
	//if (m_minCost==2042) posBranch=-1;
	cout<<"best: "<<int(bestcost)<<" real cost: "<<totalcost<<" min:"<<m_minCost<<
		" pB:"<<posBranch<<endl;
	return int(bestcost);
}

void ServerSelectionSolver::startSA(int timeS){
	vector<unsigned char> pos(g_numVert,m_numSSFast);
	struct NodeHeuris{
		int d;
		int vid;
	};
	vector<NodeHeuris> v_h(g_numVert);
	int cons_all=0;
	for (int j=1;j<g_numDem+1;++j){
		int i=vCons[j];
		v_h[j-1].d=v[i].d;
		v_h[j-1].vid=i;
	}
	sort(v_h.begin(),v_h.end(),[](const NodeHeuris a,const NodeHeuris b){return a.d>b.d;});
	for (int j=0;j<g_numDem;++j){
		int i=v_h[j].vid;
		cout<<v_h[j].d<<" ";
		pos[i]=m_pos_serv[i];//m_pnumSS[i]-1;
		/*for (int i=0;i<m_pos_serv[i];++i){
			if (v_h[j].d<=g_pServFast[i].cap){
				pos[i]=i;
				break;
			}
		}*/
	}
	/*for (int i=0;i<g_numDem;++i){
		int pos_put=rand()%g_numVert;
		pos[pos_put]=rand()%m_pnumSS[pos_put];
	}*/
	int cost=networkSimplexAlg(pos,g_edge);
	calcCost(cost,g_edge);
	if (cost<m_minCost){
		m_minCost=cost;
		m_bestServerPos=pos;
		cout<<"min0: "<<m_minCost<<endl;
	}
	vector<int> server_level(g_numVert);
	int server_num,old_cost_SA=m_minCost;
	vecRef.resize(g_numVert);
	for (int i=0;i<g_numVert;++i){
		vecRef[i].vid=i;
	}
	updateSAInfo(server_num,server_level,pos);
	double t=100;
	if (g_numVert>=1000)
		t=200;
	int l=1,delta_t,converg=0,step=100;
	do{
		int cost_new;
		for (int k=0;k<l;++k){
			vector<unsigned char> pos_new(pos);
			/*int pos_rand=rand()%g_numVert;
			if (pos_new[pos_rand]==m_numSSFast)
				pos_new[pos_rand]=rand()%m_pnumSS[pos_rand];
			else
				pos_new[pos_rand]=m_numSSFast;*/
			int pos_rand=rand()%server_num+1,i,old_level;
			/*for (i=0;i<g_numVert;++i){
				if (pos_new[i]!=m_numSSFast){
					//if (server_level[i]==m_numSSFast)
					//	while (1) cout<<"??!!"<<i<<" "<<g_count2<<" "<<(int)pos_new[i]<<" "<<server_level[i]<<endl;
					if (--pos_rand==0){
						//old_level=pos_new[i];
						pos_new[i]=m_numSSFast;
						break;
					}
				}
			}*/
			//pos_rand=rand()%(server_num/2)+1;
			i=vecRef[pos_rand-1].vid;
			pos_new[i]=m_numSSFast;
			int pos_rand2,new_i;
			//if (rand()%2||converg<100){
				pos_rand2=rand()%v[i].idEdge.size();
				new_i=g_edge[v[i].idEdge[pos_rand2]].idEnd;
				/*while (v[new_i].idEdge.size()==1&&v[i].idEdge.size()!=1){
					pos_rand2=rand()%v[i].idEdge.size();
					new_i=g_edge[v[i].idEdge[pos_rand2]].idEnd;
				}*/
				if (server_level[new_i]==m_numSSFast){
					if (server_level[i]<m_pos_serv[new_i]+1)//m_pnumSS[new_i])
						pos_new[new_i]=server_level[i];
					else
						pos_new[new_i]=m_pos_serv[new_i];//m_pnumSS[new_i]-1;
				}else{
					int flowall=0;
					for (int k=m_pnumSS[i]-1;k>=0;--k){
						if (g_edge[g_srcEdge[i][k]].x>0){
							flowall+=g_edge[g_srcEdge[i][k]].x;
							if (k>0)
								flowall+=g_pServFast[k-1].cap;
							break;
						}
					}
					for (int k=m_pnumSS[new_i]-1;k>=0;--k){
						if (g_edge[g_srcEdge[new_i][k]].x>0){
							flowall+=g_edge[g_srcEdge[new_i][k]].x;
							if (k>0)
								flowall+=g_pServFast[k-1].cap;
							break;
						}
					}
					pos_new[new_i]=m_pos_serv[new_i];//m_pnumSS[new_i]-1;
					for (int k=0;k<m_pos_serv[new_i]+1;++k){//m_pnumSS[new_i];++k){
						if (flowall<=g_pServFast[k].cap){
							pos_new[new_i]=k;
							break;
						}
					}
					/*if (server_level[i]+1<m_pnumSS[new_i])
						pos_new[new_i]=server_level[i]+1;
					else
						pos_new[new_i]=m_pnumSS[new_i]-1;*/
					/*if (server_level[i]<server_level[new_i])
						pos_new[new_i]=server_level[new_i];
					else
						pos_new[new_i]=server_level[i];*/
				}
			/*}else{
				new_i=i;
				pos_new[new_i]=old_level;
			}*/
			//if (pos_new[new_i{]>(m_numSSFast-1)>>1){
				int kServ=pos_new[new_i],old_cost=~0u>>1;
				for (kServ=kServ;kServ>=0;--kServ){
					pos_new[new_i]=kServ;
					cost_new=judgefunc(pos_new);
					if (old_cost<=cost_new)
						break;
					old_cost=cost_new;
				}
				if (old_cost==~0u>>1) continue;
				pos_new[new_i]=kServ+1;
			//}
			cost_new=judgefunc(pos_new);
			//cout<<cost_new<<endl;
			if (cost_new==~0u>>1){
				continue;
			}
			delta_t=cost_new-old_cost_SA;//m_minCost;//
			//cout<<delta_t<<endl;
			if (delta_t<=0){
				pos=pos_new;
				old_cost_SA=cost_new;
				if (cost_new<m_minCost){
					m_minCost=cost_new;
					m_bestServerPos=pos_new;
					converg=0;
					cout<<"min_new: "<<m_minCost<<endl;
				}
				updateSAInfo(server_num,server_level,pos);
			}else if (rand()<RAND_MAX*exp(-delta_t*1.0/t)){
				pos=pos_new;
				old_cost_SA=cost_new;
				//cout<<cost_new<<endl;
				cout<<step<<endl;
				++step;
				updateSAInfo(server_num,server_level,pos);
			}
			if (clock()-g_tmStart>timeS*CLOCKS_PER_SEC)
				break;
		}
		/*if (1000/step>0)
			t-=1000/step;
		else
			--t;*/
		if (cost_new!=~0u>>1){
			if (g_numVert<1000)
				t*=0.9999;
			else
				t*=0.9996;
		}
		++converg;
		if (clock()-g_tmStart>timeS*CLOCKS_PER_SEC)
			break;
	}while (t>0);//&&converg<100);
	cout<<cost<<" T="<<t<<" converg="<<converg<<endl;
}
int ServerSelectionSolver::judgefunc(vector<unsigned char>& pos){
	int cost=networkSimplexAlg(pos,g_edge);
	if (cost==~0u>>1) return cost;
	/*for (int i=0;i<g_edgeCoreCount;++i)
		g_edgeTmp[i]=g_edge[i];
	processNetwork(pos,cost,g_edgeTmp,false,1);
	calcCost(cost,g_edgeTmp);*/
	//calcCost(cost,g_edge);
	cost=getRawCost(pos);
	return cost;
}
void ServerSelectionSolver::makejude(vector<int>& m_judgePos){
	int min=~0u>>1,max=0;
	m_judgesum=0;
	m_judgePos.resize(g_numVert);
	for (int i=0;i<g_numVert;++i){
		m_judgePos[i]=v[i].d+m_maxout[i];
		if (max<m_judgePos[i])
			max=m_judgePos[i];
		if (min>m_judgePos[i])
			min=m_judgePos[i];
	}
	for (int i=0;i<g_numVert;++i){
		m_judgePos[i]=1.0*(m_judgePos[i]-min)/(max-min)*(RAND_MAX*8.0/10)+RAND_MAX*1.0/10;
		//cout<<m_judgePos[i]*1.0/RAND_MAX<<" ";
		m_judgesum+=m_judgePos[i];
	}
	cout<<endl;
	int del_num=0;
	for (int i=0;i<g_numVert;++i){
		if (v[i].id==0){
			//cout<<v[i].idEdge.size()<<" ";
			if (v[i].idEdge.size()==1){
				++del_num;
				m_fixedServerPos[i]=m_numSSFast;
			}
		}
	}
	cout<<endl;
	for (int i=1;i<g_numDem+1;++i){
		//cout<<v[vCons[i]].idEdge.size()<<" ";
		if (v[i].idEdge.size()==1){
			++del_num;
			m_fixedServerPos[i]=m_numSSFast;
		}
	}
	cout<<del_num<<endl;
	cout<<endl;
}
void ServerSelectionSolver::updateSAInfo(int& server_num,vector<int>& server_level,vector<unsigned char>& pos){
	server_num=0;
	for (int i=0;i<g_numVert;++i){
		if (g_edge[g_srcEdge[i][0]].x>0){
			if (pos[i]!=m_numSSFast){
				if (m_fast_mode==1){
					for (int k=0;k<=m_pos_serv[i];++k){
						if (g_edge[g_srcEdge[i][0]].x<=g_pServFast[k].cap){
							server_level[i]=k;
							if (pos[i]>k) --pos[i];
							else if (pos[i]<k) pos[i]=k;
							break;
						}
					}
				}else
					for (int k=m_pnumSS[i]-1;k>=0;--k){
						if (g_edge[g_srcEdge[i][k]].x>0){
							server_level[i]=k;
							if (pos[i]>k) --pos[i];
							else if (pos[i]<k) pos[i]=k;
							break;
						}
					}
				++server_num;
			}else{
				//while (1) cout<<"??"<<endl;
				if (m_fast_mode==1){
					for (int k=0;k<=m_pos_serv[i];++k){
						if (g_edge[g_srcEdge[i][0]].x<=g_pServFast[k].cap){
							server_level[i]=k;
							if (pos[i]>k) --pos[i];
							break;
						}
					}
				}else
					for (int k=m_pnumSS[i]-1;k>=0;--k){
						if (g_edge[g_srcEdge[i][k]].x>0){
							server_level[i]=k;
							if (pos[i]>k) --pos[i];
							break;
						}
					}
				++server_num;
			}
		}else{
			if (pos[i]!=m_numSSFast){
				server_level[i]=m_numSSFast-1;
				//server_level[i]=0;
				++server_num;
			}else server_level[i]=m_numSSFast;
		}
	}
	for (int j=0;j<g_numVert;++j){
		int i=vecRef[j].vid;
		if (g_edge[g_srcEdge[i][0]].x>0){
			for (int k=m_pnumSS[i]-1;k>=0;--k){
				if (g_edge[g_srcEdge[i][k]].x>0){
					vecRef[j].x=g_edge[g_srcEdge[i][k]].x;
					if (k>0)
						vecRef[j].x+=g_pServFast[k-1].cap;
					break;
				}
			}
		}else{
			vecRef[j].x=0;
		}
	}
	sort(vecRef.begin(),vecRef.end(),[&pos,this](NodePoint& a,NodePoint& b){
		if (a.x==0&&b.x==0){
			if (pos[a.vid]!=m_numSSFast)
				return true;
			else
				return false;
		}else
			return a.x>b.x;
	});
}