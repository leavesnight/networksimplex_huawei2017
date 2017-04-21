#include "deploy.h"
#include "deploy_solver.h"
//#include <limits.h>
#include <algorithm>
#include <random>
//#include <iterator>
using namespace std;

clock_t g_tmStart=clock();
int g_count,g_count2;

//你要完成的功能总入口
void deploy_server(char * topo[MAX_EDGE_NUM], int line_num,char * filename)
{
	// 需要输出的内容
	ServerSelectionSolver sssolver(topo,line_num);//,true);
	if (sssolver.isapNA()){
		write_result("NA", filename);
		return;
	}
	//g_tmStart=clock();
	srand((int)time(0));
	sssolver.startSolver(88);
	//sssolver.xjbs();

	/*if (g_numVert==0&&g_numDem>0)//customers' need exists while no nodes, impossible
		strTopo="NA";
	else{*/

	// 直接调用输出文件的方法输出到指定文件中(ps请注意格式的正确性，如果有解，第一行只有一个数据；第二行为空；第三行开始才是具体的数据，数据之间用一个空格分隔开)
	const char* topo_file=sssolver.getResult();
	cout<<g_count<<endl<<"simplex num: "<<g_count2<<endl;
	cin.get();
	write_result(topo_file, filename);

}

GenAlg::GenAlg(int popSize,ServerSelectionSolver* _psssr,vector<unsigned char>& greedyPos):
	m_mutationRate((int)(RAND_MAX*0.2)),m_crossoverRate((int)(RAND_MAX*0.9)),//70%?80~95%?
	psssr(_psssr),m_chromoBitSize(_psssr->g_numVert),//0.05~0.3?0.5~1%?
	m_maxFit(~0u>>1),m_conCMax(200){//?
	//m_maxFit=psssr->m_minCost;
	m_popSize=0;
	m_pop.push_back(Genome(greedyPos));
	++m_popSize;
	while (m_popSize<popSize){
		vector<unsigned char> posTmp(m_chromoBitSize,psssr->m_numSSFast);
		for (int i=0;i<=rand()%m_chromoBitSize;++i){
			int pos=rand()%m_chromoBitSize;
			while (psssr->m_fixedServerPos[pos]<psssr->m_numSSFast+1){
				pos=rand()%m_chromoBitSize;
			}
			posTmp[pos]=rand()%psssr->m_pnumSS[pos];
		}
		//setTmp.insert(sFixedServerPos.begin(),sFixedServerPos.end());
		Genome tmpGenome(posTmp);
		m_pop.push_back(tmpGenome);
		++m_popSize;
		//tmpGenome.print();
	}
}
void GenAlg::calcFit(int timeS){
	m_totalFit=0;
	for (int i=0;i<m_popSize;++i){
		vector<unsigned char> posTmp=m_pop[i].decodeToVec();
		psssr->networkSimplexAlg(posTmp,psssr->g_edge);
		int nTotalCost=0,nTotalNum=0;
		for (int i=0;i<psssr->g_edgeCoreCount;++i){
			psssr->g_edgeTmp[i].x=psssr->g_edge[i].x;
		}
		psssr->processNetwork(posTmp,nTotalCost,psssr->g_edgeTmp,true);
		//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
		//nTotalCost=0;nTotalNum=setTmp.size();
		psssr->calcCost(nTotalCost,psssr->g_edgeTmp);//false
		if (nTotalCost<m_maxFit){
			m_pop[i]=Genome(posTmp,m_maxFit-nTotalCost);
			//m_pop[i].m_fitness=m_maxFit-nTotalCost;
			if (m_pop[i].m_fitness>m_bestGenome.m_fitness){
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
void GenAlg::crossover(vector<unsigned char>& chromo1,vector<unsigned char>& chromo2,double crossoverRate,int type){
	if (rand()<crossoverRate){
		int pos=rand()%m_chromoBitSize;
		int pos2=rand()%m_chromoBitSize;
		if (type==0)
			pos2=0;
		if (pos<pos2){
			int nTmp=pos;
			pos=pos2;
			pos2=nTmp;
		}
		for (int i=pos2;i<=pos;++i){
			int nTmp=chromo1[i];
			chromo1[i]=chromo2[i];
			chromo2[i]=nTmp;
		}
	}
}
void GenAlg::mutate(vector<unsigned char>& chromo){
	if (rand()<m_mutationRate){
		int i=rand()%m_chromoBitSize;
		if (chromo[i]==psssr->m_numSSFast)
			chromo[i]=rand()%psssr->m_pnumSS[i];
		else
			chromo[i]=psssr->m_numSSFast;
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
	m_pBest=m_pop;
	if (psssr->g_numVert>200)
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
				int pos=rand()%m_chromoBitSize,pos2=rand()%m_chromoBitSize;
				while (psssr->m_fixedServerPos[pos]<psssr->m_numSSFast+1){
					pos=rand()%m_chromoBitSize;
				}
				while (psssr->m_fixedServerPos[pos2]<psssr->m_numSSFast+1){
					pos2=rand()%m_chromoBitSize;
				}
				if (pos<pos2){
					int nTmp=pos;
					pos=pos2;
					pos2=nTmp;
				}
				int nTmp=m_pop[i].m_genome[pos];
				m_pop[i].m_genome[pos]=m_pop[i].m_genome[pos2];
				m_pop[i].m_genome[pos2]=nTmp;
			}
			crossover(genomeP.m_genome,m_pop[i].m_genome,c1,0);
			if (rand()%2==0)
				m_pop[i]=genomeP;
			crossover(genomeG.m_genome,m_pop[i].m_genome,c2);
			if (rand()%2==0)
				m_pop[i]=genomeG;
			++i;
		}
		//type=3;
		int loop=0;
		Genome genomeLS=m_bestGenome;
		do{
			Genome genomeL=genomeLS;
			int pos,test=0;
			for (int i=0;i<2;++i){
				pos=rand()%m_chromoBitSize;
				while (psssr->m_fixedServerPos[pos]<psssr->m_numSSFast+1){
					pos=rand()%m_chromoBitSize;
				}
				if (genomeL.m_genome[pos]==psssr->m_numSSFast){
					genomeL.m_genome[pos]=rand()%psssr->m_pnumSS[pos];
					test=1;
				}else
					genomeL.m_genome[pos]=psssr->m_numSSFast;
			}
			vector<unsigned char> posTmp=genomeL.m_genome;
			int nTmTmp=clock();
			int nTotalCost=psssr->networkSimplexAlg(posTmp,psssr->g_edge);
			g_count+=clock()-nTmTmp;
			if (type==1){
				for (int i=0;i<psssr->g_edgeCoreCount;++i){
					psssr->g_edgeTmp[i].x=psssr->g_edge[i].x;
				}
				psssr->processNetwork(posTmp,nTotalCost,psssr->g_edgeTmp,true,0);
				psssr->calcCost(nTotalCost,psssr->g_edgeTmp);//false
				//psssr->calcCost(nTotalCost,psssr->g_edge);
			}else{
				nTotalCost=psssr->getRawCost(posTmp);
			}
			if (m_maxFit-nTotalCost>=genomeLS.m_fitness){
				genomeLS=Genome(posTmp,m_maxFit-nTotalCost);
			}else
				loop++;
			if (clock()-g_tmStart>=timeS*CLOCKS_PER_SEC)
				break;
		}while (loop<m_chromoBitSize);
		if (genomeLS.m_fitness>=m_bestGenome.m_fitness){
			if (genomeLS!=m_bestGenome){
				m_convergCount=0;
				m_bestGenome=genomeLS;
				cout<<"LS:　"<<m_maxFit-m_bestGenome.m_fitness<<endl;
			}
		}
	}
}
void GenAlg::calcPGBest(int timeS,int type){
	for (int i=0;i<m_popSize;++i){
		vector<unsigned char> posTmp=m_pop[i].m_genome;
		int nTmTmp=clock();
		int nTotalCost=psssr->networkSimplexAlg(posTmp,psssr->g_edge);
		g_count+=clock()-nTmTmp;
		if (type==1){
			for (int i=0;i<psssr->g_edgeCoreCount;++i){
				psssr->g_edgeTmp[i].x=psssr->g_edge[i].x;
			}
			psssr->processNetwork(posTmp,nTotalCost,psssr->g_edgeTmp,true,0);
			//networkSimplexAlg(m_chromoBitSize,m_cServer,setTmp,g,v);
			//nTotalCost=0;nTotalNum=setTmp.size();
			psssr->calcCost(nTotalCost,psssr->g_edgeTmp);//false
			//psssr->calcCost(nTotalCost,psssr->g_edge);
		}else if (type==0){
			psssr->processNetwork(posTmp,nTotalCost,psssr->g_edge,true,0);
			psssr->networkSimplexAlg(posTmp,psssr->g_edge);
			//g_count+=clock()-nTmTmp;
			nTotalCost=0;
			psssr->calcCost(nTotalCost,psssr->g_edge,false);
		}else{
			nTotalCost=psssr->getRawCost(posTmp);
		}
		if (nTotalCost<m_maxFit){
			m_pop[i]=Genome(posTmp,m_maxFit-nTotalCost);
			//m_pop[i].m_fitness=m_maxFit-nTotalCost;
			if (m_pop[i].m_fitness>m_pBest[i].m_fitness){
				m_pBest[i]=m_pop[i];
				if (m_pop[i].m_fitness>m_bestGenome.m_fitness){
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

/*
Flow& randomwalk(){
	vector<unsigned char> dir_servers;
	for (int i=0;i<ssssr->g_numVert;++i){
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
*/
/*auto dupItr=unique(dudes.begin(),dudes.end());
		int delSize=dudes.end()-dupItr;
		cout<<delSize<<endl;
		while (delSize>0){
			delete dudes.back().flow;
			dudes.pop_back();
			--delSize;
		}*/

struct xjb_dude{
	ServerSelectionSolver* psssr;
	xjb_dude(vector<unsigned char>& f_,ServerSelectionSolver* _psssr):psssr(_psssr){
        f=f_;
		Flow flow(f,_psssr);
        v=flow.val();
		f2=flow.servers;
        age=0;
        contri=0;
    }
    vector<unsigned char> f;
	vector<unsigned char> f2;
    int v;
    int age;
    xjb_dude*pr;
    int contri;
	bool operator==(const xjb_dude&b)const{
		return this->f==b.f&&this->v==b.v;
	}
	bool operator!=(const xjb_dude&b)const{
		return !(*this==b);
	}
	bool operator<(const xjb_dude& b)const{
		if(this->v!=b.v)
			return this->v<b.v;
		for(int i=0;this->psssr->g_numVert;++i)
			if(this->f[i]!=b.f[i])
				return this->f[i]<b.f[i];
		if(this->age!=b.age)
			return this->age>b.age;
		return false;
	}
};
bool cmp(xjb_dude*a,xjb_dude*b){
	return *a<*b;
}
struct xjb_set{
	ServerSelectionSolver* psssr;
    xjb_set(vector<unsigned char>& f_,int v_,ServerSelectionSolver* _psssr):psssr(_psssr){
        f=f_;
        v=v_;
    }
    vector<unsigned char> f;
    int v;
	bool operator<(const xjb_set&b) const{
		if(this->v!=b.v)
			return this->v<b.v;
		for(int i=0;i<this->psssr->g_numVert;++i)
			if(this->f[i]!=b.f[i])
				return this->f[i]<b.f[i];
		return false;
	}
};

void ServerSelectionSolver::xjb_search(int timeS){
    vector<unsigned char> nServerPos(g_numVert,m_numSSFast);
    set<xjb_set>hash;
	bool time_up=false;
	for (int i=0;i<g_numVert;++i)
		if (v[i].id)
			nServerPos[i]=m_pnumSS[i]-1;
	int z_seed=Flow(nServerPos,this).val(),z_best,z_seed_before=z_seed;
	cout<<"Direct Seed: "<<z_seed<<endl;
	for(int changed=1,tm=0;changed;){
		changed=0;
		for(int i=0;i<g_numVert;++i){
			bool change_state=false;
			int old_state=nServerPos[i];
			nServerPos[i]=1;//m_pnumSS[i];
			while (!change_state){
				++tm;
				if (old_state<m_numSSFast){
					nServerPos[i]=m_numSSFast;
					change_state=true;
				}else{
					--nServerPos[i];
					//if (nServerPos[i]==0)
						change_state=true;
				}
				int t=Flow(nServerPos,this).val();
				if(t<z_seed){
					z_seed=t;
					changed=1;
					old_state=nServerPos[i];
					if (nServerPos[i]<m_numSSFast&&nServerPos[i]>0)
						change_state=false;
					cout<<z_seed<<" "<<i<<" "<<nServerPos[i]<<endl;
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
    vector<xjb_dude*> dudes;
    dudes.push_back(new xjb_dude(nServerPos,this));
	int cost=0;
	processNetwork(nServerPos,cost,g_edge,true,0);
    m_bestServerPos=nServerPos;
	m_minCost=z_seed;
    z_best=z_seed;
    int max_qu=2;
    for(int it=0;;++it){
        for(int i=dudes.size()-1;i>=0;--i){
			vector<unsigned char> f=dudes[i]->f;
			++dudes[i]->age;
			int pos=rand()%g_numVert;
			while(f[pos]==m_numSSFast&&rand()%3){
				pos=rand()%g_numVert;
			}
			if (f[pos]==m_numSSFast)
				f[pos]=0;//rand()%m_pnumSS[pos];
			else
				f[pos]=m_numSSFast;
			auto dude=new xjb_dude(f,this);
			if(!hash.count(xjb_set(dude->f,dude->v,this))){
				dudes.push_back(dude);
				dudes.back()->pr=dudes[i];
			}else{
				delete dude;
			}
        }
        sort(dudes.begin(),dudes.end(),cmp);
        vector<xjb_dude*> new_dudes;
        for(int i=0;i<dudes.size();++i){
            if(i==0||*dudes[i]!=*dudes[i-1])
                new_dudes.push_back(dudes[i]);
            else{
                delete dudes[i];
			}
        }
        //for(int i=0;i<new_dudes.size()&&i<max_qu;++i)
        //    if(new_dudes[i]->age==0)
        //        ++new_dudes[i]->pr->contri;
        dudes.clear();
        for(int i=0;i<new_dudes.size();++i){
            if(new_dudes[i]->age>80){
                hash.insert(xjb_set(new_dudes[i]->f,new_dudes[i]->v,this));
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
            m_bestServerPos=dudes[0]->f2;
			m_minCost=z_best;
        }
        if (it%int(40000.0/g_numVert+1)==0){
            cout<<"Iteration "<<it+1<<": ";
            for(int i=0;i<min((int)dudes.size(),5);++i)
                cout<<dudes[i]->v<<","<<"???"<<","<<dudes[i]->age<<","<<dudes[i]->contri<<" ";
            cout<<endl;
			//cout<<hash.size()<<endl;
            //cout<<endl;
            //cout<<"Best solution: "<<z_best<<endl<<endl;
        }
        if (clock()-g_tmStart>timeS*CLOCKS_PER_SEC){
            break;
        }
    }
}
