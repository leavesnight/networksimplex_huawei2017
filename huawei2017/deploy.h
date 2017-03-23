#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib_io.h"

#include <set>
#include <vector>
#include <iostream>
using namespace std;

void deploy_server(char * graph[MAX_EDGE_NUM], int edge_num, char * filename);

class Genome{
public:
	friend class GenAlg;
	static const int m_intBitNS=sizeof(int)*8-1;
	Genome():m_fitness(0){}
	Genome(vector<int> vec,double f=0):m_genome(vec),m_fitness(f){}
	Genome(set<int> set,double f=0,int size=1):m_genome(size,0),m_fitness(f){
		for (auto i=set.begin();i!=set.end();i++){
			if (m_genome.size()<*i/m_intBitNS+1)
				m_genome.resize(*i/m_intBitNS+1,0);
			m_genome[*i/m_intBitNS]|=0x1<<*i%m_intBitNS;
		}
	}
	set<int> decodeToSet(){
		set<int> setTmp;
		for (int i=0;i<m_genome.size();i++){
			for (int j=0;j<m_intBitNS;j++){
				if (m_genome[i]&0x1<<j){
					setTmp.insert(i*m_intBitNS+j);
				}
			}
		}
		return setTmp;
	}
	void print(){
		for (int i=0;i<m_genome.size();i++){
			cout<<m_genome[i]<<" ";
		}
		cout<<endl;
	}
private:
	vector<int> m_genome;
	double m_fitness;
};
class GenAlg{
private:
	vector<Genome> m_pop;
	int m_popSize;//the size of population
	int m_chromoBitSize;//from 1;0 means 0bit can be the server while we won't choose it;here it should be n
	double m_totalFit;
	double m_bestFit;
	Genome m_bestGenome;
	int m_mutationRate;
	int m_crossoverRate;
	int m_cServer,m_maxFit;
	int m_convergCount;
	const int m_conCMax;
public:
	GenAlg(int bitSize,int popSize,int nd,int cServer);
	void calcFit(int timeS);
	Genome& getChromoRoulette();
	void crossover(vector<int>& chromo1,vector<int>& chromo2);
	void mutate(vector<int>& chromo);
	void startGA(int timeS);//max run time(S)
	int getMinCost(){
		return m_maxFit-m_bestFit;
	}
	set<int> getBestServerPos(){
		m_bestGenome.print();
		return m_bestGenome.decodeToSet();
	}
};

#endif
