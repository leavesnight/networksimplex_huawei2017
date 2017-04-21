#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib_io.h"
#include "deploy_solver.h"

#include <set>
#include <vector>
#include <iostream>
using namespace std;

void deploy_server(char * graph[MAX_EDGE_NUM], int edge_num, char * filename);

class Genome{
public:
	friend class GenAlg;
	static const int m_intBitNS=sizeof(int)*8-1;
	Genome(double f=0):m_fitness(f){}
	Genome(vector<unsigned char>& pos,double f=0):m_fitness(f){
		m_genome.insert(m_genome.begin(),pos.begin(),pos.end());
	}
	vector<unsigned char> decodeToVec(){return m_genome;}
	void print(){
		for (int i=0;i<(int)m_genome.size();++i){
			cout<<(int)m_genome[i]<<" ";
		}
		cout<<endl;
	}
	bool operator==(const Genome& genome){
		return genome.m_fitness==m_fitness&&genome.m_genome==m_genome;
	}
	bool operator!=(const Genome& genome){
		return genome.m_fitness!=m_fitness||genome.m_genome!=m_genome;
	}
private:
	vector<unsigned char> m_genome;
	double m_fitness;
};
class GenAlg{
private:
	ServerSelectionSolver* psssr;
	vector<Genome> m_pop;
	int m_popSize;//the size of population
	int m_chromoBitSize;//from 1;0 means 0bit can be the server while we won't choose it;here it should be n
	double m_totalFit;
	Genome m_bestGenome;
	int m_mutationRate;
	int m_crossoverRate;
	int m_maxFit;
	int m_convergCount;
	int m_conCMax;

	vector<Genome> m_pBest;
	vector<Genome> m_v;
public:
	GenAlg(int popSize,ServerSelectionSolver* _psssr,vector<unsigned char>& greedyPos);
	void calcFit(int timeS);
	Genome& getChromoRoulette();
	void crossover(vector<unsigned char>& chromo1,vector<unsigned char>& chromo2,double crossoverRate,int type=1);
	void mutate(vector<unsigned char>& chromo);
	void startGA(int timeS);//max run time(S)

	void startPSO(int timeS,int w,int c1,int c2,int type=0);
	void calcPGBest(int timeS,int type=0);

	int getMinCost(){
		return m_maxFit-(int)m_bestGenome.m_fitness;
	}
	vector<unsigned char> getBestServerPos(){
		return m_bestGenome.m_genome;
	}
};

#endif
