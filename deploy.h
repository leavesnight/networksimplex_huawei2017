#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib_io.h"

#include <set>
#include <vector>
#include <iostream>
#include <cmath>
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
			if ((int)m_genome.size()<*i/m_intBitNS+1)
				m_genome.resize(*i/m_intBitNS+1,0);
			m_genome[*i/m_intBitNS]|=0x1<<*i%m_intBitNS;
		}
	}
	set<int> decodeToSet(){
		set<int> setTmp;
		for (int i=0;i<(int)m_genome.size();i++){
			for (int j=0;j<m_intBitNS;j++){
				if (m_genome[i]&0x1<<j){
					setTmp.insert(i*m_intBitNS+j);
				}
			}
		}
		return setTmp;
	}
	void print(){
		for (int i=0;i<(int)m_genome.size();i++){
			cout<<m_genome[i]<<" ";
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
	int m_conCMax;

	vector<Genome> m_pBest;
	vector<Genome> m_v;
public:
	GenAlg(int bitSize,int popSize,int nd,int cServer,set<int> greedyPos);
	void calcFit(int timeS);
	Genome& getChromoRoulette();
	void crossover(vector<int>& chromo1,vector<int>& chromo2,double crossoverRate,int type=1);
	void mutate(vector<int>& chromo);
	void startGA(int timeS);//max run time(S)

	void startPSO(int timeS,int w,int c1,int c2,int type=0);
	void calcPGBest(int timeS,int type=0);

	int getMinCost(){
		return m_maxFit-(int)m_bestFit;
	}
	set<int> getBestServerPos(){
		m_bestGenome.print();
		return m_bestGenome.decodeToSet();
	}
};
template<class T> struct NodeHeap{
	T val;
	size_t mk;
	NodeHeap():val(0){}
	NodeHeap(const T& _val):val(_val){}
	NodeHeap(const NodeHeap& _nodeheap):val(_nodeheap.val),mk(_nodeheap.mk){}
	inline bool operator<(const NodeHeap& _nodeheap){
		return val<_nodeheap.val;
	};
};
template<class T,class Comp> class ZkwHeap{
private:
	typedef NodeHeap<T> Node;
	typedef ZkwHeap<T,Comp> Heap;
	
	Comp cmp;
	Node* NodeList;
	size_t n;//max size of the array
	T init_val;

	void fix(size_t _pos){
		if (cmp(NodeList[_pos<<1].val,NodeList[(_pos<<1)+1].val))
			NodeList[_pos]=NodeList[(_pos<<1)+1];
		else
			NodeList[_pos]=NodeList[_pos<<1];
	}
public:
	ZkwHeap(const T& _init_val=~0u>>1):init_val(_init_val){//small heap is default
		NodeList=new Node[1];
	}
	void initSize(const size_t& _maxn){
		n=1<<1+(size_t)(log(_maxn)/log(2));
		delete[] NodeList;
		NodeList=new Node[n<<1];
		for (size_t i=1;i<n+n;++i)
			NodeList[i].val=init_val;
		for (size_t i=n;i<n+n;++i)
			NodeList[i].mk=i-n+1;//from 1 to n
	}
	ZkwHeap(const size_t& _maxn,const T& _init_val):init_val(_init_val),
		n(1<<1+(size_t)(log(_maxn)/log(2))){
		NodeList=new Node[n<<1];
		for (size_t i=1;i<n+n;++i)
			NodeList[i].val==init_val;
		for (size_t i=n;i<n+n;++i)
			NodeList[i].mk=i-n+1;//from 1 to n
	}
	~ZkwHeap(){
		delete[] NodeList;
	}
	T top(){
		return NodeList[1].val;
	}
	T top_pos(){
		return NodeList[1].mk;
	}
	void modify(size_t _pos,const T& _new_val){//mark with 1~n input
		int pos=_pos+n-1;
		NodeList[pos].val=_new_val;
		while (pos)
			fix(pos>>=1);
	}
	T pop(){
		T retVal=NodeList[1].val;
		modify(NodeList[1].mk,init_val);
		return retVal;
	}
};
/*template<class T,class Comp> class hwHeap{
private:
	vector<NodeHeap<T>> heap;
	T init_val;
	Com cmp;
public:
	hwHeap(const T& _init_val):init_val(_init_val){
	}
};*/

#endif
