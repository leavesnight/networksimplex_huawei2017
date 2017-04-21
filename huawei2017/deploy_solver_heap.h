#ifndef __SOLVERHEAP_H__
#define __SOLVERHEAP_H__

#include <cmath>
using namespace std;

template<class T> struct NodeHeap{
	T val;
	size_t mk;
	NodeHeap():val(0){}
	NodeHeap(const T& _val):val(_val){}
	NodeHeap(const NodeHeap& _nodeheap):val(_nodeheap.val),mk(_nodeheap.mk){}
	inline bool operator<(const NodeHeap& _nodeheap){
		return val<_nodeheap.val;
	}
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