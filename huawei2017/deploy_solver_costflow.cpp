#include "deploy_solver.h"

#include <iostream>
using namespace std;

bool ServerSelectionSolver::isapNA(){
	ISAP<int> isap(g_numVert+m_numSS+2,g_numVert+m_numSS,g_numVert+m_numSS+1);
	for (int i=0;i<g_edgeCount-g_numDem-1;++i)
		isap.add(g_edge[i].idBegin,g_edge[i].idEnd,g_edge[i].u);
	for (int i=1;i<g_numDem+1;++i)
		isap.add(vCons[i],g_numVert+m_numSS+1,v[vCons[i]].d);
	int flow=isap.run();
	if (flow<g_totalDem) return true;
	/*int i;
	for (i=0;i<g_edgeCount-g_numVert*m_numSS;++i){
		int tmp1=isap.egs[i*2].l-isap.egs[i*2].c;
		int tmp2=isap.egs[(i^1)*2^1].l-isap.egs[(i^1)*2^1].c;
		g_edge[i].x=(tmp1>0?tmp1:0)+(tmp2>0?tmp2:0);
	}
	for (i=i;i<g_edgeCount;++i){
		int tmp1=isap.egs[i*2].l-isap.egs[i*2].c;
		g_edge[i].x=tmp1>0?tmp1:0;
	}*/
	netSAinit(g_edge);
	m_bestServerPos.resize(g_numVert,m_numSSFast);//with k==0 will also get an feasible answer
	m_minCost-=1;
	//networkSimplexAlg(m_bestServerPos,g_edge);
	//calcCost(m_minCost=0,g_edge);
	return false;
}
void ServerSelectionSolver::netSAinit(NodeEdge g_edge[]){//completely reset
	//make an initial feasible solution
	for (int i=0;i<g_edgeCount-g_numDem-1;++i){
		g_edge[i].x=0;
	}
	g_edge[g_edgeCount-1].x=g_totalDem;
	int edge_id=g_edgeCount-g_numDem-2;
	for (int i=1;i<g_numDem+1;++i){
		g_edge[++edge_id].x=v[vCons[i]].d;
	}
	//make an initial strongly feasible tree
	int bss=m_root-1;//big M point must be the m_root
	v[m_root].parent=m_root;//direct can be arbitrary
	v[m_root].depth=0;
	v[m_root].next=vCons[1];
	v[vCons[1]].prec=m_root;//!!!very important initial
	for (int j=1;j<g_numDem+1;++j){
		int i=vCons[j];
		v[i].parent=m_root;//all consumer points' parent is m_root && v[m_root].depth=0
		v[i].directToParent=false;//v[i].direction=false cannot be omitted!!!
		v[i].depth=1;//v[m_root].depth=0, depth is from 0
		if (j<g_numDem){
			v[i].next=vCons[j+1];//i's next dfs node is vCons[j+1] except vn.next
			v[vCons[j+1]].prec=i;//notice it may over the border!!!
		}else{
			v[vCons[g_numDem]].next=bss;
			v[bss].prec=vCons[g_numDem];
		}
	}
	v[bss].parent=m_root;
	v[bss].directToParent=true;
	v[bss].depth=1;
	v[bss].next=g_numVert;//next is ss(k==0)
	v[g_numVert].prec=bss;
	int ik_final=g_numVert+m_numSS-1;
	for (int ik=g_numVert;ik<g_numVert+m_numSS;++ik){
		v[ik].parent=bss;
		v[ik].directToParent=false;//v[i].direction=false cannot be omitted!!!
		v[ik].depth=2;//v[bss].depth=1, depth is from 0
		if (ik<ik_final){
			v[ik].next=ik+1;
			v[ik+1].prec=ik;//notice it may over the border!!!
		}else{
			v[ik_final].next=m_root;//!here is important
			v[m_root].prec=ik_final;
		}
	}
	int i_inter;
	for (i_inter=0;i_inter<g_numVert;++i_inter)
		if (!v[i_inter].id) break;
	if (i_inter<g_numVert){//else they are all consumer points
		ik_final=g_numVert;//at least i has a server as k=0!
		v[ik_final].next=i_inter;
		v[i_inter].prec=ik_final;
		int i=i_inter;
		while (i<g_numVert){
			for (i_inter=i+1;i_inter<g_numVert;++i_inter)
				if (!v[i_inter].id) break;
			v[i].parent=ik_final;
			v[i].directToParent=false;//v[i].direction=false cannot be omitted!!!
			v[i].depth=3;//v[ik_final].depth=2, depth is from 0
			if (i_inter<g_numVert){
				v[i].next=i_inter;
				v[i_inter].prec=i;//notice it may over the border!!!
			}else{
				if (m_numSS==1){
					v[i].next=m_root;
					v[m_root].prec=i;
				}else{
					v[i].next=ik_final+1;//notice it may over the border!
					v[ik_final+1].prec=i;
				}
			}
			i=i_inter;
		}
	}
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
}
int ServerSelectionSolver::networkSimplexAlg(vector<unsigned char>& pos,NodeEdge g_edge[]){
	int nTmp,nTmp2,nTmp3;
	g_count2++;
	//make an initial strongly feasible tree or solution
	if (m_fast_mode==1){
		for (int i=0;i<g_numVert;++i){
			if (pos[i]==m_numSSFast){
				g_edge[g_srcEdge[i][0]].c=m_maxCS;//g_pServFast[m_pos_serv[i]].cost+g_pExtraCost[i];
				//g_edge[g_srcEdge[i][0]].u=g_pServFast[m_pos_serv[i]].cap;
			}else{//<m_numSS here is a pos[i] type server
				g_edge[g_srcEdge[i][0]].c=0;
				/*if (pos[i]<m_pos_serv[i])
					g_edge[g_srcEdge[i][0]].u=g_pServFast[pos[i]].cap;
				else
					g_edge[g_srcEdge[i][0]].u=g_pServFast[m_pos_serv[i]].cap;*/
			}
		}
		//netSAinit(g_edge);
	}else if (m_fast_mode<3)
		for (int i=0;i<g_numVert;++i){
			if (pos[i]==m_numSSFast){
				for (int k=0;k<m_pnumSS[i];++k)
					g_edge[g_srcEdge[i][k]].c=g_pServFast[k].cost+g_pExtraCost[i];
			}else{//<m_numSS here is a pos[i] type server
				if (pos[i]<m_pnumSS[i]){
					for (int k=0;k<=pos[i];++k)
						g_edge[g_srcEdge[i][k]].c=0;
					for (int k=pos[i]+1;k<m_pnumSS[i];++k)
						g_edge[g_srcEdge[i][k]].c=g_pServFast[k].cost-g_pServFast[pos[i]].cost;
				}else
					for (int k=0;k<m_pnumSS[i];++k)
						g_edge[g_srcEdge[i][k]].c=0;
			}
		}
	int d;
	for (int i=v[m_root].next;i!=m_root;i=v[i].next){
		/*if (v[i].parent==m_root){
			if (v[i].directToParent)
				d=g_edge[g[i][m_root]].c-v[i].pi;
			else
				d=-g_edge[g[m_root][i]].c-v[i].pi;
		}
		v[i].pi+=d;*/
		if (v[i].directToParent)
			v[i].pi=g_edge[g[i][v[i].parent]].c+v[v[i].parent].pi;
		else
			v[i].pi=-g_edge[g[v[i].parent][i]].c+v[v[i].parent].pi;
	}
	//find the maximum residual reduced cost(-cπij/cπij for L/U)
	/*for (int i=0;i<g_edgeCount;++i){
		g_edge[i].cpi=g_edge[i].c-v[g_edge[i].idBegin].pi+v[g_edge[i].idEnd].pi;
		if (g_bUseHeap) zkwtreeModify(i);
	}*/
	for (int i=0;i<g_m;++i){
		changeEdgeUnD(i,g_edge);
	}
	for (int i=g_m+g_m;i<g_edgeCount;++i){
		changeEdgeDUnD(i,g_edge);
	}
	//int iMapEdgeUnD=g_m;
	int	max;
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
			if (m_fast_mode==2){
				for (int i=0;i<g_edgeCoreUnDCount;++i){
					if (g_edgeUnD[i].cpi>max){//this is a modified cpi!~
						max=g_edgeUnD[i].cpi;
						p=g_edgeUnD[i].idBegin;q=g_edgeUnD[i].idEnd;
					}
				}
			}else if (m_fast_mode==1){
				for (int i=0;i<g_edgeUnDCount;++i){
					if (g_edgeUnD[i].cpi>max){//this is a modified cpi!~
						max=g_edgeUnD[i].cpi;
						p=g_edgeUnD[i].idBegin;q=g_edgeUnD[i].idEnd;
					}
				}
			}else{
				for (int i=0;i<g_m;++i){
					if (g_edgeUnD[i].cpi>max){//this is a modified cpi!~
						max=g_edgeUnD[i].cpi;
						p=g_edgeUnD[i].idBegin;q=g_edgeUnD[i].idEnd;
					}
				}
				for (int i=g_m;i<g_edgeCoreUnDCount;++i){
					if (g_edgeUnD[i].cpi>=max){//this is a modified cpi!~
						if (g_edgeUnD[i].cpi==max&&g_edge[i+g_m].x==0){//will add flow to x so let it from the smallest k
						}else{//if ==&&x==u will return flow of x so let it from the largest k
							max=g_edgeUnD[i].cpi;
							p=g_edgeUnD[i].idBegin;q=g_edgeUnD[i].idEnd;
						}
					}
				}
				for (int i=g_edgeCoreUnDCount;i<g_edgeUnDCount;++i){
					if (g_edgeUnD[i].cpi>max){//this is a modified cpi!~
						max=g_edgeUnD[i].cpi;
						p=g_edgeUnD[i].idBegin;q=g_edgeUnD[i].idEnd;
					}
				}
			}
			//g_count++;
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
				int i=g[p][q];
				if (i<g_m+g_m)
					changeEdgeUnD(i>>1,g_edge);//m_rootvCons[i]&&bssm_root &&bssik(with u=INT_MAX) cannot enter here!
				else
					changeEdgeDUnD(i,g_edge);
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
				/*if (1||p==160&&q==99){
					int pCheck=m_root,num=1;
					while (v[pCheck].next!=m_root){
						if (pCheck!=m_root&&v[pCheck].depth!=v[v[pCheck].parent].depth+1)
							while (true){
								cout<<"ErrorDepth!";
							}
						int nTmp=pCheck;
						while (v[v[pCheck].next].prec!=pCheck)
							cout<<"ErrorPrec!"<<pCheck<<" "<<v[pCheck].next<<" "<<v[pCheck].prec<<" "<<v[v[pCheck].next].prec;
						pCheck=v[pCheck].next;num++;
						if (v[pCheck].parent==nTmp||v[pCheck].parent!=nTmp&&v[pCheck].depth<=v[nTmp].depth){
						}else{
							while (true){
								cout<<"ErrorParent!";
							}
						}
					}
					if (num!=g_numVert+m_numSS+2){
						cout<<"ErrorNum!";
					}
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
				if (pSearch<g_numVert){//pSearch cannot be m_root
					int kmUnD=v[pSearch].idEdge.size();
					for (int k=0;k<kmUnD;++k){
						changeEdgeUnD(v[pSearch].idEdge[k]>>1,g_edge);//if i<g_m*2, it's the same as using idEdgeUnD if both are from 0
					}
					for (int k=kmUnD;k<v[pSearch].idEdgeFrom.size();++k){//it has the same undirectional edges' number as the idEdge
						changeEdgeDUnD(v[pSearch].idEdgeFrom[k],g_edge);
					}
				}else if (pSearch==m_root-1){
					for (int k=0;k<(int)v[pSearch].idEdge.size();++k){
						changeEdgeDUnD(v[pSearch].idEdge[k],g_edge);
					}
				}else{//<g_numVert+m_numSS&&>=g_numVert
					for (int k=0;k<(int)v[pSearch].idEdge.size();++k){
						changeEdgeDUnD(v[pSearch].idEdge[k],g_edge);
					}
					changeEdgeDUnD(g_edgeCoreCount+pSearch-g_numVert,g_edge);
				}
				/*int kmUnD;
				for (kmUnD=v[pSearch].idEdge.size()-1;kmUnD>=0;--kmUnD){//pSearch cannot be m_root
					int i=v[pSearch].idEdge[kmUnD];
					if (i>>1<g_m)
						break;
					changeEdgeDUnD(i,g_edge);
				}
				for (int k=kmUnD;k>=0;--k){
					changeEdgeUnD(v[pSearch].idEdge[k]>>1,g_edge);//if i<g_m*2, it's the same as using idEdgeUnD if both are from 0
				}
				for (int k=kmUnD+1;k<v[pSearch].idEdgeFrom.size();++k){//it has the same undirectional edges' number as the idEdge
					changeEdgeDUnD(v[pSearch].idEdgeFrom[k],g_edge);
				}*/
				while (v[pSearch].next!=nTmp){//traverse the T2;nTmp is released here for label
					pSearch=v[pSearch].next;
					v[pSearch].depth=v[v[pSearch].parent].depth+1;
					if (nPQH==p&&bXpqL||nPQH==q&&!bXpqL){
						v[pSearch].pi+=max;//-=-max because cπnPQHnPQL=cπpq for L=-cπpq for U<0 && max always>0
					}else{
						v[pSearch].pi-=max;//vice versa
					}
					if (pSearch<g_numVert){//pSearch cannot be m_root
						int kmUnD=v[pSearch].idEdge.size();
						for (int k=0;k<kmUnD;++k){
							changeEdgeUnD(v[pSearch].idEdge[k]>>1,g_edge);//if i<g_m*2, it's the same as using idEdgeUnD if both are from 0
						}
						for (int k=kmUnD;k<v[pSearch].idEdgeFrom.size();++k){//it has the same undirectional edges' number as the idEdge
							changeEdgeDUnD(v[pSearch].idEdgeFrom[k],g_edge);
						}
					}else if (pSearch==m_root-1){
						for (int k=0;k<(int)v[pSearch].idEdge.size();++k){
							changeEdgeDUnD(v[pSearch].idEdge[k],g_edge);
						}
					}else{//<g_numVert+m_numSS&&>=g_numVert
						for (int k=0;k<(int)v[pSearch].idEdge.size();++k){
							changeEdgeDUnD(v[pSearch].idEdge[k],g_edge);
						}
						changeEdgeDUnD(g_edgeCoreCount+pSearch-g_numVert,g_edge);
					}
				}
				//g_count+=clock()-nTmTmp;
			}
		}
		//g_count+=clock()-nTmTmp;
	}while (max!=0);
	if (m_fast_mode==1){
		for (int i=g_edgeCount-g_numDem-1;i<g_edgeCount;++i)
			if (g_edge[i].x!=0){
				cout<<"NA"<<endl;
				return ~0u>>1;
			}
	}
	return 0;
}
void ServerSelectionSolver::changeEdgeDUnD(int i,NodeEdge g_edge[]){//if i>=g_m*2, input i is the id of g_edge!
	//if (g_edgeUnD[i-g_m].idBegin!=g_edge[i].idBegin||g_edgeUnD[i-g_m].idEnd!=g_edge[i].idEnd) cout<<"Error"<<endl;
	if (g_edge[i].x>0){//cpi maybe 0(x<u) or +cπij(x==u)
		g_edgeUnD[i-g_m].cpi=g_edge[i].c-v[g_edge[i].idBegin].pi+v[g_edge[i].idEnd].pi;
	}else{//x=0 so cpi=-cπij
		g_edgeUnD[i-g_m].cpi=-(g_edge[i].c-v[g_edge[i].idBegin].pi+v[g_edge[i].idEnd].pi);//idBegin=ki;idEnd=kj;
	}
	if (g_bUseHeap) zkwtreeModify(i-g_m);
}
void ServerSelectionSolver::changeEdgeUnD(int i,NodeEdge g_edge[]){//input i is the id of g_edgeUnD! && i<g_m
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
void ServerSelectionSolver::zkwtreeModify(int i){
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
void ServerSelectionSolver::findOutgoing(int& k,int& l,bool& outPTop,bool bResult,
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
