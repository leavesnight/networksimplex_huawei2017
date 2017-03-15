#include "deploy.h"
#include "lib_io.h"
#include "lib_time.h"
#include "stdio.h"
//#include <fstream>

//const int MAX_STRING_SIZE=20;

int main(int argc, char *argv[])
{
    print_time("Begin");
    char *topo[MAX_EDGE_NUM];
    int line_num=0;

    char *topo_file = argv[1];

	/*std::ifstream fin(topo_file);
	while (line_num<MAX_EDGE_NUM){
		topo[line_num]=new char[MAX_STRING_SIZE];
		fin.getline(topo[line_num++],MAX_STRING_SIZE);
	}*/
	line_num = read_file(topo, MAX_EDGE_NUM, topo_file);

    printf("line num is :%d \n", line_num);
    if (line_num == 0)
    {
        printf("Please input valid topo file.\n");
        return -1;
    }

    char *result_file = argv[2];

    deploy_server(topo, line_num, result_file);

	/*for (int i=0;i<line_num;i++){
		delete []topo[i];
	}*/
	release_buff(topo, line_num);

    print_time("End");
	
	//fin.close();

	return 0;
}

