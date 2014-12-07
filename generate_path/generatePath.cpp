#include "generatePath.h"
#include <stdlib.h>
#include <algorithm>


void generatePath::astarPath(){

	int w,h,wdiv,hdiv;//グリッドマップの大きさと分割する大きさ
	float heuristic;

	int openListNo   = -1;
	int nodeNo       = 0;
	float minCost    = 0;// 最小コスト
	int mCost;// ノードのコスト
	int mx,my;// 移動するノードの座標

	float fm;//全体のコスト
	float ga=0;//基準となるノードまでのコスト
	float costNM;//基準のノードから隣接ノードまでのコスト
	int oflag,cflag;

	Node s,g,m,n;//スタートとゴール 基準となるノード

	//A*に必要なリスト
	std::vector<Node> openList;
	std::vector<Node> closeList;

	openList = std::vector<Node>();
	closeList = std::vector<Node>();

	//グリッドマップの大きさ
	w = 1500;// x方向
	h = 1500;// y方向

	wdiv = 100;// x方向をどのくらいの間隔で区切るか
	hdiv = 100;// y方向をどのくらいの間隔で区切るか

	minCost = w*h;// 比較するコストの初期化

	s.Set_Pos(0.0, 0.0);// スタートの位置
	s.Set_OurNum(0);    // ノード番号
	s.Set_PreNum(-1);   // 親ノードの番号

	g.Set_Pos(1000.0, 1000.0);// ゴールの位置
	g.Set_OurNum(g.Our_Pos().x() + g.Our_Pos().y()*10);//　ノード番号の初期化

	heuristic = sqrt(pow(g.Our_Pos().x()-s.Our_Pos().x(),2) + pow(g.Our_Pos().y()-s.Our_Pos().y(),2));// ヒューリスティック　ゴールまでの距離
	openList.push_back(s);// openListにスタートノードを入れる

	while(1){

		if(openList.size()==0) break;//ゴールまでの経路が存在しない
		//今は隣接している8つのノードに移動するコストはすべて同じとするので移動コストはヒューリスティックのみ
		for(int i=0; i<openList.size(); i++){
			
			int opy=openList[i].Our_Pos().y();
			int opx=openList[i].Our_Pos().x();

			//コストの比較
			heuristic = sqrt(pow(g.Our_Pos().x()-openList[i].Our_Pos().x(),2) + pow(g.Our_Pos().y()-openList[i].Our_Pos().y(),2));
			if(heuristic < minCost){
				
				openListNo = i;
				minCost   = heuristic;
				m         = openList[i];
			}

		}

		if((m.Our_Pos().x()==g.Our_Pos().x()) && (m.Our_Pos().y()==g.Our_Pos().y())){//探索終了
			closeList.push_back(m);
			nodeNo = closeList.size()-1;
			break;
		}
		else{

			closeList.push_back(m);
			std::vector<Node>::iterator open;
			open = openList.erase(openList.begin() + openListNo);
		}

		//隣接ノードについて
		for(int y=-hdiv; y<(hdiv+1); y+=hdiv){// 分割数
			for(int x=-wdiv; x<(wdiv+1); x+=wdiv){// 分割数

				mx = m.Our_Pos().x() + x;
				my = m.Our_Pos().y() + y;
				
				// xyの範囲の指定，今は指定した範囲の±の値 
				// ここで障害物のデータとの照合が必要
				if(mx<-w);
				else if(my<-h);
				else if(mx>w);
				else if(my>h);
				else if(y==0 && x==0);// 求めたノードが今と同じ位置

			
				// 以下A*の処理
				else{
					fm = sqrt(pow(g.Our_Pos().x()-mx,2) + pow(g.Our_Pos().y()-my,2));
					// openListとcloseListのどっちに入っているか検索
					oflag=cflag=0;
					for(int i=0; i<openList.size(); i++){
						if((mx==openList[i].Our_Pos().x()) && (my==openList[i].Our_Pos().y())){
							oflag=1;
							openListNo=i;
							break;
						}
					}
					for(int i=0; i<closeList.size(); i++){
						if((mx==closeList[i].Our_Pos().x()) && (my==closeList[i].Our_Pos().y())){
							cflag=1;
							openListNo=i;
							break;
						}
					}
					// それぞれの場合の操作
					// openListにある場合
					if(oflag){
						if(fm < heuristic){
							openList[openListNo].Set_PreNum(m.Our_Num());
						}
					}
					//closeListにある場合
					else if(cflag){
						if(fm  < heuristic){

							n = closeList[openListNo];
							n.Set_PreNum(m.Our_Num());
							openList.push_back(n);
							std::vector<Node>::iterator close;
							close = closeList.erase(closeList.begin() + openListNo);
						}
					}
					//どっちにも無い場合
					else{

						nodeNo++;
						//std::cout << "mx=" << mx << " my=" << my << std::endl;
						n.Set_Pos(mx,my);
						n.Set_OurNum(nodeNo);
						n.Set_PreNum(m.Our_Num());
						openList.push_back(n);
					}
				}
			}
		}
	}

	pathData.push_back(closeList[nodeNo]);
	std::vector<Node>::iterator pathIte = pathData.begin();
	for(int i=0; i<closeList.size(); i++){
		
		//pathData.push_back(closeList[nodeNo]);
		for(int n=0; n<closeList.size(); n++){
			if(closeList[nodeNo].Pre_Num()==closeList[n].Our_Num()) nodeNo=n;// 親ノードをたどる
			if(closeList[nodeNo].Pre_Num() == -1) i=closeList.size();// スタート位置の場合抜ける
		}
		pathData.insert(pathIte, 1, closeList[nodeNo]);
		pathIte = pathData.begin();
	}

	std::ofstream ofs;
	ofs.open("navi.csv",std::ios::trunc);// 経路の出力先
	for(int i=0; i<pathData.size(); i++){
		
		//std::cout << pathData[i].Our_Pos().x() << "," << pathData[i].Our_Pos().y() << std::endl;
		ofs << pathData[i].Our_Pos().x() << "," << pathData[i].Our_Pos().y() << std::endl;
	}


}




// マージンを追加した経路の作成
// 出力される値は進行方向に対して
// 左のxy，右にxyの順で計４つの値が
// 一行ごとに出力される
void generatePath::createMarjin(){

	std::ifstream ifs;
	ifs.open("navi.csv");//元の経路

	int x[2]={0},y[2]={0},deg[2]={0};
	int count,num,flag,marjin;
	int t;
	float theta;
	char buf[4];

	std::string line;
	std::string hoge;

	count  = 0;
	num    = 0;
	flag   = 0;
	marjin = 10;// マージンの値 要変更
	t      = 0;

	std::ofstream ofs;
	ofs.open("geneMarjin.csv",std::ios::trunc);//マージンの出力先

	if(!ifs) std::cout << "no file" << std::endl;
	else{

		//元の経路の読み込み
		while(std::getline(ifs,line)){

			std::istringstream stream(line);
			while(std::getline(stream, hoge, ',')){
				if(count==0){
					x[num] = atoi(hoge.c_str());
				}
				if(count==1){
					y[num] = atoi(hoge.c_str());
				}
				count++;
			}
			num++;
			count=0;

			if(num>1){

				theta  = maxPi(atan2((double)(y[1]-y[0]), (double)(x[1]-x[0])));
				deg[0] = (int)(theta*180.0/M_PI)-360;//一周分足されてる値が出てるので -360

				//y方向へ平行移動
				if((x[1]-x[0]) == 0){
					if((y[1]-y[0]) == 0);
					else{
						if(y[1]-y[0]>0){
						
							ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
								<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;

						}
						if(y[1]-y[0]<0){
					
							ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
								<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
						}
						else;
					}
				}
				else{
					//x方向へ平行移動
					if((y[1]-y[0]) == 0){
						if(x[1]-x[0]>0){
						
							ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
								<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;

						}
						if(x[1]-x[0]<0){
					
							ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
								<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
						}
						else;
					}

					else{
						//第一象限
						if(x[0]>=0 && y[0]>=0){
							if((x[1]-x[0])>0){
								if(deg[0]>=0 && deg[0]<=90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]>90 && deg[0]<=180){
									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<0 && deg[0]>=-90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<-90 && deg[0]>=-180){

									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
							}
							else{

								if(deg[0]>=0 && deg[0]<=90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]>90 && deg[0]<=180){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<0 && deg[0]>=-90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<-90 && deg[0]>=-180){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
							}
						}
					
						//第二象限
						if(x[0]<0 && y[0]>0){
							if((x[1]-x[0])>0){
								if(deg[0]>=0 && deg[0]<=90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]>90 && deg[0]<=180){
									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<0 && deg[0]>=-90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<-90 && deg[0]>=-180){

									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
							}
							else{
								if(deg[0]>=0 && deg[0]<=90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]>90 && deg[0]<=180){
									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<0 && deg[0]>=-90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<-90 && deg[0]>=-180){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
							}
						}
				
						//第四象限
						if(x[0]>0 && y[0]<0){
							if((x[1]-x[0])>0){
								if(deg[0]>=0 && deg[0]<=90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]>90 && deg[0]<=180){

									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<0 && deg[0]>=-90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
							
								if(deg[0]<-90 && deg[0]>=-180){

									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
							}
							else{
								if(deg[0]>=0 && deg[0]<=90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]>90 && deg[0]<=180){

									ofs << x[0]-marjin*cos(theta-M_PI_2) << "," << y[0]-marjin*sin(theta-M_PI_2) << "," 
										<< x[0]+marjin*cos(theta-M_PI_2) << "," << y[0]+marjin*sin(theta-M_PI_2) << std::endl;
								}
								if(deg[0]<0 && deg[0]>=-90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
							
								if(deg[0]<-90 && deg[0]>=-180){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
							}
						}
				

						//第三象限
						if(x[0]<0 && y[0]<0){
							if((x[1]-x[0])>0){
								if(deg[0]>=0 && deg[0]<=90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]>90 && deg[0]<=180){

									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<0 && deg[0]>=-90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<-90 && deg[0]>=-180){

									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
							}
							else{
								if(deg[0]>=0 && deg[0]<=90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]>90 && deg[0]<=180){

									ofs << x[0]-marjin*cos(theta-M_PI_2) << "," << y[0]-marjin*sin(theta-M_PI_2) << "," 
										<< x[0]+marjin*cos(theta-M_PI_2) << "," << y[0]+marjin*sin(theta-M_PI_2) << std::endl;
								}
								if(deg[0]<0 && deg[0]>=-90){

									ofs << x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
								if(deg[0]<-90 && deg[0]>=-180){

									ofs << x[0]-marjin*cos(theta+M_PI_2) << "," << y[0]-marjin*sin(theta+M_PI_2) << "," 
										<< x[0]+marjin*cos(theta+M_PI_2) << "," << y[0]+marjin*sin(theta+M_PI_2) << std::endl;
								}
							}
						}


					}
				}
			
				num=1;
				if(t==0){
					//スタート地点の設定
					xs = x[0];
					ys = y[0];
					t++;
				}

				x[0] = x[1];
				y[0] = y[1];
				deg[0] = deg[1];
			}

		}//while
	}//else
}

// 評価関数
//とりあえず重心がマージンの領域内に入っているかどうか
//重心がある時間t秒目の１つ前と１つ先のwaypointの範囲までに入っているかを見る
bool generatePath::func1(std::vector<float> xl, std::vector<float> yl, std::vector<float> xr, std::vector<float> yr, Node our, int t){

	float la,lb,ra,rb;
	int deg,count,i;

	t=t-1;
	count=0;
	i=1;
	deg = (int)our.Theta()*180/M_PI;

	while(2>count){
		if(t<0);
	//y方向に平行移動
		else if((xl[t+i]-xl[t]) == 0){
			//x方向の判定が必要
			if(deg == 90){
				if(xl[t]<our.Our_Pos().x() && xr[t]>our.Our_Pos().x()) return 1;
			}
			if(deg == -90){
				if(xl[t]>our.Our_Pos().x() && xr[t]<our.Our_Pos().x()) return 1;
			}
		}
		else{
	
			la = (yl[t+i]-yl[t])/(xl[t+i]-xl[t]);
			lb = yl[t] - la*xl[t];

			ra = (yr[t+i]-yr[t])/(xr[t+i]-xr[t]);
			rb = yr[t] - ra*xr[t];

			if(deg>=0 && deg<90){
				//y方向が範囲に入っているか
				if((our.Our_Pos().x()*la+lb)>our.Our_Pos().y() && (our.Our_Pos().x()*ra+rb)<our.Our_Pos().y()) return 1;
			}
			if(deg<0 && deg>-90){
				if((our.Our_Pos().x()*la+lb)>our.Our_Pos().y() && (our.Our_Pos().x()*ra+rb)<our.Our_Pos().y()) return 1;
			}
			if(deg>90 && deg<=180){
				if((our.Our_Pos().x()*la+lb)<our.Our_Pos().y() && (our.Our_Pos().x()*ra+rb)>our.Our_Pos().y()) return 1;
			}
			if(deg<-90 && deg>-180){
				if((our.Our_Pos().x()*la+lb)<our.Our_Pos().y() && (our.Our_Pos().x()*ra+rb)>our.Our_Pos().y()) return 1;
			}
		}
		count++;
		t++;
	}
	return 0;
}



//経路探索
void generatePath::pathPlan(){

	std::ifstream ifs;
	ifs.open("geneMarjin.csv");// マージンのファイルの読み込み

	std::ofstream ofs;
	ofs.open("path.csv",std::ios::trunc);// 求めた経路の出力先

	std::vector<float> xl,yl,xr,yr;
	float a[2],b[2];
	float vr,vl,accel;
	int count,num,flag,t;
	unsigned int nodeNum,fastNum;

	std::string line,hoge;

	count=0;
	num=0;
	t=1;
	flag=1;
	accel=100;// 加速度 あとで変更　(mm/s^2)
	nodeNum=0;

	//スタート地点 角度 番号の設定
	node.Set_Pos(xs, ys);
	node.Set_theta(thetas);
	node.Set_OurNum(0);
	node.Set_PreNum(0);

	pathData.push_back(node);
	preNode.push_back(node);


	if(!ifs) std::cout << "no file" << std::endl;
	else{
		//waypointごとのマージンの範囲の読み込み
		while(std::getline(ifs,line)){

			std::istringstream stream(line);
			while(std::getline(stream, hoge, ',')){
				if(count==0) xl.push_back((float)atof(hoge.c_str()));
				if(count==1) yl.push_back((float)atof(hoge.c_str()));
				if(count==2) xr.push_back((float)atof(hoge.c_str()));
				if(count==3) yr.push_back((float)atof(hoge.c_str()));
				count++;
			}
			count=0;
		}
		
		//経路探索
		while(flag){

			t++;
			if(t>5) break;//デバッグのために今は適当な回数回している
			for(unsigned int i=0; i<preNode.size(); i++){
				for(float r=-1.0; r<2.0; r++){
					for(float l=-1.0; l<2.0; l++){

						vr = r*accel;
						vl = l*accel;

						// 考える重複するノードについて
						if(preNode[i].Vr()==500.0 && r==1);
						else if(preNode[i].Vl()==500.0 && l==1);
						else if((preNode[i].Our_Pos().x()==0 && preNode[i].Our_Pos().x()==0) && (r==0 && l==0));
						else if((preNode[i].Our_Pos().x()==0 && preNode[i].Our_Pos().x()==0) && (r==0 && l==0));
								
						else{
							
							tcar.equation_of_motion(vr,vl,1,preNode[i]);
							//マージンの領域内に入っているかどうか
							if(func1(xl,yl,xr,yr,tcar.Our_Node(),t)){

								nodeNum++;
								tcar.Set_OurNum(nodeNum);
								tcar.Set_PreNum(preNode[i].Our_Num());
								pathData.push_back(tcar.Our_Node());
								preNode2.push_back(tcar.Our_Node());

								//今は最短経路ではなくマージンの領域内に入っているノードすべてを出力している 
								ofs << tcar.Our_Node().Our_Pos().x() << "," << tcar.Our_Node().Our_Pos().y() << std::endl;
							}
						}
					}
				}
			}

			//std::cout << "nodeNum " << nodeNum << std::endl;
			//std::cout << "time = " << t << std::endl;
			std::cout << std::endl;
			preNode.clear();
			for(unsigned int i=0; i<preNode2.size(); i++){
			
				preNode.push_back(preNode2[i]);
			}
			preNode2.clear();
		}
	}
}









