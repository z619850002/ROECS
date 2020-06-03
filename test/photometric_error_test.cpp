// #include <iostream>
// #include <map>
// #include <string>
// #include <vector>
// #include <sstream>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/features2d/features2d.hpp>
// #include <opencv2/calib3d/calib3d.hpp>


// using namespace std;

// void deleteAllMark(string &s, const string &mark)
// {
// 	size_t nSize = mark.size();
// 	while(1)
// 	{
// 		size_t pos = s.find(mark);
// 		if(pos == string::npos)
// 		{
// 			return;
// 		}
 
// 		s.erase(pos, nSize);
// 	}
// }

// void split(const string& s,vector<string>& sv,const char flag = ' ') {
//     sv.clear();
//     istringstream iss(s);
//     string temp;

//     while (getline(iss, temp, flag)) {
//         sv.push_back(temp);
//     }
//     return;
// }

// int countChar(const string& s , const char flag){
// 	int count = 0;
// 	for (int i=0;i<s.size();i++){
// 		if (s[i]==flag){
// 			count ++;
// 		}
// 	}
// 	return count;
// }



// vector<double> ConvertStringToNumbers(string aString){
// 	//Delete all repeated spaces.
// 	while(1)
// 	{
// 		size_t pos = aString.find("  ");
// 		if(pos == string::npos)
// 		{
// 			break;
// 		}
 
// 		s.erase(pos, 1);
// 	}

// 	vector<string> gVec;
// 	vector<string> gResultVec;
// 	split(aString, gVec);
// 	for (auto item : gVec){
// 		if (item.size()==0){
// 			continue;
// 		}
// 		gResultVec.push_back(gVec);
// 	}
// 	if (gResultVec.size()!= 4){
// 		cerr << "Error result size when converting string to 4 numbers" << endl;
// 	}

// 	vector<double> gResult;
// 	for (auto item : gResultVec){
// 		gResult.push_back(atof(item));
// 	}
// 	return gResult;
// }


// vector<vector<Sophus::SE3>> ConvertFileToPoses(vector<string> aFile){
// 	int nIteration = aFile.size()/

// }


// int main(){
// 	return 0;
// }