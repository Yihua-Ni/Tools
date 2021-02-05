#include "tools.h"
#include <iostream>


void simpleSplit(std::string str, std::vector<std::string>& strVec, char s)
{
	strVec.clear();
	while(1)
	{
		int spacePosition = str.find(s);
		int len = str.length();
		if(spacePosition >= 0)
		{
			std::string strFir = str.substr(0, spacePosition);
			std::string strSub = str.substr(spacePosition+1, len-1);
			str = strSub;
			if(strFir.size()>0){
				strVec.push_back(strFir);
			}
		}
		else
		{
			std::string strFir = str.substr(0, len);
			strVec.push_back(strFir);
			break;
		}
	}
}