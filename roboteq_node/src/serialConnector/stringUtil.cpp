#include "stringUtil.h"

TStrParam    StringUtil::split(const std::string& str, char ch)
{
    TStrParam vec;

    if( str.empty() )
        return vec;

    std::string::size_type startIdx(0);

    while(true)
    {
        std::string::size_type startIdy = str.find_first_of(ch, startIdx);

        if( startIdy == std::string::npos )
        {
	    if( str.substr( startIdx ).length() )
            	vec.push_back( str.substr( startIdx ) );
            break;
        }
        else
        {
            vec.push_back( str.substr( startIdx, startIdy -  startIdx) );
            startIdx = startIdy+1;
        }
    }

    return vec;
}

std::string     StringUtil::trimRight(const std::string& str, char ch)
{
    std::string::size_type Idx = str.find_last_not_of(ch);

    if( Idx != std::string::npos )
        return str.substr(0, Idx+1);
    else if( str.length() > 0 && str[0] == ch)
        return "";
    else return str;
}

std::string     StringUtil::trimLeft(const std::string& str, char ch)
{
    std::string::size_type Idx = str.find_first_not_of(ch);

    if( Idx != std::string::npos )
        return str.substr(Idx);
    else if( str.length() > 0 && str[str.length()-1] == ch)
        return "";
    else return str;
}

std::string     StringUtil::trimBoth(const std::string& str, char ch)
{
    return trimRight( trimLeft( str, ch), ch);
}



