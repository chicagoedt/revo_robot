#ifndef __STRING_UTIL_H__
#define __STRING_UTIL_H__

#include <string>
#include <vector>

typedef std::vector<std::string> TStrParam;

class StringUtil
{
    public:
        static TStrParam    split(const std::string& str, char ch = ' ');
        static std::string  trimRight(const std::string& str, char ch = ' ');
        static std::string  trimLeft(const std::string& str, char ch = ' ');
        static std::string  trimBoth(const std::string& str, char ch = ' ');
};

#endif // __STRING_UTIL_H__

