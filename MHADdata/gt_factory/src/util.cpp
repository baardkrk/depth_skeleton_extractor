#include <cctype>
#include <algorithm>
#include <locale>
#include <iostream>


std::string ltrim(std::string s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
	return !std::isspace(ch);
      }));
  return s;
}

std::string rtrim(std::string s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
	return !std::isspace(ch);
      }).base(), s.end());
  return s;
}

std::string trim(std::string s)
{
  s = ltrim(s);
  s = rtrim(s);
  return s;
}



// int main(int argc, char *argv[])
// {
//   std::string s1 = " this is a test ";
  
//   std::cout << "Left-trim:\n" << ltrim(s1) << "nospace" << std::endl;
//   std::cout << "Right-trim:\n" << rtrim(s1) << "nospace" << std::endl;
//   std::cout << "Trim both:\n" << trim(s1) << "nospace" << std::endl;
//   return 0;
// }
