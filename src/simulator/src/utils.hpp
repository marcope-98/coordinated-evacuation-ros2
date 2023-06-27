#ifndef SIMULATOR_UTILS_HPP_
#define SIMULATOR_UTILS_HPP_

#include <cstdint>
#include <string>
#include <vector>

struct utils
{
  static bool contains(const std::string &str, const std::string &pattern)
  {
    return str.find(pattern) != std::string::npos;
  }

  static std::size_t find_character(const std::string &str,
                                    const std::string &character)
  {
    return str.find(character);
  }

  static std::vector<std::string> split_by_delimiter(const std::string &str,
                                                     const std::string &delimiter)
  {
    std::string              temp = str;
    std::vector<std::string> out;
    std::size_t              offset;

    for (;;)
    {
      if (!contains(temp, delimiter)) break;
      offset = find_character(temp, delimiter);
      out.emplace_back(temp.substr(0, offset));
      temp = temp.substr(offset + delimiter.size());
    }
    out.emplace_back(temp);
    return out;
  }
};

#endif // SIMULATOR_UTILS_HPP_