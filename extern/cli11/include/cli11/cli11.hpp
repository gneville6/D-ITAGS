/*
 * Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020-2021
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

// Standard combined includes:
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <type_traits>

#define CLI11_VERSION_MAJOR 2
#define CLI11_VERSION_MINOR 0
#define CLI11_VERSION_PATCH 0
#define CLI11_VERSION       "2.0.0"

// The following version macro is very similar to the one in pybind11
#if !(defined(_MSC_VER) && __cplusplus == 199711L) && !defined(__INTEL_COMPILER)
#    if __cplusplus >= 201402L
#        define CLI11_CPP14
#        if __cplusplus >= 201703L
#            define CLI11_CPP17
#            if __cplusplus > 201703L
#                define CLI11_CPP20
#            endif
#        endif
#    endif
#elif defined(_MSC_VER) && __cplusplus == 199711L
// MSVC sets _MSVC_LANG rather than __cplusplus (supposedly until the standard is fully implemented)
// Unless you use the /Zc:__cplusplus flag on Visual Studio 2017 15.7 Preview 3 or newer
#    if _MSVC_LANG >= 201402L
#        define CLI11_CPP14
#        if _MSVC_LANG > 201402L && _MSC_VER >= 1910
#            define CLI11_CPP17
#            if __MSVC_LANG > 201703L && _MSC_VER >= 1910
#                define CLI11_CPP20
#            endif
#        endif
#    endif
#endif

#if defined(CLI11_CPP14)
#    define CLI11_DEPRECATED(reason) [[deprecated(reason)]]
#elif defined(_MSC_VER)
#    define CLI11_DEPRECATED(reason) __declspec(deprecated(reason))
#else
#    define CLI11_DEPRECATED(reason) __attribute__((deprecated(reason)))
#endif

// C standard library
// Only needed for existence checking
#if defined CLI11_CPP17 && defined __has_include && !defined CLI11_HAS_FILESYSTEM
#    if __has_include(<filesystem>)
// Filesystem cannot be used if targeting macOS < 10.15
#        if defined __MAC_OS_X_VERSION_MIN_REQUIRED && __MAC_OS_X_VERSION_MIN_REQUIRED < 101500
#            define CLI11_HAS_FILESYSTEM 0
#        else
#            include <filesystem>
#            if defined __cpp_lib_filesystem && __cpp_lib_filesystem >= 201703
#                if defined _GLIBCXX_RELEASE && _GLIBCXX_RELEASE >= 9
#                    define CLI11_HAS_FILESYSTEM 1
#                elif defined(__GLIBCXX__)
// if we are using gcc and Version <9 default to no filesystem
#                    define CLI11_HAS_FILESYSTEM 0
#                else
#                    define CLI11_HAS_FILESYSTEM 1
#                endif
#            else
#                define CLI11_HAS_FILESYSTEM 0
#            endif
#        endif
#    endif
#endif

#if defined CLI11_HAS_FILESYSTEM && CLI11_HAS_FILESYSTEM > 0
#    include <filesystem>  // NOLINT(build/include)
#else
#    include <sys/stat.h>
#    include <sys/types.h>
#endif

namespace CLI
{

    /// Include the items in this namespace to get free conversion of enums to/from streams.
    /// (This is available inside CLI as well, so CLI11 will use this without a using statement).
    namespace enums
    {

        /// output streaming for enumerations
        template <typename T, typename = typename std::enable_if<std::is_enum<T>::value>::type>
        std::ostream &operator<<(std::ostream &in, const T &item)
        {
            // make sure this is out of the detail namespace otherwise it won't be found when needed
            return in << static_cast<typename std::underlying_type<T>::type>(item);
        }

    }  // namespace enums

    /// Export to CLI namespace
    using enums::operator<<;

    namespace detail
    {
        /// a constant defining an expected max vector size defined to be a big number that could be multiplied by 4 and
        /// not produce overflow for some expected uses
        constexpr int expected_max_vector_size{1 << 29};
        // Based on http://stackoverflow.com/questions/236129/split-a-string-in-c
        /// Split a string by a delim
        inline std::vector<std::string> split(const std::string &s, char delim)
        {
            std::vector<std::string> elems;
            // Check to see if empty string, give consistent result
            if(s.empty())
            {
                elems.emplace_back();
            }
            else
            {
                std::stringstream ss;
                ss.str(s);
                std::string item;
                while(std::getline(ss, item, delim))
                {
                    elems.push_back(item);
                }
            }
            return elems;
        }

        /// Simple function to join a string
        template <typename T>
        std::string join(const T &v, std::string delim = ",")
        {
            std::ostringstream s;
            auto beg = std::begin(v);
            auto end = std::end(v);
            if(beg != end)
                s << *beg++;
            while(beg != end)
            {
                s << delim << *beg++;
            }
            return s.str();
        }

        /// Simple function to join a string from processed elements
        template <typename T,
                  typename Callable,
                  typename = typename std::enable_if<!std::is_constructible<std::string, Callable>::value>::type>
        std::string join(const T &v, Callable func, std::string delim = ",")
        {
            std::ostringstream s;
            auto beg = std::begin(v);
            auto end = std::end(v);
            auto loc = s.tellp();
            while(beg != end)
            {
                auto nloc = s.tellp();
                if(nloc > loc)
                {
                    s << delim;
                    loc = nloc;
                }
                s << func(*beg++);
            }
            return s.str();
        }

        /// Join a string in reverse order
        template <typename T>
        std::string rjoin(const T &v, std::string delim = ",")
        {
            std::ostringstream s;
            for(std::size_t start = 0; start < v.size(); start++)
            {
                if(start > 0)
                    s << delim;
                s << v[v.size() - start - 1];
            }
            return s.str();
        }

        // Based roughly on http://stackoverflow.com/questions/25829143/c-trim-whitespace-from-a-string

        /// Trim whitespace from left of string
        inline std::string &ltrim(std::string &str)
        {
            auto it = std::find_if(str.begin(),
                                   str.end(),
                                   [](char ch)
                                   {
                                       return !std::isspace<char>(ch, std::locale());
                                   });
            str.erase(str.begin(), it);
            return str;
        }

        /// Trim anything from left of string
        inline std::string &ltrim(std::string &str, const std::string &filter)
        {
            auto it = std::find_if(str.begin(),
                                   str.end(),
                                   [&filter](char ch)
                                   {
                                       return filter.find(ch) == std::string::npos;
                                   });
            str.erase(str.begin(), it);
            return str;
        }

        /// Trim whitespace from right of string
        inline std::string &rtrim(std::string &str)
        {
            auto it = std::find_if(str.rbegin(),
                                   str.rend(),
                                   [](char ch)
                                   {
                                       return !std::isspace<char>(ch, std::locale());
                                   });
            str.erase(it.base(), str.end());
            return str;
        }

        /// Trim anything from right of string
        inline std::string &rtrim(std::string &str, const std::string &filter)
        {
            auto it = std::find_if(str.rbegin(),
                                   str.rend(),
                                   [&filter](char ch)
                                   {
                                       return filter.find(ch) == std::string::npos;
                                   });
            str.erase(it.base(), str.end());
            return str;
        }

        /// Trim whitespace from string
        inline std::string &trim(std::string &str)
        {
            return ltrim(rtrim(str));
        }

        /// Trim anything from string
        inline std::string &trim(std::string &str, const std::string filter)
        {
            return ltrim(rtrim(str, filter), filter);
        }

        /// Make a copy of the string and then trim it
        inline std::string trim_copy(const std::string &str)
        {
            std::string s = str;
            return trim(s);
        }

        /// remove quotes at the front and back of a string either '"' or '\''
        inline std::string &remove_quotes(std::string &str)
        {
            if(str.length() > 1 && (str.front() == '"' || str.front() == '\''))
            {
                if(str.front() == str.back())
                {
                    str.pop_back();
                    str.erase(str.begin(), str.begin() + 1);
                }
            }
            return str;
        }

        /// Make a copy of the string and then trim it, any filter string can be used (any char in string is filtered)
        inline std::string trim_copy(const std::string &str, const std::string &filter)
        {
            std::string s = str;
            return trim(s, filter);
        }
        /// Print a two part "help" string
        inline std::ostream &format_help(std::ostream &out,
                                         std::string name,
                                         const std::string &description,
                                         std::size_t wid)
        {
            name = "  " + name;
            out << std::setw(static_cast<int>(wid)) << std::left << name;
            if(!description.empty())
            {
                if(name.length() >= wid)
                    out << "\n" << std::setw(static_cast<int>(wid)) << "";
                for(const char c: description)
                {
                    out.put(c);
                    if(c == '\n')
                    {
                        out << std::setw(static_cast<int>(wid)) << "";
                    }
                }
            }
            out << "\n";
            return out;
        }

        /// Print subcommand aliases
        inline std::ostream &format_aliases(std::ostream &out, const std::vector<std::string> &aliases, std::size_t wid)
        {
            if(!aliases.empty())
            {
                out << std::setw(static_cast<int>(wid)) << "     aliases: ";
                bool front = true;
                for(const auto &alias: aliases)
                {
                    if(!front)
                    {
                        out << ", ";
                    }
                    else
                    {
                        front = false;
                    }
                    out << alias;
                }
                out << "\n";
            }
            return out;
        }

        /// Verify the first character of an option
        template <typename T>
        bool valid_first_char(T c)
        {
            return std::isalnum(c, std::locale()) || c == '_' || c == '?' || c == '@';
        }

        /// Verify following characters of an option
        template <typename T>
        bool valid_later_char(T c)
        {
            return valid_first_char(c) || c == '.' || c == '-';
        }

        /// Verify an option name
        inline bool valid_name_string(const std::string &str)
        {
            if(str.empty() || !valid_first_char(str[0]))
                return false;
            for(auto c: str.substr(1))
                if(!valid_later_char(c))
                    return false;
            return true;
        }

        /// check if a string is a container segment separator (empty or "%%")
        inline bool is_separator(const std::string &str)
        {
            static const std::string sep("%%");
            return (str.empty() || str == sep);
        }

        /// Verify that str consists of letters only
        inline bool isalpha(const std::string &str)
        {
            return std::all_of(str.begin(),
                               str.end(),
                               [](char c)
                               {
                                   return std::isalpha(c, std::locale());
                               });
        }

        /// Return a lower case version of a string
        inline std::string to_lower(std::string str)
        {
            std::transform(std::begin(str),
                           std::end(str),
                           std::begin(str),
                           [](const std::string::value_type &x)
                           {
                               return std::tolower(x, std::locale());
                           });
            return str;
        }

        /// remove underscores from a string
        inline std::string remove_underscore(std::string str)
        {
            str.erase(std::remove(std::begin(str), std::end(str), '_'), std::end(str));
            return str;
        }

        /// Find and replace a substring with another substring
        inline std::string find_and_replace(std::string str, std::string from, std::string to)
        {
            std::size_t start_pos = 0;

            while((start_pos = str.find(from, start_pos)) != std::string::npos)
            {
                str.replace(start_pos, from.length(), to);
                start_pos += to.length();
            }

            return str;
        }

        /// check if the flag definitions has possible false flags
        inline bool has_default_flag_values(const std::string &flags)
        {
            return (flags.find_first_of("{!") != std::string::npos);
        }

        inline void remove_default_flag_values(std::string &flags)
        {
            auto loc = flags.find_first_of('{');
            while(loc != std::string::npos)
            {
                auto finish = flags.find_first_of("},", loc + 1);
                if((finish != std::string::npos) && (flags[finish] == '}'))
                {
                    flags.erase(flags.begin() + static_cast<std::ptrdiff_t>(loc),
                                flags.begin() + static_cast<std::ptrdiff_t>(finish) + 1);
                }
                loc = flags.find_first_of('{', loc + 1);
            }
            flags.erase(std::remove(flags.begin(), flags.end(), '!'), flags.end());
        }

        /// Check if a string is a member of a list of strings and optionally ignore case or ignore underscores
        inline std::ptrdiff_t find_member(std::string name,
                                          const std::vector<std::string> names,
                                          bool ignore_case       = false,
                                          bool ignore_underscore = false)
        {
            auto it = std::end(names);
            if(ignore_case)
            {
                if(ignore_underscore)
                {
                    name = detail::to_lower(detail::remove_underscore(name));
                    it   = std::find_if(std::begin(names),
                                      std::end(names),
                                      [&name](std::string local_name)
                                      {
                                          return detail::to_lower(detail::remove_underscore(local_name)) == name;
                                      });
                }
                else
                {
                    name = detail::to_lower(name);
                    it   = std::find_if(std::begin(names),
                                      std::end(names),
                                      [&name](std::string local_name)
                                      {
                                          return detail::to_lower(local_name) == name;
                                      });
                }
            }
            else if(ignore_underscore)
            {
                name = detail::remove_underscore(name);
                it   = std::find_if(std::begin(names),
                                  std::end(names),
                                  [&name](std::string local_name)
                                  {
                                      return detail::remove_underscore(local_name) == name;
                                  });
            }
            else
            {
                it = std::find(std::begin(names), std::end(names), name);
            }

            return (it != std::end(names)) ? (it - std::begin(names)) : (-1);
        }

        /// Find a trigger string and call a modify callable function that takes the current string and starting
        /// position of the trigger and returns the position in the string to search for the next trigger string
        template <typename Callable>
        inline std::string find_and_modify(std::string str, std::string trigger, Callable modify)
        {
            std::size_t start_pos = 0;
            while((start_pos = str.find(trigger, start_pos)) != std::string::npos)
            {
                start_pos = modify(str, start_pos);
            }
            return str;
        }

        /// Split a string '"one two" "three"' into 'one two', 'three'
        /// Quote characters can be ` ' or "
        inline std::vector<std::string> split_up(std::string str, char delimiter = '\0')
        {
            const std::string delims("\'\"`");
            auto find_ws = [delimiter](char ch)
            {
                return (delimiter == '\0') ? (std::isspace<char>(ch, std::locale()) != 0) : (ch == delimiter);
            };
            trim(str);

            std::vector<std::string> output;
            bool embeddedQuote = false;
            char keyChar       = ' ';
            while(!str.empty())
            {
                if(delims.find_first_of(str[0]) != std::string::npos)
                {
                    keyChar  = str[0];
                    auto end = str.find_first_of(keyChar, 1);
                    while((end != std::string::npos) && (str[end - 1] == '\\'))
                    {  // deal with escaped quotes
                        end           = str.find_first_of(keyChar, end + 1);
                        embeddedQuote = true;
                    }
                    if(end != std::string::npos)
                    {
                        output.push_back(str.substr(1, end - 1));
                        if(end + 2 < str.size())
                        {
                            str = str.substr(end + 2);
                        }
                        else
                        {
                            str.clear();
                        }
                    }
                    else
                    {
                        output.push_back(str.substr(1));
                        str = "";
                    }
                }
                else
                {
                    auto it = std::find_if(std::begin(str), std::end(str), find_ws);
                    if(it != std::end(str))
                    {
                        std::string value = std::string(str.begin(), it);
                        output.push_back(value);
                        str = std::string(it + 1, str.end());
                    }
                    else
                    {
                        output.push_back(str);
                        str = "";
                    }
                }
                // transform any embedded quotes into the regular character
                if(embeddedQuote)
                {
                    output.back() =
                        find_and_replace(output.back(), std::string("\\") + keyChar, std::string(1, keyChar));
                    embeddedQuote = false;
                }
                trim(str);
            }
            return output;
        }

        /// Add a leader to the beginning of all new lines (nothing is added
        /// at the start of the first line). `"; "` would be for ini files
        ///
        /// Can't use Regex, or this would be a subs.
        inline std::string fix_newlines(const std::string &leader, std::string input)
        {
            std::string::size_type n = 0;
            while(n != std::string::npos && n < input.size())
            {
                n = input.find('\n', n);
                if(n != std::string::npos)
                {
                    input = input.substr(0, n + 1) + leader + input.substr(n + 1);
                    n += leader.size();
                }
            }
            return input;
        }

        /// This function detects an equal or colon followed by an escaped quote after an argument
        /// then modifies the string to replace the equality with a space.  This is needed
        /// to allow the split up function to work properly and is intended to be used with the find_and_modify function
        /// the return value is the offset+1 which is required by the find_and_modify function.
        inline std::size_t escape_detect(std::string &str, std::size_t offset)
        {
            auto next = str[offset + 1];
            if((next == '\"') || (next == '\'') || (next == '`'))
            {
                auto astart = str.find_last_of("-/ \"\'`", offset - 1);
                if(astart != std::string::npos)
                {
                    if(str[astart] == ((str[offset] == '=') ? '-' : '/'))
                        str[offset] = ' ';  // interpret this as a space so the split_up works properly
                }
            }
            return offset + 1;
        }

        /// Add quotes if the string contains spaces
        inline std::string &add_quotes_if_needed(std::string &str)
        {
            if((str.front() != '"' && str.front() != '\'') || str.front() != str.back())
            {
                char quote = str.find('"') < str.find('\'') ? '\'' : '"';
                if(str.find(' ') != std::string::npos)
                {
                    str.insert(0, 1, quote);
                    str.append(1, quote);
                }
            }
            return str;
        }

    }  // namespace detail

    // Use one of these on all error classes.
    // These are temporary and are undef'd at the end of this file.
#define CLI11_ERROR_DEF(parent, name)                                                                                  \
   protected:                                                                                                          \
    name(std::string ename, std::string msg, int exit_code)                                                            \
        : parent(std::move(ename), std::move(msg), exit_code)                                                          \
    {}                                                                                                                 \
    name(std::string ename, std::string msg, ExitCodes exit_code)                                                      \
        : parent(std::move(ename), std::move(msg), exit_code)                                                          \
    {}                                                                                                                 \
                                                                                                                       \
   public:                                                                                                             \
    name(std::string msg, ExitCodes exit_code)                                                                         \
        : parent(#name, std::move(msg), exit_code)                                                                     \
    {}                                                                                                                 \
    name(std::string msg, int exit_code)                                                                               \
        : parent(#name, std::move(msg), exit_code)                                                                     \
    {}

// This is added after the one above if a class is used directly and builds its own message
#define CLI11_ERROR_SIMPLE(name)                                                                                       \
    explicit name(std::string msg)                                                                                     \
        : name(#name, msg, ExitCodes::name)                                                                            \
    {}

    /// These codes are part of every error in CLI. They can be obtained from e using e.exit_code or as a quick
    /// shortcut, int values from e.get_error_code().
    enum class ExitCodes
    {
        Success               = 0,
        IncorrectConstruction = 100,
        BadNameString,
        OptionAlreadyAdded,
        FileError,
        ConversionError,
        ValidationError,
        RequiredError,
        RequiresError,
        ExcludesError,
        ExtrasError,
        ConfigError,
        InvalidError,
        HorribleError,
        OptionNotFound,
        ArgumentMismatch,
        BaseClass = 127
    };

    // Error definitions

    /// @defgroup error_group Errors
    /// @brief Errors thrown by CLI11
    ///
    /// These are the errors that can be thrown. Some of them, like CLI::Success, are not really errors.
    /// @{

    /// All errors derive from this one
    class Error : public std::runtime_error
    {
        int actual_exit_code;
        std::string error_name{"Error"};

       public:
        int get_exit_code() const
        {
            return actual_exit_code;
        }

        std::string get_name() const
        {
            return error_name;
        }

        Error(std::string name, std::string msg, int exit_code = static_cast<int>(ExitCodes::BaseClass))
            : runtime_error(msg)
            , actual_exit_code(exit_code)
            , error_name(std::move(name))
        {}

        Error(std::string name, std::string msg, ExitCodes exit_code)
            : Error(name, msg, static_cast<int>(exit_code))
        {}
    };

    // Note: Using Error::Error constructors does not work on GCC 4.7

    /// Construction errors (not in parsing)
    class ConstructionError : public Error
    {
        CLI11_ERROR_DEF(Error, ConstructionError)
    };

    /// Thrown when an option is set to conflicting values (non-vector and multi args, for example)
    class IncorrectConstruction : public ConstructionError
    {
        CLI11_ERROR_DEF(ConstructionError, IncorrectConstruction)
        CLI11_ERROR_SIMPLE(IncorrectConstruction)
        static IncorrectConstruction PositionalFlag(std::string name)
        {
            return IncorrectConstruction(name + ": Flags cannot be positional");
        }
        static IncorrectConstruction Set0Opt(std::string name)
        {
            return IncorrectConstruction(name + ": Cannot set 0 expected, use a flag instead");
        }
        static IncorrectConstruction SetFlag(std::string name)
        {
            return IncorrectConstruction(name + ": Cannot set an expected number for flags");
        }
        static IncorrectConstruction ChangeNotVector(std::string name)
        {
            return IncorrectConstruction(name + ": You can only change the expected arguments for vectors");
        }
        static IncorrectConstruction AfterMultiOpt(std::string name)
        {
            return IncorrectConstruction(
                name + ": You can't change expected arguments after you've changed the multi option policy!");
        }
        static IncorrectConstruction MissingOption(std::string name)
        {
            return IncorrectConstruction("Option " + name + " is not defined");
        }
        static IncorrectConstruction MultiOptionPolicy(std::string name)
        {
            return IncorrectConstruction(name + ": multi_option_policy only works for flags and exact value options");
        }
    };

    /// Thrown on construction of a bad name
    class BadNameString : public ConstructionError
    {
        CLI11_ERROR_DEF(ConstructionError, BadNameString)
        CLI11_ERROR_SIMPLE(BadNameString)
        static BadNameString OneCharName(std::string name)
        {
            return BadNameString("Invalid one char name: " + name);
        }
        static BadNameString BadLongName(std::string name)
        {
            return BadNameString("Bad long name: " + name);
        }
        static BadNameString DashesOnly(std::string name)
        {
            return BadNameString("Must have a name, not just dashes: " + name);
        }
        static BadNameString MultiPositionalNames(std::string name)
        {
            return BadNameString("Only one positional name allowed, remove: " + name);
        }
    };

    /// Thrown when an option already exists
    class OptionAlreadyAdded : public ConstructionError
    {
        CLI11_ERROR_DEF(ConstructionError, OptionAlreadyAdded)
        explicit OptionAlreadyAdded(std::string name)
            : OptionAlreadyAdded(name + " is already added", ExitCodes::OptionAlreadyAdded)
        {}
        static OptionAlreadyAdded Requires(std::string name, std::string other)
        {
            return OptionAlreadyAdded(name + " requires " + other, ExitCodes::OptionAlreadyAdded);
        }
        static OptionAlreadyAdded Excludes(std::string name, std::string other)
        {
            return OptionAlreadyAdded(name + " excludes " + other, ExitCodes::OptionAlreadyAdded);
        }
    };

    // Parsing errors

    /// Anything that can error in Parse
    class ParseError : public Error
    {
        CLI11_ERROR_DEF(Error, ParseError)
    };

    // Not really "errors"

    /// This is a successful completion on parsing, supposed to exit
    class Success : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, Success)
        Success()
            : Success("Successfully completed, should be caught and quit", ExitCodes::Success)
        {}
    };

    /// -h or --help on command line
    class CallForHelp : public Success
    {
        CLI11_ERROR_DEF(Success, CallForHelp)
        CallForHelp()
            : CallForHelp("This should be caught in your main function, see examples", ExitCodes::Success)
        {}
    };

    /// Usually something like --help-all on command line
    class CallForAllHelp : public Success
    {
        CLI11_ERROR_DEF(Success, CallForAllHelp)
        CallForAllHelp()
            : CallForAllHelp("This should be caught in your main function, see examples", ExitCodes::Success)
        {}
    };

    /// -v or --version on command line
    class CallForVersion : public Success
    {
        CLI11_ERROR_DEF(Success, CallForVersion)
        CallForVersion()
            : CallForVersion("This should be caught in your main function, see examples", ExitCodes::Success)
        {}
    };

    /// Does not output a diagnostic in CLI11_PARSE, but allows main() to return with a specific error code.
    class RuntimeError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, RuntimeError)
        explicit RuntimeError(int exit_code = 1)
            : RuntimeError("Runtime error", exit_code)
        {}
    };

    /// Thrown when parsing an INI file and it is missing
    class FileError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, FileError)
        CLI11_ERROR_SIMPLE(FileError)
        static FileError Missing(std::string name)
        {
            return FileError(name + " was not readable (missing?)");
        }
    };

    /// Thrown when conversion call back fails, such as when an int fails to coerce to a string
    class ConversionError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, ConversionError)
        CLI11_ERROR_SIMPLE(ConversionError)
        ConversionError(std::string member, std::string name)
            : ConversionError("The value " + member + " is not an allowed value for " + name)
        {}
        ConversionError(std::string name, std::vector<std::string> results)
            : ConversionError("Could not convert: " + name + " = " + detail::join(results))
        {}
        static ConversionError TooManyInputsFlag(std::string name)
        {
            return ConversionError(name + ": too many inputs for a flag");
        }
        static ConversionError TrueFalse(std::string name)
        {
            return ConversionError(name + ": Should be true/false or a number");
        }
    };

    /// Thrown when validation of results fails
    class ValidationError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, ValidationError)
        CLI11_ERROR_SIMPLE(ValidationError)
        explicit ValidationError(std::string name, std::string msg)
            : ValidationError(name + ": " + msg)
        {}
    };

    /// Thrown when a required option is missing
    class RequiredError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, RequiredError)
        explicit RequiredError(std::string name)
            : RequiredError(name + " is required", ExitCodes::RequiredError)
        {}
        static RequiredError Subcommand(std::size_t min_subcom)
        {
            if(min_subcom == 1)
            {
                return RequiredError("A subcommand");
            }
            return RequiredError("Requires at least " + std::to_string(min_subcom) + " subcommands",
                                 ExitCodes::RequiredError);
        }
        static RequiredError Option(std::size_t min_option,
                                    std::size_t max_option,
                                    std::size_t used,
                                    const std::string &option_list)
        {
            if((min_option == 1) && (max_option == 1) && (used == 0))
                return RequiredError("Exactly 1 option from [" + option_list + "]");
            if((min_option == 1) && (max_option == 1) && (used > 1))
            {
                return RequiredError("Exactly 1 option from [" + option_list + "] is required and " +
                                         std::to_string(used) + " were given",
                                     ExitCodes::RequiredError);
            }
            if((min_option == 1) && (used == 0))
                return RequiredError("At least 1 option from [" + option_list + "]");
            if(used < min_option)
            {
                return RequiredError("Requires at least " + std::to_string(min_option) + " options used and only " +
                                         std::to_string(used) + "were given from [" + option_list + "]",
                                     ExitCodes::RequiredError);
            }
            if(max_option == 1)
                return RequiredError("Requires at most 1 options be given from [" + option_list + "]",
                                     ExitCodes::RequiredError);

            return RequiredError("Requires at most " + std::to_string(max_option) + " options be used and " +
                                     std::to_string(used) + "were given from [" + option_list + "]",
                                 ExitCodes::RequiredError);
        }
    };

    /// Thrown when the wrong number of arguments has been received
    class ArgumentMismatch : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, ArgumentMismatch)
        CLI11_ERROR_SIMPLE(ArgumentMismatch)
        ArgumentMismatch(std::string name, int expected, std::size_t received)
            : ArgumentMismatch(expected > 0 ? ("Expected exactly " + std::to_string(expected) + " arguments to " +
                                               name + ", got " + std::to_string(received))
                                            : ("Expected at least " + std::to_string(-expected) + " arguments to " +
                                               name + ", got " + std::to_string(received)),
                               ExitCodes::ArgumentMismatch)
        {}

        static ArgumentMismatch AtLeast(std::string name, int num, std::size_t received)
        {
            return ArgumentMismatch(name + ": At least " + std::to_string(num) + " required but received " +
                                    std::to_string(received));
        }
        static ArgumentMismatch AtMost(std::string name, int num, std::size_t received)
        {
            return ArgumentMismatch(name + ": At Most " + std::to_string(num) + " required but received " +
                                    std::to_string(received));
        }
        static ArgumentMismatch TypedAtLeast(std::string name, int num, std::string type)
        {
            return ArgumentMismatch(name + ": " + std::to_string(num) + " required " + type + " missing");
        }
        static ArgumentMismatch FlagOverride(std::string name)
        {
            return ArgumentMismatch(name + " was given a disallowed flag override");
        }
    };

    /// Thrown when a requires option is missing
    class RequiresError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, RequiresError)
        RequiresError(std::string curname, std::string subname)
            : RequiresError(curname + " requires " + subname, ExitCodes::RequiresError)
        {}
    };

    /// Thrown when an excludes option is present
    class ExcludesError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, ExcludesError)
        ExcludesError(std::string curname, std::string subname)
            : ExcludesError(curname + " excludes " + subname, ExitCodes::ExcludesError)
        {}
    };

    /// Thrown when too many positionals or options are found
    class ExtrasError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, ExtrasError)
        explicit ExtrasError(std::vector<std::string> args)
            : ExtrasError((args.size() > 1 ? "The following arguments were not expected: "
                                           : "The following argument was not expected: ") +
                              detail::rjoin(args, " "),
                          ExitCodes::ExtrasError)
        {}
        ExtrasError(const std::string &name, std::vector<std::string> args)
            : ExtrasError(name,
                          (args.size() > 1 ? "The following arguments were not expected: "
                                           : "The following argument was not expected: ") +
                              detail::rjoin(args, " "),
                          ExitCodes::ExtrasError)
        {}
    };

    /// Thrown when extra values are found in an INI file
    class ConfigError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, ConfigError)
        CLI11_ERROR_SIMPLE(ConfigError)
        static ConfigError Extras(std::string item)
        {
            return ConfigError("INI was not able to parse " + item);
        }
        static ConfigError NotConfigurable(std::string item)
        {
            return ConfigError(item + ": This option is not allowed in a configuration file");
        }
    };

    /// Thrown when validation fails before parsing
    class InvalidError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, InvalidError)
        explicit InvalidError(std::string name)
            : InvalidError(name + ": Too many positional arguments with unlimited expected args",
                           ExitCodes::InvalidError)
        {}
    };

    /// This is just a safety check to verify selection and parsing match - you should not ever see it
    /// Strings are directly added to this error, but again, it should never be seen.
    class HorribleError : public ParseError
    {
        CLI11_ERROR_DEF(ParseError, HorribleError)
        CLI11_ERROR_SIMPLE(HorribleError)
    };

    // After parsing

    /// Thrown when counting a non-existent option
    class OptionNotFound : public Error
    {
        CLI11_ERROR_DEF(Error, OptionNotFound)
        explicit OptionNotFound(std::string name)
            : OptionNotFound(name + " not found", ExitCodes::OptionNotFound)
        {}
    };

#undef CLI11_ERROR_DEF
#undef CLI11_ERROR_SIMPLE

    /// @}

    // Type tools

    // Utilities for type enabling
    namespace detail
    {
        // Based generally on https://rmf.io/cxx11/almost-static-if
        /// Simple empty scoped class
        enum class enabler
        {};

        /// An instance to use in EnableIf
        constexpr enabler dummy = {};
    }  // namespace detail

    /// A copy of enable_if_t from C++14, compatible with C++11.
    ///
    /// We could check to see if C++14 is being used, but it does not hurt to redefine this
    /// (even Google does this: https://github.com/google/skia/blob/master/include/private/SkTLogic.h)
    /// It is not in the std namespace anyway, so no harm done.
    template <bool B, class T = void>
    using enable_if_t = typename std::enable_if<B, T>::type;

    /// A copy of std::void_t from C++17 (helper for C++11 and C++14)
    template <typename... Ts>
    struct make_void
    {
        using type = void;
    };

    /// A copy of std::void_t from C++17 - same reasoning as enable_if_t, it does not hurt to redefine
    template <typename... Ts>
    using void_t = typename make_void<Ts...>::type;

    /// A copy of std::conditional_t from C++14 - same reasoning as enable_if_t, it does not hurt to redefine
    template <bool B, class T, class F>
    using conditional_t = typename std::conditional<B, T, F>::type;

    /// Check to see if something is bool (fail check by default)
    template <typename T>
    struct is_bool : std::false_type
    {};

    /// Check to see if something is bool (true if actually a bool)
    template <>
    struct is_bool<bool> : std::true_type
    {};

    /// Check to see if something is a shared pointer
    template <typename T>
    struct is_shared_ptr : std::false_type
    {};

    /// Check to see if something is a shared pointer (True if really a shared pointer)
    template <typename T>
    struct is_shared_ptr<std::shared_ptr<T>> : std::true_type
    {};

    /// Check to see if something is a shared pointer (True if really a shared pointer)
    template <typename T>
    struct is_shared_ptr<const std::shared_ptr<T>> : std::true_type
    {};

    /// Check to see if something is copyable pointer
    template <typename T>
    struct is_copyable_ptr
    {
        static bool const value = is_shared_ptr<T>::value || std::is_pointer<T>::value;
    };

    /// This can be specialized to override the type deduction for IsMember.
    template <typename T>
    struct IsMemberType
    {
        using type = T;
    };

    /// The main custom type needed here is const char * should be a string.
    template <>
    struct IsMemberType<const char *>
    {
        using type = std::string;
    };

    namespace detail
    {

        // These are utilities for IsMember and other transforming objects

        /// Handy helper to access the element_type generically. This is not part of is_copyable_ptr because it requires
        /// that pointer_traits<T> be valid.

        /// not a pointer
        template <typename T, typename Enable = void>
        struct element_type
        {
            using type = T;
        };

        template <typename T>
        struct element_type<T, typename std::enable_if<is_copyable_ptr<T>::value>::type>
        {
            using type = typename std::pointer_traits<T>::element_type;
        };

        /// Combination of the element type and value type - remove pointer (including smart pointers) and get the
        /// value_type of the container
        template <typename T>
        struct element_value_type
        {
            using type = typename element_type<T>::type::value_type;
        };

        /// Adaptor for set-like structure: This just wraps a normal container in a few utilities that do almost
        /// nothing.
        template <typename T, typename _ = void>
        struct pair_adaptor : std::false_type
        {
            using value_type  = typename T::value_type;
            using first_type  = typename std::remove_const<value_type>::type;
            using second_type = typename std::remove_const<value_type>::type;

            /// Get the first value (really just the underlying value)
            template <typename Q>
            static auto first(Q &&pair_value) -> decltype(std::forward<Q>(pair_value))
            {
                return std::forward<Q>(pair_value);
            }
            /// Get the second value (really just the underlying value)
            template <typename Q>
            static auto second(Q &&pair_value) -> decltype(std::forward<Q>(pair_value))
            {
                return std::forward<Q>(pair_value);
            }
        };

        /// Adaptor for map-like structure (true version, must have key_type and mapped_type).
        /// This wraps a mapped container in a few utilities access it in a general way.
        template <typename T>
        struct pair_adaptor<
            T,
            conditional_t<false, void_t<typename T::value_type::first_type, typename T::value_type::second_type>, void>>
            : std::true_type
        {
            using value_type  = typename T::value_type;
            using first_type  = typename std::remove_const<typename value_type::first_type>::type;
            using second_type = typename std::remove_const<typename value_type::second_type>::type;

            /// Get the first value (really just the underlying value)
            template <typename Q>
            static auto first(Q &&pair_value) -> decltype(std::get<0>(std::forward<Q>(pair_value)))
            {
                return std::get<0>(std::forward<Q>(pair_value));
            }
            /// Get the second value (really just the underlying value)
            template <typename Q>
            static auto second(Q &&pair_value) -> decltype(std::get<1>(std::forward<Q>(pair_value)))
            {
                return std::get<1>(std::forward<Q>(pair_value));
            }
        };

        // Warning is suppressed due to "bug" in gcc<5.0 and gcc 7.0 with c++17 enabled that generates a Wnarrowing
        // warning in the unevaluated context even if the function that was using this wasn't used.  The standard says
        // narrowing in brace initialization shouldn't be allowed but for backwards compatibility gcc allows it in some
        // contexts.  It is a little fuzzy what happens in template constructs and I think that was something GCC took a
        // little while to work out. But regardless some versions of gcc generate a warning when they shouldn't from the
        // following code so that should be suppressed
#ifdef __GNUC__
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wnarrowing"
#endif
        // check for constructibility from a specific type and copy assignable used in the parse detection
        template <typename T, typename C>
        class is_direct_constructible
        {
            template <typename TT, typename CC>
            static auto test(int, std::true_type) -> decltype(
            // NVCC warns about narrowing conversions here
#ifdef __CUDACC__
#    pragma diag_suppress 2361
#endif
                TT{std::declval<CC>()}
#ifdef __CUDACC__
#    pragma diag_default 2361
#endif
                ,
                std::is_move_assignable<TT>());

            template <typename TT, typename CC>
            static auto test(int, std::false_type) -> std::false_type;

            template <typename, typename>
            static auto test(...) -> std::false_type;

           public:
            static constexpr bool value = decltype(test<T, C>(0, typename std::is_constructible<T, C>::type()))::value;
        };
#ifdef __GNUC__
#    pragma GCC diagnostic pop
#endif

        // Check for output streamability
        // Based on
        // https://stackoverflow.com/questions/22758291/how-can-i-detect-if-a-type-can-be-streamed-to-an-stdostream

        template <typename T, typename S = std::ostringstream>
        class is_ostreamable
        {
            template <typename TT, typename SS>
            static auto test(int) -> decltype(std::declval<SS &>() << std::declval<TT>(), std::true_type());

            template <typename, typename>
            static auto test(...) -> std::false_type;

           public:
            static constexpr bool value = decltype(test<T, S>(0))::value;
        };

        /// Check for input streamability
        template <typename T, typename S = std::istringstream>
        class is_istreamable
        {
            template <typename TT, typename SS>
            static auto test(int) -> decltype(std::declval<SS &>() >> std::declval<TT &>(), std::true_type());

            template <typename, typename>
            static auto test(...) -> std::false_type;

           public:
            static constexpr bool value = decltype(test<T, S>(0))::value;
        };

        /// Check for complex
        template <typename T>
        class is_complex
        {
            template <typename TT>
            static auto test(int) -> decltype(std::declval<TT>().real(), std::declval<TT>().imag(), std::true_type());

            template <typename>
            static auto test(...) -> std::false_type;

           public:
            static constexpr bool value = decltype(test<T>(0))::value;
        };

        /// Templated operation to get a value from a stream
        template <typename T, enable_if_t<is_istreamable<T>::value, detail::enabler> = detail::dummy>
        bool from_stream(const std::string &istring, T &obj)
        {
            std::istringstream is;
            is.str(istring);
            is >> obj;
            return !is.fail() && !is.rdbuf()->in_avail();
        }

        template <typename T, enable_if_t<!is_istreamable<T>::value, detail::enabler> = detail::dummy>
        bool from_stream(const std::string & /*istring*/, T & /*obj*/)
        {
            return false;
        }

        // check to see if an object is a mutable container (fail by default)
        template <typename T, typename _ = void>
        struct is_mutable_container : std::false_type
        {};

        /// type trait to test if a type is a mutable container meaning it has a value_type, it has an iterator, a
        /// clear, and end methods and an insert function.  And for our purposes we exclude std::string and types that
        /// can be constructed from a std::string
        template <typename T>
        struct is_mutable_container<
            T,
            conditional_t<false,
                          void_t<typename T::value_type,
                                 decltype(std::declval<T>().end()),
                                 decltype(std::declval<T>().clear()),
                                 decltype(std::declval<T>().insert(std::declval<decltype(std::declval<T>().end())>(),
                                                                   std::declval<const typename T::value_type &>()))>,
                          void>>
            : public conditional_t<std::is_constructible<T, std::string>::value, std::false_type, std::true_type>
        {};

        // check to see if an object is a mutable container (fail by default)
        template <typename T, typename _ = void>
        struct is_readable_container : std::false_type
        {};

        /// type trait to test if a type is a container meaning it has a value_type, it has an iterator, a clear, and an
        /// end methods and an insert function.  And for our purposes we exclude std::string and types that can be
        /// constructed from a std::string
        template <typename T>
        struct is_readable_container<
            T,
            conditional_t<false, void_t<decltype(std::declval<T>().end()), decltype(std::declval<T>().begin())>, void>>
            : public std::true_type
        {};

        // check to see if an object is a wrapper (fail by default)
        template <typename T, typename _ = void>
        struct is_wrapper : std::false_type
        {};

        // check if an object is a wrapper (it has a value_type defined)
        template <typename T>
        struct is_wrapper<T, conditional_t<false, void_t<typename T::value_type>, void>> : public std::true_type
        {};

        // Check for tuple like types, as in classes with a tuple_size type trait
        template <typename S>
        class is_tuple_like
        {
            template <typename SS>
            // static auto test(int)
            //     -> decltype(std::conditional<(std::tuple_size<SS>::value > 0), std::true_type,
            //     std::false_type>::type());
            static auto test(int) -> decltype(std::tuple_size<typename std::decay<SS>::type>::value, std::true_type{});
            template <typename>
            static auto test(...) -> std::false_type;

           public:
            static constexpr bool value = decltype(test<S>(0))::value;
        };

        /// Convert an object to a string (directly forward if this can become a string)
        template <typename T, enable_if_t<std::is_convertible<T, std::string>::value, detail::enabler> = detail::dummy>
        auto to_string(T &&value) -> decltype(std::forward<T>(value))
        {
            return std::forward<T>(value);
        }

        /// Construct a string from the object
        template <
            typename T,
            enable_if_t<std::is_constructible<std::string, T>::value && !std::is_convertible<T, std::string>::value,
                        detail::enabler> = detail::dummy>
        std::string to_string(const T &value)
        {
            return std::string(value);
        }

        /// Convert an object to a string (streaming must be supported for that type)
        template <typename T,
                  enable_if_t<!std::is_convertible<std::string, T>::value &&
                                  !std::is_constructible<std::string, T>::value && is_ostreamable<T>::value,
                              detail::enabler> = detail::dummy>
        std::string to_string(T &&value)
        {
            std::stringstream stream;
            stream << value;
            return stream.str();
        }

        /// If conversion is not supported, return an empty string (streaming is not supported for that type)
        template <typename T,
                  enable_if_t<!std::is_constructible<std::string, T>::value && !is_ostreamable<T>::value &&
                                  !is_readable_container<typename std::remove_const<T>::type>::value,
                              detail::enabler> = detail::dummy>
        std::string to_string(T &&)
        {
            return std::string{};
        }

        /// convert a readable container to a string
        template <typename T,
                  enable_if_t<!std::is_constructible<std::string, T>::value && !is_ostreamable<T>::value &&
                                  is_readable_container<T>::value,
                              detail::enabler> = detail::dummy>
        std::string to_string(T &&variable)
        {
            std::vector<std::string> defaults;
            auto cval = variable.begin();
            auto end  = variable.end();
            while(cval != end)
            {
                defaults.emplace_back(CLI::detail::to_string(*cval));
                ++cval;
            }
            return std::string("[" + detail::join(defaults) + "]");
        }

        /// special template overload
        template <typename T1,
                  typename T2,
                  typename T,
                  enable_if_t<std::is_same<T1, T2>::value, detail::enabler> = detail::dummy>
        auto checked_to_string(T &&value) -> decltype(to_string(std::forward<T>(value)))
        {
            return to_string(std::forward<T>(value));
        }

        /// special template overload
        template <typename T1,
                  typename T2,
                  typename T,
                  enable_if_t<!std::is_same<T1, T2>::value, detail::enabler> = detail::dummy>
        std::string checked_to_string(T &&)
        {
            return std::string{};
        }
        /// get a string as a convertible value for arithmetic types
        template <typename T, enable_if_t<std::is_arithmetic<T>::value, detail::enabler> = detail::dummy>
        std::string value_string(const T &value)
        {
            return std::to_string(value);
        }
        /// get a string as a convertible value for enumerations
        template <typename T, enable_if_t<std::is_enum<T>::value, detail::enabler> = detail::dummy>
        std::string value_string(const T &value)
        {
            return std::to_string(static_cast<typename std::underlying_type<T>::type>(value));
        }
        /// for other types just use the regular to_string function
        template <
            typename T,
            enable_if_t<!std::is_enum<T>::value && !std::is_arithmetic<T>::value, detail::enabler> = detail::dummy>
        auto value_string(const T &value) -> decltype(to_string(value))
        {
            return to_string(value);
        }

        /// template to get the underlying value type if it exists or use a default
        template <typename T, typename def, typename Enable = void>
        struct wrapped_type
        {
            using type = def;
        };

        /// Type size for regular object types that do not look like a tuple
        template <typename T, typename def>
        struct wrapped_type<T, def, typename std::enable_if<is_wrapper<T>::value>::type>
        {
            using type = typename T::value_type;
        };

        /// This will only trigger for actual void type
        template <typename T, typename Enable = void>
        struct type_count_base
        {
            static const int value{0};
        };

        /// Type size for regular object types that do not look like a tuple
        template <typename T>
        struct type_count_base<T,
                               typename std::enable_if<!is_tuple_like<T>::value && !is_mutable_container<T>::value &&
                                                       !std::is_void<T>::value>::type>
        {
            static constexpr int value{1};
        };

        /// the base tuple size
        template <typename T>
        struct type_count_base<
            T,
            typename std::enable_if<is_tuple_like<T>::value && !is_mutable_container<T>::value>::type>
        {
            static constexpr int value{std::tuple_size<T>::value};
        };

        /// Type count base for containers is the type_count_base of the individual element
        template <typename T>
        struct type_count_base<T, typename std::enable_if<is_mutable_container<T>::value>::type>
        {
            static constexpr int value{type_count_base<typename T::value_type>::value};
        };

        /// Set of overloads to get the type size of an object

        /// forward declare the subtype_count structure
        template <typename T>
        struct subtype_count;

        /// forward declare the subtype_count_min structure
        template <typename T>
        struct subtype_count_min;

        /// This will only trigger for actual void type
        template <typename T, typename Enable = void>
        struct type_count
        {
            static const int value{0};
        };

        /// Type size for regular object types that do not look like a tuple
        template <typename T>
        struct type_count<T,
                          typename std::enable_if<!is_wrapper<T>::value && !is_tuple_like<T>::value &&
                                                  !is_complex<T>::value && !std::is_void<T>::value>::type>
        {
            static constexpr int value{1};
        };

        /// Type size for complex since it sometimes looks like a wrapper
        template <typename T>
        struct type_count<T, typename std::enable_if<is_complex<T>::value>::type>
        {
            static constexpr int value{2};
        };

        /// Type size of types that are wrappers,except complex and tuples(which can also be wrappers sometimes)
        template <typename T>
        struct type_count<T, typename std::enable_if<is_mutable_container<T>::value>::type>
        {
            static constexpr int value{subtype_count<typename T::value_type>::value};
        };

        /// Type size of types that are wrappers,except containers complex and tuples(which can also be wrappers
        /// sometimes)
        template <typename T>
        struct type_count<T,
                          typename std::enable_if<is_wrapper<T>::value && !is_complex<T>::value &&
                                                  !is_tuple_like<T>::value && !is_mutable_container<T>::value>::type>
        {
            static constexpr int value{type_count<typename T::value_type>::value};
        };

        /// 0 if the index > tuple size
        template <typename T, std::size_t I>
        constexpr typename std::enable_if<I == type_count_base<T>::value, int>::type tuple_type_size()
        {
            return 0;
        }

        /// Recursively generate the tuple type name
        template <typename T, std::size_t I>
            constexpr typename std::enable_if < I<type_count_base<T>::value, int>::type tuple_type_size()
        {
            return subtype_count<typename std::tuple_element<I, T>::type>::value + tuple_type_size<T, I + 1>();
        }

        /// Get the type size of the sum of type sizes for all the individual tuple types
        template <typename T>
        struct type_count<T, typename std::enable_if<is_tuple_like<T>::value>::type>
        {
            static constexpr int value{tuple_type_size<T, 0>()};
        };

        /// definition of subtype count
        template <typename T>
        struct subtype_count
        {
            static constexpr int value{is_mutable_container<T>::value ? expected_max_vector_size
                                                                      : type_count<T>::value};
        };

        /// This will only trigger for actual void type
        template <typename T, typename Enable = void>
        struct type_count_min
        {
            static const int value{0};
        };

        /// Type size for regular object types that do not look like a tuple
        template <typename T>
        struct type_count_min<
            T,
            typename std::enable_if<!is_mutable_container<T>::value && !is_tuple_like<T>::value &&
                                    !is_wrapper<T>::value && !is_complex<T>::value && !std::is_void<T>::value>::type>
        {
            static constexpr int value{type_count<T>::value};
        };

        /// Type size for complex since it sometimes looks like a wrapper
        template <typename T>
        struct type_count_min<T, typename std::enable_if<is_complex<T>::value>::type>
        {
            static constexpr int value{1};
        };

        /// Type size min of types that are wrappers,except complex and tuples(which can also be wrappers sometimes)
        template <typename T>
        struct type_count_min<
            T,
            typename std::enable_if<is_wrapper<T>::value && !is_complex<T>::value && !is_tuple_like<T>::value>::type>
        {
            static constexpr int value{subtype_count_min<typename T::value_type>::value};
        };

        /// 0 if the index > tuple size
        template <typename T, std::size_t I>
        constexpr typename std::enable_if<I == type_count_base<T>::value, int>::type tuple_type_size_min()
        {
            return 0;
        }

        /// Recursively generate the tuple type name
        template <typename T, std::size_t I>
            constexpr typename std::enable_if < I<type_count_base<T>::value, int>::type tuple_type_size_min()
        {
            return subtype_count_min<typename std::tuple_element<I, T>::type>::value + tuple_type_size_min<T, I + 1>();
        }

        /// Get the type size of the sum of type sizes for all the individual tuple types
        template <typename T>
        struct type_count_min<T, typename std::enable_if<is_tuple_like<T>::value>::type>
        {
            static constexpr int value{tuple_type_size_min<T, 0>()};
        };

        /// definition of subtype count
        template <typename T>
        struct subtype_count_min
        {
            static constexpr int value{
                is_mutable_container<T>::value
                    ? ((type_count<T>::value < expected_max_vector_size) ? type_count<T>::value : 0)
                    : type_count_min<T>::value};
        };

        /// This will only trigger for actual void type
        template <typename T, typename Enable = void>
        struct expected_count
        {
            static const int value{0};
        };

        /// For most types the number of expected items is 1
        template <typename T>
        struct expected_count<T,
                              typename std::enable_if<!is_mutable_container<T>::value && !is_wrapper<T>::value &&
                                                      !std::is_void<T>::value>::type>
        {
            static constexpr int value{1};
        };
        /// number of expected items in a vector
        template <typename T>
        struct expected_count<T, typename std::enable_if<is_mutable_container<T>::value>::type>
        {
            static constexpr int value{expected_max_vector_size};
        };

        /// number of expected items in a vector
        template <typename T>
        struct expected_count<T, typename std::enable_if<!is_mutable_container<T>::value && is_wrapper<T>::value>::type>
        {
            static constexpr int value{expected_count<typename T::value_type>::value};
        };

        // Enumeration of the different supported categorizations of objects
        enum class object_category : int
        {
            char_value            = 1,
            integral_value        = 2,
            unsigned_integral     = 4,
            enumeration           = 6,
            boolean_value         = 8,
            floating_point        = 10,
            number_constructible  = 12,
            double_constructible  = 14,
            integer_constructible = 16,
            // string like types
            string_assignable    = 23,
            string_constructible = 24,
            other                = 45,
            // special wrapper or container types
            wrapper_value   = 50,
            complex_number  = 60,
            tuple_value     = 70,
            container_value = 80,
        };

        /// Set of overloads to classify an object according to type

        /// some type that is not otherwise recognized
        template <typename T, typename Enable = void>
        struct classify_object
        {
            static constexpr object_category value{object_category::other};
        };

        /// Signed integers
        template <typename T>
        struct classify_object<
            T,
            typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, char>::value &&
                                    std::is_signed<T>::value && !is_bool<T>::value && !std::is_enum<T>::value>::type>
        {
            static constexpr object_category value{object_category::integral_value};
        };

        /// Unsigned integers
        template <typename T>
        struct classify_object<T,
                               typename std::enable_if<std::is_integral<T>::value && std::is_unsigned<T>::value &&
                                                       !std::is_same<T, char>::value && !is_bool<T>::value>::type>
        {
            static constexpr object_category value{object_category::unsigned_integral};
        };

        /// single character values
        template <typename T>
        struct classify_object<T,
                               typename std::enable_if<std::is_same<T, char>::value && !std::is_enum<T>::value>::type>
        {
            static constexpr object_category value{object_category::char_value};
        };

        /// Boolean values
        template <typename T>
        struct classify_object<T, typename std::enable_if<is_bool<T>::value>::type>
        {
            static constexpr object_category value{object_category::boolean_value};
        };

        /// Floats
        template <typename T>
        struct classify_object<T, typename std::enable_if<std::is_floating_point<T>::value>::type>
        {
            static constexpr object_category value{object_category::floating_point};
        };

        /// String and similar direct assignment
        template <typename T>
        struct classify_object<
            T,
            typename std::enable_if<!std::is_floating_point<T>::value && !std::is_integral<T>::value &&
                                    std::is_assignable<T &, std::string>::value>::type>
        {
            static constexpr object_category value{object_category::string_assignable};
        };

        /// String and similar constructible and copy assignment
        template <typename T>
        struct classify_object<
            T,
            typename std::enable_if<!std::is_floating_point<T>::value && !std::is_integral<T>::value &&
                                    !std::is_assignable<T &, std::string>::value && (type_count<T>::value == 1) &&
                                    std::is_constructible<T, std::string>::value>::type>
        {
            static constexpr object_category value{object_category::string_constructible};
        };

        /// Enumerations
        template <typename T>
        struct classify_object<T, typename std::enable_if<std::is_enum<T>::value>::type>
        {
            static constexpr object_category value{object_category::enumeration};
        };

        template <typename T>
        struct classify_object<T, typename std::enable_if<is_complex<T>::value>::type>
        {
            static constexpr object_category value{object_category::complex_number};
        };

        /// Handy helper to contain a bunch of checks that rule out many common types (integers, string like, floating
        /// point, vectors, and enumerations
        template <typename T>
        struct uncommon_type
        {
            using type =
                typename std::conditional<!std::is_floating_point<T>::value && !std::is_integral<T>::value &&
                                              !std::is_assignable<T &, std::string>::value &&
                                              !std::is_constructible<T, std::string>::value && !is_complex<T>::value &&
                                              !is_mutable_container<T>::value && !std::is_enum<T>::value,
                                          std::true_type,
                                          std::false_type>::type;
            static constexpr bool value = type::value;
        };

        /// wrapper type
        template <typename T>
        struct classify_object<T,
                               typename std::enable_if<(!is_mutable_container<T>::value && is_wrapper<T>::value &&
                                                        !is_tuple_like<T>::value && uncommon_type<T>::value)>::type>
        {
            static constexpr object_category value{object_category::wrapper_value};
        };

        /// Assignable from double or int
        template <typename T>
        struct classify_object<
            T,
            typename std::enable_if<uncommon_type<T>::value && type_count<T>::value == 1 && !is_wrapper<T>::value &&
                                    is_direct_constructible<T, double>::value &&
                                    is_direct_constructible<T, int>::value>::type>
        {
            static constexpr object_category value{object_category::number_constructible};
        };

        /// Assignable from int
        template <typename T>
        struct classify_object<
            T,
            typename std::enable_if<uncommon_type<T>::value && type_count<T>::value == 1 && !is_wrapper<T>::value &&
                                    !is_direct_constructible<T, double>::value &&
                                    is_direct_constructible<T, int>::value>::type>
        {
            static constexpr object_category value{object_category::integer_constructible};
        };

        /// Assignable from double
        template <typename T>
        struct classify_object<
            T,
            typename std::enable_if<uncommon_type<T>::value && type_count<T>::value == 1 && !is_wrapper<T>::value &&
                                    is_direct_constructible<T, double>::value &&
                                    !is_direct_constructible<T, int>::value>::type>
        {
            static constexpr object_category value{object_category::double_constructible};
        };

        /// Tuple type
        template <typename T>
        struct classify_object<
            T,
            typename std::enable_if<is_tuple_like<T>::value &&
                                    ((type_count<T>::value >= 2 && !is_wrapper<T>::value) ||
                                     (uncommon_type<T>::value && !is_direct_constructible<T, double>::value &&
                                      !is_direct_constructible<T, int>::value))>::type>
        {
            static constexpr object_category value{object_category::tuple_value};
            // the condition on this class requires it be like a tuple, but on some compilers (like Xcode) tuples can be
            // constructed from just the first element so tuples of <string, int,int> can be constructed from a string,
            // which could lead to issues so there are two variants of the condition, the first isolates things with a
            // type size >=2 mainly to get tuples on Xcode with the exception of wrappers, the second is the main one
            // and just separating out those cases that are caught by other object classifications
        };

        /// container type
        template <typename T>
        struct classify_object<T, typename std::enable_if<is_mutable_container<T>::value>::type>
        {
            static constexpr object_category value{object_category::container_value};
        };

        // Type name print

        /// Was going to be based on
        ///  http://stackoverflow.com/questions/1055452/c-get-name-of-type-in-template
        /// But this is cleaner and works better in this case

        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::char_value, detail::enabler> = detail::dummy>
        constexpr const char *type_name()
        {
            return "CHAR";
        }

        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::integral_value ||
                                  classify_object<T>::value == object_category::integer_constructible,
                              detail::enabler> = detail::dummy>
        constexpr const char *type_name()
        {
            return "INT";
        }

        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::unsigned_integral, detail::enabler> =
                      detail::dummy>
        constexpr const char *type_name()
        {
            return "UINT";
        }

        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::floating_point ||
                                  classify_object<T>::value == object_category::number_constructible ||
                                  classify_object<T>::value == object_category::double_constructible,
                              detail::enabler> = detail::dummy>
        constexpr const char *type_name()
        {
            return "FLOAT";
        }

        /// Print name for enumeration types
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::enumeration, detail::enabler> = detail::dummy>
        constexpr const char *type_name()
        {
            return "ENUM";
        }

        /// Print name for enumeration types
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::boolean_value, detail::enabler> = detail::dummy>
        constexpr const char *type_name()
        {
            return "BOOLEAN";
        }

        /// Print name for enumeration types
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::complex_number, detail::enabler> = detail::dummy>
        constexpr const char *type_name()
        {
            return "COMPLEX";
        }

        /// Print for all other types
        template <typename T,
                  enable_if_t<classify_object<T>::value >= object_category::string_assignable &&
                                  classify_object<T>::value <= object_category::other,
                              detail::enabler> = detail::dummy>
        constexpr const char *type_name()
        {
            return "TEXT";
        }
        /// typename for tuple value
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::tuple_value && type_count_base<T>::value >= 2,
                        detail::enabler> = detail::dummy>
        std::string type_name();  // forward declaration

        /// Generate type name for a wrapper or container value
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::container_value ||
                                  classify_object<T>::value == object_category::wrapper_value,
                              detail::enabler> = detail::dummy>
        std::string type_name();  // forward declaration

        /// Print name for single element tuple types
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::tuple_value && type_count_base<T>::value == 1,
                        detail::enabler> = detail::dummy>
        inline std::string type_name()
        {
            return type_name<typename std::decay<typename std::tuple_element<0, T>::type>::type>();
        }

        /// Empty string if the index > tuple size
        template <typename T, std::size_t I>
        inline typename std::enable_if<I == type_count_base<T>::value, std::string>::type tuple_name()
        {
            return std::string{};
        }

        /// Recursively generate the tuple type name
        template <typename T, std::size_t I>
        inline typename std::enable_if<(I < type_count_base<T>::value), std::string>::type tuple_name()
        {
            std::string str =
                std::string(type_name<typename std::decay<typename std::tuple_element<I, T>::type>::type>()) + ',' +
                tuple_name<T, I + 1>();
            if(str.back() == ',')
                str.pop_back();
            return str;
        }

        /// Print type name for tuples with 2 or more elements
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::tuple_value && type_count_base<T>::value >= 2,
                        detail::enabler>>
        inline std::string type_name()
        {
            auto tname = std::string(1, '[') + tuple_name<T, 0>();
            tname.push_back(']');
            return tname;
        }

        /// get the type name for a type that has a value_type member
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::container_value ||
                                  classify_object<T>::value == object_category::wrapper_value,
                              detail::enabler>>
        inline std::string type_name()
        {
            return type_name<typename T::value_type>();
        }

        // Lexical cast

        /// Convert to an unsigned integral
        template <typename T, enable_if_t<std::is_unsigned<T>::value, detail::enabler> = detail::dummy>
        bool integral_conversion(const std::string &input, T &output) noexcept
        {
            if(input.empty())
            {
                return false;
            }
            char *val               = nullptr;
            std::uint64_t output_ll = std::strtoull(input.c_str(), &val, 0);
            output                  = static_cast<T>(output_ll);
            return val == (input.c_str() + input.size()) && static_cast<std::uint64_t>(output) == output_ll;
        }

        /// Convert to a signed integral
        template <typename T, enable_if_t<std::is_signed<T>::value, detail::enabler> = detail::dummy>
        bool integral_conversion(const std::string &input, T &output) noexcept
        {
            if(input.empty())
            {
                return false;
            }
            char *val              = nullptr;
            std::int64_t output_ll = std::strtoll(input.c_str(), &val, 0);
            output                 = static_cast<T>(output_ll);
            return val == (input.c_str() + input.size()) && static_cast<std::int64_t>(output) == output_ll;
        }

        /// Convert a flag into an integer value  typically binary flags
        inline std::int64_t to_flag_value(std::string val)
        {
            static const std::string trueString("true");
            static const std::string falseString("false");
            if(val == trueString)
            {
                return 1;
            }
            if(val == falseString)
            {
                return -1;
            }
            val = detail::to_lower(val);
            std::int64_t ret;
            if(val.size() == 1)
            {
                if(val[0] >= '1' && val[0] <= '9')
                {
                    return (static_cast<std::int64_t>(val[0]) - '0');
                }
                switch(val[0])
                {
                    case '0':
                    case 'f':
                    case 'n':
                    case '-':
                        ret = -1;
                        break;
                    case 't':
                    case 'y':
                    case '+':
                        ret = 1;
                        break;
                    default:
                        throw std::invalid_argument("unrecognized character");
                }
                return ret;
            }
            if(val == trueString || val == "on" || val == "yes" || val == "enable")
            {
                ret = 1;
            }
            else if(val == falseString || val == "off" || val == "no" || val == "disable")
            {
                ret = -1;
            }
            else
            {
                ret = std::stoll(val);
            }
            return ret;
        }

        /// Integer conversion
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::integral_value ||
                                  classify_object<T>::value == object_category::unsigned_integral,
                              detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            return integral_conversion(input, output);
        }

        /// char values
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::char_value, detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            if(input.size() == 1)
            {
                output = static_cast<T>(input[0]);
                return true;
            }
            return integral_conversion(input, output);
        }

        /// Boolean values
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::boolean_value, detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            try
            {
                auto out = to_flag_value(input);
                output   = (out > 0);
                return true;
            }
            catch(const std::invalid_argument &)
            {
                return false;
            }
            catch(const std::out_of_range &)
            {
                // if the number is out of the range of a 64 bit value then it is still a number and for this purpose is
                // still valid all we care about the sign
                output = (input[0] != '-');
                return true;
            }
        }

        /// Floats
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::floating_point, detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            if(input.empty())
            {
                return false;
            }
            char *val      = nullptr;
            auto output_ld = std::strtold(input.c_str(), &val);
            output         = static_cast<T>(output_ld);
            return val == (input.c_str() + input.size());
        }

        /// complex
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::complex_number, detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            using XC = typename wrapped_type<T, double>::type;
            XC x{0.0}, y{0.0};
            auto str1   = input;
            bool worked = false;
            auto nloc   = str1.find_last_of("+-");
            if(nloc != std::string::npos && nloc > 0)
            {
                worked = detail::lexical_cast(str1.substr(0, nloc), x);
                str1   = str1.substr(nloc);
                if(str1.back() == 'i' || str1.back() == 'j')
                    str1.pop_back();
                worked = worked && detail::lexical_cast(str1, y);
            }
            else
            {
                if(str1.back() == 'i' || str1.back() == 'j')
                {
                    str1.pop_back();
                    worked = detail::lexical_cast(str1, y);
                    x      = XC{0};
                }
                else
                {
                    worked = detail::lexical_cast(str1, x);
                    y      = XC{0};
                }
            }
            if(worked)
            {
                output = T{x, y};
                return worked;
            }
            return from_stream(input, output);
        }

        /// String and similar direct assignment
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::string_assignable, detail::enabler> =
                      detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            output = input;
            return true;
        }

        /// String and similar constructible and copy assignment
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::string_constructible, detail::enabler> =
                      detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            output = T(input);
            return true;
        }

        /// Enumerations
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::enumeration, detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            typename std::underlying_type<T>::type val;
            if(!integral_conversion(input, val))
            {
                return false;
            }
            output = static_cast<T>(val);
            return true;
        }

        /// wrapper types
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::wrapper_value &&
                                  std::is_assignable<T &, typename T::value_type>::value,
                              detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            typename T::value_type val;
            if(lexical_cast(input, val))
            {
                output = val;
                return true;
            }
            return from_stream(input, output);
        }

        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::wrapper_value &&
                                  !std::is_assignable<T &, typename T::value_type>::value &&
                                  std::is_assignable<T &, T>::value,
                              detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            typename T::value_type val;
            if(lexical_cast(input, val))
            {
                output = T{val};
                return true;
            }
            return from_stream(input, output);
        }

        /// Assignable from double or int
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::number_constructible, detail::enabler> =
                      detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            int val;
            if(integral_conversion(input, val))
            {
                output = T(val);
                return true;
            }
            else
            {
                double dval;
                if(lexical_cast(input, dval))
                {
                    output = T{dval};
                    return true;
                }
            }
            return from_stream(input, output);
        }

        /// Assignable from int
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::integer_constructible, detail::enabler> =
                      detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            int val;
            if(integral_conversion(input, val))
            {
                output = T(val);
                return true;
            }
            return from_stream(input, output);
        }

        /// Assignable from double
        template <typename T,
                  enable_if_t<classify_object<T>::value == object_category::double_constructible, detail::enabler> =
                      detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            double val;
            if(lexical_cast(input, val))
            {
                output = T{val};
                return true;
            }
            return from_stream(input, output);
        }

        /// Non-string convertible from an int
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::other && std::is_assignable<T &, int>::value,
                        detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            int val;
            if(integral_conversion(input, val))
            {
#ifdef _MSC_VER
#    pragma warning(push)
#    pragma warning(disable : 4800)
#endif
                // with Atomic<XX> this could produce a warning due to the conversion but if atomic gets here it is an
                // old style so will most likely still work
                output = val;
#ifdef _MSC_VER
#    pragma warning(pop)
#endif
                return true;
            }
            // LCOV_EXCL_START
            // This version of cast is only used for odd cases in an older compilers the fail over
            // from_stream is tested elsewhere an not relevant for coverage here
            return from_stream(input, output);
            // LCOV_EXCL_STOP
        }

        /// Non-string parsable by a stream
        template <
            typename T,
            enable_if_t<classify_object<T>::value == object_category::other && !std::is_assignable<T &, int>::value,
                        detail::enabler> = detail::dummy>
        bool lexical_cast(const std::string &input, T &output)
        {
            static_assert(
                is_istreamable<T>::value,
                "option object type must have a lexical cast overload or streaming input operator(>>) defined, if it "
                "is convertible from another type use the add_option<T, XC>(...) with XC being the known type");
            return from_stream(input, output);
        }

        /// Assign a value through lexical cast operations
        /// Strings can be empty so we need to do a little different
        template <typename AssignTo,
                  typename ConvertTo,
                  enable_if_t<std::is_same<AssignTo, ConvertTo>::value &&
                                  (classify_object<AssignTo>::value == object_category::string_assignable ||
                                   classify_object<AssignTo>::value == object_category::string_constructible),
                              detail::enabler> = detail::dummy>
        bool lexical_assign(const std::string &input, AssignTo &output)
        {
            return lexical_cast(input, output);
        }

        /// Assign a value through lexical cast operations
        template <
            typename AssignTo,
            typename ConvertTo,
            enable_if_t<std::is_same<AssignTo, ConvertTo>::value && std::is_assignable<AssignTo &, AssignTo>::value &&
                            classify_object<AssignTo>::value != object_category::string_assignable &&
                            classify_object<AssignTo>::value != object_category::string_constructible,
                        detail::enabler> = detail::dummy>
        bool lexical_assign(const std::string &input, AssignTo &output)
        {
            if(input.empty())
            {
                output = AssignTo{};
                return true;
            }

            return lexical_cast(input, output);
        }

        /// Assign a value through lexical cast operations
        template <
            typename AssignTo,
            typename ConvertTo,
            enable_if_t<std::is_same<AssignTo, ConvertTo>::value && !std::is_assignable<AssignTo &, AssignTo>::value &&
                            classify_object<AssignTo>::value == object_category::wrapper_value,
                        detail::enabler> = detail::dummy>
        bool lexical_assign(const std::string &input, AssignTo &output)
        {
            if(input.empty())
            {
                typename AssignTo::value_type emptyVal{};
                output = emptyVal;
                return true;
            }
            return lexical_cast(input, output);
        }

        /// Assign a value through lexical cast operations for int compatible values
        /// mainly for atomic operations on some compilers
        template <
            typename AssignTo,
            typename ConvertTo,
            enable_if_t<std::is_same<AssignTo, ConvertTo>::value && !std::is_assignable<AssignTo &, AssignTo>::value &&
                            classify_object<AssignTo>::value != object_category::wrapper_value &&
                            std::is_assignable<AssignTo &, int>::value,
                        detail::enabler> = detail::dummy>
        bool lexical_assign(const std::string &input, AssignTo &output)
        {
            if(input.empty())
            {
                output = 0;
                return true;
            }
            int val;
            if(lexical_cast(input, val))
            {
                output = val;
                return true;
            }
            return false;
        }

        /// Assign a value converted from a string in lexical cast to the output value directly
        template <
            typename AssignTo,
            typename ConvertTo,
            enable_if_t<!std::is_same<AssignTo, ConvertTo>::value && std::is_assignable<AssignTo &, ConvertTo &>::value,
                        detail::enabler> = detail::dummy>
        bool lexical_assign(const std::string &input, AssignTo &output)
        {
            ConvertTo val{};
            bool parse_result = (!input.empty()) ? lexical_cast<ConvertTo>(input, val) : true;
            if(parse_result)
            {
                output = val;
            }
            return parse_result;
        }

        /// Assign a value from a lexical cast through constructing a value and move assigning it
        template <typename AssignTo,
                  typename ConvertTo,
                  enable_if_t<!std::is_same<AssignTo, ConvertTo>::value &&
                                  !std::is_assignable<AssignTo &, ConvertTo &>::value &&
                                  std::is_move_assignable<AssignTo>::value,
                              detail::enabler> = detail::dummy>
        bool lexical_assign(const std::string &input, AssignTo &output)
        {
            ConvertTo val{};
            bool parse_result = input.empty() ? true : lexical_cast<ConvertTo>(input, val);
            if(parse_result)
            {
                output = AssignTo(val);  // use () form of constructor to allow some implicit conversions
            }
            return parse_result;
        }

        /// primary lexical conversion operation, 1 string to 1 type of some kind
        template <typename AssignTo,
                  typename ConvertTo,
                  enable_if_t<classify_object<ConvertTo>::value <= object_category::other &&
                                  classify_object<AssignTo>::value <= object_category::wrapper_value,
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std ::string> &strings, AssignTo &output)
        {
            return lexical_assign<AssignTo, ConvertTo>(strings[0], output);
        }

        /// Lexical conversion if there is only one element but the conversion type is for two, then call a two element
        /// constructor
        template <typename AssignTo,
                  typename ConvertTo,
                  enable_if_t<(type_count<AssignTo>::value <= 2) && expected_count<AssignTo>::value == 1 &&
                                  is_tuple_like<ConvertTo>::value && type_count_base<ConvertTo>::value == 2,
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std ::string> &strings, AssignTo &output)
        {
            // the remove const is to handle pair types coming from a container
            typename std::remove_const<typename std::tuple_element<0, ConvertTo>::type>::type v1;
            typename std::tuple_element<1, ConvertTo>::type v2;
            bool retval = lexical_assign<decltype(v1), decltype(v1)>(strings[0], v1);
            if(strings.size() > 1)
            {
                retval = retval && lexical_assign<decltype(v2), decltype(v2)>(strings[1], v2);
            }
            if(retval)
            {
                output = AssignTo{v1, v2};
            }
            return retval;
        }

        /// Lexical conversion of a container types of single elements
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_mutable_container<AssignTo>::value && is_mutable_container<ConvertTo>::value &&
                                  type_count<ConvertTo>::value == 1,
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std ::string> &strings, AssignTo &output)
        {
            output.erase(output.begin(), output.end());
            for(const auto &elem: strings)
            {
                typename AssignTo::value_type out;
                bool retval = lexical_assign<typename AssignTo::value_type, typename ConvertTo::value_type>(elem, out);
                if(!retval)
                {
                    return false;
                }
                output.insert(output.end(), std::move(out));
            }
            return (!output.empty());
        }

        /// Lexical conversion for complex types
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_complex<ConvertTo>::value, detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std::string> &strings, AssignTo &output)
        {
            if(strings.size() >= 2 && !strings[1].empty())
            {
                using XC2 = typename wrapped_type<ConvertTo, double>::type;
                XC2 x{0.0}, y{0.0};
                auto str1 = strings[1];
                if(str1.back() == 'i' || str1.back() == 'j')
                {
                    str1.pop_back();
                }
                auto worked = detail::lexical_cast(strings[0], x) && detail::lexical_cast(str1, y);
                if(worked)
                {
                    output = ConvertTo{x, y};
                }
                return worked;
            }
            else
            {
                return lexical_assign<AssignTo, ConvertTo>(strings[0], output);
            }
        }

        /// Conversion to a vector type using a particular single type as the conversion type
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_mutable_container<AssignTo>::value && (expected_count<ConvertTo>::value == 1) &&
                                  (type_count<ConvertTo>::value == 1),
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std ::string> &strings, AssignTo &output)
        {
            bool retval = true;
            output.clear();
            output.reserve(strings.size());
            for(const auto &elem: strings)
            {
                output.emplace_back();
                retval = retval && lexical_assign<typename AssignTo::value_type, ConvertTo>(elem, output.back());
            }
            return (!output.empty()) && retval;
        }

        // forward declaration

        /// Lexical conversion of a container types with conversion type of two elements
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_mutable_container<AssignTo>::value && is_mutable_container<ConvertTo>::value &&
                                  type_count_base<ConvertTo>::value == 2,
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(std::vector<std::string> strings, AssignTo &output);

        /// Lexical conversion of a vector types with type_size >2 forward declaration
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_mutable_container<AssignTo>::value && is_mutable_container<ConvertTo>::value &&
                                  type_count_base<ConvertTo>::value != 2 &&
                                  ((type_count<ConvertTo>::value > 2) ||
                                   (type_count<ConvertTo>::value > type_count_base<ConvertTo>::value)),
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std::string> &strings, AssignTo &output);

        /// Conversion for tuples
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_tuple_like<AssignTo>::value && is_tuple_like<ConvertTo>::value &&
                                  (type_count_base<ConvertTo>::value != type_count<ConvertTo>::value ||
                                   type_count<ConvertTo>::value > 2),
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std::string> &strings, AssignTo &output);  // forward declaration

        /// Conversion for operations where the assigned type is some class but the conversion is a mutable container or
        /// large tuple
        template <typename AssignTo,
                  typename ConvertTo,
                  enable_if_t<!is_tuple_like<AssignTo>::value && !is_mutable_container<AssignTo>::value &&
                                  classify_object<ConvertTo>::value != object_category::wrapper_value &&
                                  (is_mutable_container<ConvertTo>::value || type_count<ConvertTo>::value > 2),
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std ::string> &strings, AssignTo &output)
        {
            if(strings.size() > 1 || (!strings.empty() && !(strings.front().empty())))
            {
                ConvertTo val;
                auto retval = lexical_conversion<ConvertTo, ConvertTo>(strings, val);
                output      = AssignTo{val};
                return retval;
            }
            output = AssignTo{};
            return true;
        }

        /// function template for converting tuples if the static Index is greater than the tuple size
        template <class AssignTo, class ConvertTo, std::size_t I>
        inline typename std::enable_if<(I >= type_count_base<AssignTo>::value), bool>::type tuple_conversion(
            const std::vector<std::string> &,
            AssignTo &)
        {
            return true;
        }

        /// Conversion of a tuple element where the type size ==1 and not a mutable container
        template <class AssignTo, class ConvertTo>
        inline typename std::enable_if<!is_mutable_container<ConvertTo>::value && type_count<ConvertTo>::value == 1,
                                       bool>::type
        tuple_type_conversion(std::vector<std::string> &strings, AssignTo &output)
        {
            auto retval = lexical_assign<AssignTo, ConvertTo>(strings[0], output);
            strings.erase(strings.begin());
            return retval;
        }

        /// Conversion of a tuple element where the type size !=1 but the size is fixed and not a mutable container
        template <class AssignTo, class ConvertTo>
        inline typename std::enable_if<!is_mutable_container<ConvertTo>::value && (type_count<ConvertTo>::value > 1) &&
                                           type_count<ConvertTo>::value == type_count_min<ConvertTo>::value,
                                       bool>::type
        tuple_type_conversion(std::vector<std::string> &strings, AssignTo &output)
        {
            auto retval = lexical_conversion<AssignTo, ConvertTo>(strings, output);
            strings.erase(strings.begin(), strings.begin() + type_count<ConvertTo>::value);
            return retval;
        }

        /// Conversion of a tuple element where the type is a mutable container or a type with different min and max
        /// type sizes
        template <class AssignTo, class ConvertTo>
        inline typename std::enable_if<is_mutable_container<ConvertTo>::value ||
                                           type_count<ConvertTo>::value != type_count_min<ConvertTo>::value,
                                       bool>::type
        tuple_type_conversion(std::vector<std::string> &strings, AssignTo &output)
        {
            std::size_t index{subtype_count_min<ConvertTo>::value};
            const std::size_t mx_count{subtype_count<ConvertTo>::value};
            const std::size_t mx{(std::max)(mx_count, strings.size())};

            while(index < mx)
            {
                if(is_separator(strings[index]))
                {
                    break;
                }
                ++index;
            }
            bool retval = lexical_conversion<AssignTo, ConvertTo>(
                std::vector<std::string>(strings.begin(), strings.begin() + static_cast<std::ptrdiff_t>(index)),
                output);
            strings.erase(strings.begin(), strings.begin() + static_cast<std::ptrdiff_t>(index) + 1);
            return retval;
        }

        /// Tuple conversion operation
        template <class AssignTo, class ConvertTo, std::size_t I>
        inline typename std::enable_if<(I < type_count_base<AssignTo>::value), bool>::type tuple_conversion(
            std::vector<std::string> strings,
            AssignTo &output)
        {
            bool retval            = true;
            using ConvertToElement = typename std::conditional<is_tuple_like<ConvertTo>::value,
                                                               typename std::tuple_element<I, ConvertTo>::type,
                                                               ConvertTo>::type;
            if(!strings.empty())
            {
                retval =
                    retval && tuple_type_conversion<typename std::tuple_element<I, AssignTo>::type, ConvertToElement>(
                                  strings,
                                  std::get<I>(output));
            }
            retval = retval && tuple_conversion<AssignTo, ConvertTo, I + 1>(std::move(strings), output);
            return retval;
        }

        /// Lexical conversion of a container types with tuple elements of size 2
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_mutable_container<AssignTo>::value && is_mutable_container<ConvertTo>::value &&
                                  type_count_base<ConvertTo>::value == 2,
                              detail::enabler>>
        bool lexical_conversion(std::vector<std::string> strings, AssignTo &output)
        {
            output.clear();
            while(!strings.empty())
            {
                typename std::remove_const<typename std::tuple_element<0, typename ConvertTo::value_type>::type>::type
                    v1;
                typename std::tuple_element<1, typename ConvertTo::value_type>::type v2;
                bool retval = tuple_type_conversion<decltype(v1), decltype(v1)>(strings, v1);
                if(!strings.empty())
                {
                    retval = retval && tuple_type_conversion<decltype(v2), decltype(v2)>(strings, v2);
                }
                if(retval)
                {
                    output.insert(output.end(), typename AssignTo::value_type{v1, v2});
                }
                else
                {
                    return false;
                }
            }
            return (!output.empty());
        }

        /// lexical conversion of tuples with type count>2 or tuples of types of some element with a type size>=2
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_tuple_like<AssignTo>::value && is_tuple_like<ConvertTo>::value &&
                                  (type_count_base<ConvertTo>::value != type_count<ConvertTo>::value ||
                                   type_count<ConvertTo>::value > 2),
                              detail::enabler>>
        bool lexical_conversion(const std::vector<std ::string> &strings, AssignTo &output)
        {
            static_assert(!is_tuple_like<ConvertTo>::value ||
                              type_count_base<AssignTo>::value == type_count_base<ConvertTo>::value,
                          "if the conversion type is defined as a tuple it must be the same size as the type you are "
                          "converting to");
            return tuple_conversion<AssignTo, ConvertTo, 0>(strings, output);
        }

        /// Lexical conversion of a vector types for everything but tuples of two elements and types of size 1
        template <class AssignTo,
                  class ConvertTo,
                  enable_if_t<is_mutable_container<AssignTo>::value && is_mutable_container<ConvertTo>::value &&
                                  type_count_base<ConvertTo>::value != 2 &&
                                  ((type_count<ConvertTo>::value > 2) ||
                                   (type_count<ConvertTo>::value > type_count_base<ConvertTo>::value)),
                              detail::enabler>>
        bool lexical_conversion(const std::vector<std ::string> &strings, AssignTo &output)
        {
            bool retval = true;
            output.clear();
            std::vector<std::string> temp;
            std::size_t ii{0};
            std::size_t icount{0};
            std::size_t xcm{type_count<ConvertTo>::value};
            auto ii_max = strings.size();
            while(ii < ii_max)
            {
                temp.push_back(strings[ii]);
                ++ii;
                ++icount;
                if(icount == xcm || is_separator(temp.back()) || ii == ii_max)
                {
                    if(static_cast<int>(xcm) > type_count_min<ConvertTo>::value && is_separator(temp.back()))
                    {
                        temp.pop_back();
                    }
                    typename AssignTo::value_type temp_out;
                    retval =
                        retval &&
                        lexical_conversion<typename AssignTo::value_type, typename ConvertTo::value_type>(temp,
                                                                                                          temp_out);
                    temp.clear();
                    if(!retval)
                    {
                        return false;
                    }
                    output.insert(output.end(), std::move(temp_out));
                    icount = 0;
                }
            }
            return retval;
        }

        /// conversion for wrapper types
        template <typename AssignTo,
                  class ConvertTo,
                  enable_if_t<classify_object<ConvertTo>::value == object_category::wrapper_value &&
                                  std::is_assignable<ConvertTo &, ConvertTo>::value,
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std::string> &strings, AssignTo &output)
        {
            if(strings.empty() || strings.front().empty())
            {
                output = ConvertTo{};
                return true;
            }
            typename ConvertTo::value_type val;
            if(lexical_conversion<typename ConvertTo::value_type, typename ConvertTo::value_type>(strings, val))
            {
                output = ConvertTo{val};
                return true;
            }
            return false;
        }

        /// conversion for wrapper types
        template <typename AssignTo,
                  class ConvertTo,
                  enable_if_t<classify_object<ConvertTo>::value == object_category::wrapper_value &&
                                  !std::is_assignable<AssignTo &, ConvertTo>::value,
                              detail::enabler> = detail::dummy>
        bool lexical_conversion(const std::vector<std::string> &strings, AssignTo &output)
        {
            using ConvertType = typename ConvertTo::value_type;
            if(strings.empty() || strings.front().empty())
            {
                output = ConvertType{};
                return true;
            }
            ConvertType val;
            if(lexical_conversion<typename ConvertTo::value_type, typename ConvertTo::value_type>(strings, val))
            {
                output = val;
                return true;
            }
            return false;
        }

        /// Sum a vector of flag representations
        /// The flag vector produces a series of strings in a vector,  simple true is represented by a "1",  simple
        /// false is by
        /// "-1" an if numbers are passed by some fashion they are captured as well so the function just checks for the
        /// most common true and false strings then uses stoll to convert the rest for summing
        template <typename T, enable_if_t<std::is_unsigned<T>::value, detail::enabler> = detail::dummy>
        void sum_flag_vector(const std::vector<std::string> &flags, T &output)
        {
            std::int64_t count{0};
            for(auto &flag: flags)
            {
                count += detail::to_flag_value(flag);
            }
            output = (count > 0) ? static_cast<T>(count) : T{0};
        }

        /// Sum a vector of flag representations
        /// The flag vector produces a series of strings in a vector,  simple true is represented by a "1",  simple
        /// false is by
        /// "-1" an if numbers are passed by some fashion they are captured as well so the function just checks for the
        /// most common true and false strings then uses stoll to convert the rest for summing
        template <typename T, enable_if_t<std::is_signed<T>::value, detail::enabler> = detail::dummy>
        void sum_flag_vector(const std::vector<std::string> &flags, T &output)
        {
            std::int64_t count{0};
            for(auto &flag: flags)
            {
                count += detail::to_flag_value(flag);
            }
            output = static_cast<T>(count);
        }

#ifdef _MSC_VER
#    pragma warning(push)
#    pragma warning(disable : 4800)
#endif
        // with Atomic<XX> this could produce a warning due to the conversion but if atomic gets here it is an old style
        // so will most likely still work

        /// Sum a vector of flag representations
        /// The flag vector produces a series of strings in a vector,  simple true is represented by a "1",  simple
        /// false is by
        /// "-1" an if numbers are passed by some fashion they are captured as well so the function just checks for the
        /// most common true and false strings then uses stoll to convert the rest for summing
        template <
            typename T,
            enable_if_t<!std::is_signed<T>::value && !std::is_unsigned<T>::value, detail::enabler> = detail::dummy>
        void sum_flag_vector(const std::vector<std::string> &flags, T &output)
        {
            std::int64_t count{0};
            for(auto &flag: flags)
            {
                count += detail::to_flag_value(flag);
            }
            std::string out = detail::to_string(count);
            lexical_cast(out, output);
        }

#ifdef _MSC_VER
#    pragma warning(pop)
#endif

    }  // namespace detail

    namespace detail
    {

        // Returns false if not a short option. Otherwise, sets opt name and rest and returns true
        inline bool split_short(const std::string &current, std::string &name, std::string &rest)
        {
            if(current.size() > 1 && current[0] == '-' && valid_first_char(current[1]))
            {
                name = current.substr(1, 1);
                rest = current.substr(2);
                return true;
            }
            return false;
        }

        // Returns false if not a long option. Otherwise, sets opt name and other side of = and returns true
        inline bool split_long(const std::string &current, std::string &name, std::string &value)
        {
            if(current.size() > 2 && current.substr(0, 2) == "--" && valid_first_char(current[2]))
            {
                auto loc = current.find_first_of('=');
                if(loc != std::string::npos)
                {
                    name  = current.substr(2, loc - 2);
                    value = current.substr(loc + 1);
                }
                else
                {
                    name  = current.substr(2);
                    value = "";
                }
                return true;
            }
            return false;
        }

        // Returns false if not a windows style option. Otherwise, sets opt name and value and returns true
        inline bool split_windows_style(const std::string &current, std::string &name, std::string &value)
        {
            if(current.size() > 1 && current[0] == '/' && valid_first_char(current[1]))
            {
                auto loc = current.find_first_of(':');
                if(loc != std::string::npos)
                {
                    name  = current.substr(1, loc - 1);
                    value = current.substr(loc + 1);
                }
                else
                {
                    name  = current.substr(1);
                    value = "";
                }
                return true;
            }
            return false;
        }

        // Splits a string into multiple long and short names
        inline std::vector<std::string> split_names(std::string current)
        {
            std::vector<std::string> output;
            std::size_t val;
            while((val = current.find(",")) != std::string::npos)
            {
                output.push_back(trim_copy(current.substr(0, val)));
                current = current.substr(val + 1);
            }
            output.push_back(trim_copy(current));
            return output;
        }

        /// extract default flag values either {def} or starting with a !
        inline std::vector<std::pair<std::string, std::string>> get_default_flag_values(const std::string &str)
        {
            std::vector<std::string> flags = split_names(str);
            flags.erase(std::remove_if(
                            flags.begin(),
                            flags.end(),
                            [](const std::string &name)
                            {
                                return ((name.empty()) ||
                                        (!(((name.find_first_of('{') != std::string::npos) && (name.back() == '}')) ||
                                           (name[0] == '!'))));
                            }),
                        flags.end());
            std::vector<std::pair<std::string, std::string>> output;
            output.reserve(flags.size());
            for(auto &flag: flags)
            {
                auto def_start     = flag.find_first_of('{');
                std::string defval = "false";
                if((def_start != std::string::npos) && (flag.back() == '}'))
                {
                    defval = flag.substr(def_start + 1);
                    defval.pop_back();
                    flag.erase(def_start, std::string::npos);
                }
                flag.erase(0, flag.find_first_not_of("-!"));
                output.emplace_back(flag, defval);
            }
            return output;
        }

        /// Get a vector of short names, one of long names, and a single name
        inline std::tuple<std::vector<std::string>, std::vector<std::string>, std::string> get_names(
            const std::vector<std::string> &input)
        {
            std::vector<std::string> short_names;
            std::vector<std::string> long_names;
            std::string pos_name;

            for(std::string name: input)
            {
                if(name.length() == 0)
                {
                    continue;
                }
                if(name.length() > 1 && name[0] == '-' && name[1] != '-')
                {
                    if(name.length() == 2 && valid_first_char(name[1]))
                        short_names.emplace_back(1, name[1]);
                    else
                        throw BadNameString::OneCharName(name);
                }
                else if(name.length() > 2 && name.substr(0, 2) == "--")
                {
                    name = name.substr(2);
                    if(valid_name_string(name))
                        long_names.push_back(name);
                    else
                        throw BadNameString::BadLongName(name);
                }
                else if(name == "-" || name == "--")
                {
                    throw BadNameString::DashesOnly(name);
                }
                else
                {
                    if(pos_name.length() > 0)
                        throw BadNameString::MultiPositionalNames(name);
                    pos_name = name;
                }
            }

            return std::tuple<std::vector<std::string>, std::vector<std::string>, std::string>(short_names,
                                                                                               long_names,
                                                                                               pos_name);
        }

    }  // namespace detail

    class App;

    /// Holds values to load into Options
    struct ConfigItem
    {
        /// This is the list of parents
        std::vector<std::string> parents{};

        /// This is the name
        std::string name{};

        /// Listing of inputs
        std::vector<std::string> inputs{};

        /// The list of parents and name joined by "."
        std::string fullname() const
        {
            std::vector<std::string> tmp = parents;
            tmp.emplace_back(name);
            return detail::join(tmp, ".");
        }
    };

    /// This class provides a converter for configuration files.
    class Config
    {
       protected:
        std::vector<ConfigItem> items{};

       public:
        /// Convert an app into a configuration
        virtual std::string to_config(const App *, bool, bool, std::string) const = 0;

        /// Convert a configuration into an app
        virtual std::vector<ConfigItem> from_config(std::istream &) const = 0;

        /// Get a flag value
        virtual std::string to_flag(const ConfigItem &item) const
        {
            if(item.inputs.size() == 1)
            {
                return item.inputs.at(0);
            }
            throw ConversionError::TooManyInputsFlag(item.fullname());
        }

        /// Parse a config file, throw an error (ParseError:ConfigParseError or FileError) on failure
        std::vector<ConfigItem> from_file(const std::string &name)
        {
            std::ifstream input{name};
            if(!input.good())
                throw FileError::Missing(name);

            return from_config(input);
        }

        /// Virtual destructor
        virtual ~Config() = default;
    };

    /// This converter works with INI/TOML files; to write INI files use ConfigINI
    class ConfigBase : public Config
    {
       protected:
        /// the character used for comments
        char commentChar = '#';
        /// the character used to start an array '\0' is a default to not use
        char arrayStart = '[';
        /// the character used to end an array '\0' is a default to not use
        char arrayEnd = ']';
        /// the character used to separate elements in an array
        char arraySeparator = ',';
        /// the character used separate the name from the value
        char valueDelimiter = '=';
        /// the character to use around strings
        char stringQuote = '"';
        /// the character to use around single characters
        char characterQuote = '\'';

       public:
        std::string to_config(const App * /*app*/,
                              bool default_also,
                              bool write_description,
                              std::string prefix) const override;

        std::vector<ConfigItem> from_config(std::istream &input) const override;
        /// Specify the configuration for comment characters
        ConfigBase *comment(char cchar)
        {
            commentChar = cchar;
            return this;
        }
        /// Specify the start and end characters for an array
        ConfigBase *arrayBounds(char aStart, char aEnd)
        {
            arrayStart = aStart;
            arrayEnd   = aEnd;
            return this;
        }
        /// Specify the delimiter character for an array
        ConfigBase *arrayDelimiter(char aSep)
        {
            arraySeparator = aSep;
            return this;
        }
        /// Specify the delimiter between a name and value
        ConfigBase *valueSeparator(char vSep)
        {
            valueDelimiter = vSep;
            return this;
        }
        /// Specify the quote characters used around strings and characters
        ConfigBase *quoteCharacter(char qString, char qChar)
        {
            stringQuote    = qString;
            characterQuote = qChar;
            return this;
        }
    };

    /// the default Config is the TOML file format
    using ConfigTOML = ConfigBase;

    /// ConfigINI generates a "standard" INI compliant output
    class ConfigINI : public ConfigTOML
    {
       public:
        ConfigINI()
        {
            commentChar    = ';';
            arrayStart     = '\0';
            arrayEnd       = '\0';
            arraySeparator = ' ';
            valueDelimiter = '=';
        }
    };

    class Option;

    /// @defgroup validator_group Validators

    /// @brief Some validators that are provided
    ///
    /// These are simple `std::string(const std::string&)` validators that are useful. They return
    /// a string if the validation fails. A custom struct is provided, as well, with the same user
    /// semantics, but with the ability to provide a new type name.
    /// @{

    ///
    class Validator
    {
       protected:
        /// This is the description function, if empty the description_ will be used
        std::function<std::string()> desc_function_{[]()
                                                    {
                                                        return std::string{};
                                                    }};

        /// This is the base function that is to be called.
        /// Returns a string error message if validation fails.
        std::function<std::string(std::string &)> func_{[](std::string &)
                                                        {
                                                            return std::string{};
                                                        }};
        /// The name for search purposes of the Validator
        std::string name_{};
        /// A Validator will only apply to an indexed value (-1 is all elements)
        int application_index_ = -1;
        /// Enable for Validator to allow it to be disabled if need be
        bool active_{true};
        /// specify that a validator should not modify the input
        bool non_modifying_{false};

       public:
        Validator() = default;
        /// Construct a Validator with just the description string
        explicit Validator(std::string validator_desc)
            : desc_function_(
                  [validator_desc]()
                  {
                      return validator_desc;
                  })
        {}
        /// Construct Validator from basic information
        Validator(std::function<std::string(std::string &)> op,
                  std::string validator_desc,
                  std::string validator_name = "")
            : desc_function_(
                  [validator_desc]()
                  {
                      return validator_desc;
                  })
            , func_(std::move(op))
            , name_(std::move(validator_name))
        {}
        /// Set the Validator operation function
        Validator &operation(std::function<std::string(std::string &)> op)
        {
            func_ = std::move(op);
            return *this;
        }
        /// This is the required operator for a Validator - provided to help
        /// users (CLI11 uses the member `func` directly)
        std::string operator()(std::string &str) const
        {
            std::string retstring;
            if(active_)
            {
                if(non_modifying_)
                {
                    std::string value = str;
                    retstring         = func_(value);
                }
                else
                {
                    retstring = func_(str);
                }
            }
            return retstring;
        }

        /// This is the required operator for a Validator - provided to help
        /// users (CLI11 uses the member `func` directly)
        std::string operator()(const std::string &str) const
        {
            std::string value = str;
            return (active_) ? func_(value) : std::string{};
        }

        /// Specify the type string
        Validator &description(std::string validator_desc)
        {
            desc_function_ = [validator_desc]()
            {
                return validator_desc;
            };
            return *this;
        }
        /// Specify the type string
        Validator description(std::string validator_desc) const
        {
            Validator newval(*this);
            newval.desc_function_ = [validator_desc]()
            {
                return validator_desc;
            };
            return newval;
        }
        /// Generate type description information for the Validator
        std::string get_description() const
        {
            if(active_)
            {
                return desc_function_();
            }
            return std::string{};
        }
        /// Specify the type string
        Validator &name(std::string validator_name)
        {
            name_ = std::move(validator_name);
            return *this;
        }
        /// Specify the type string
        Validator name(std::string validator_name) const
        {
            Validator newval(*this);
            newval.name_ = std::move(validator_name);
            return newval;
        }
        /// Get the name of the Validator
        const std::string &get_name() const
        {
            return name_;
        }
        /// Specify whether the Validator is active or not
        Validator &active(bool active_val = true)
        {
            active_ = active_val;
            return *this;
        }
        /// Specify whether the Validator is active or not
        Validator active(bool active_val = true) const
        {
            Validator newval(*this);
            newval.active_ = active_val;
            return newval;
        }

        /// Specify whether the Validator can be modifying or not
        Validator &non_modifying(bool no_modify = true)
        {
            non_modifying_ = no_modify;
            return *this;
        }
        /// Specify the application index of a validator
        Validator &application_index(int app_index)
        {
            application_index_ = app_index;
            return *this;
        }
        /// Specify the application index of a validator
        Validator application_index(int app_index) const
        {
            Validator newval(*this);
            newval.application_index_ = app_index;
            return newval;
        }
        /// Get the current value of the application index
        int get_application_index() const
        {
            return application_index_;
        }
        /// Get a boolean if the validator is active
        bool get_active() const
        {
            return active_;
        }

        /// Get a boolean if the validator is allowed to modify the input returns true if it can modify the input
        bool get_modifying() const
        {
            return !non_modifying_;
        }

        /// Combining validators is a new validator. Type comes from left validator if function, otherwise only set if
        /// the same.
        Validator operator&(const Validator &other) const
        {
            Validator newval;

            newval._merge_description(*this, other, " AND ");

            // Give references (will make a copy in lambda function)
            const std::function<std::string(std::string & filename)> &f1 = func_;
            const std::function<std::string(std::string & filename)> &f2 = other.func_;

            newval.func_ = [f1, f2](std::string &input)
            {
                std::string s1 = f1(input);
                std::string s2 = f2(input);
                if(!s1.empty() && !s2.empty())
                    return std::string("(") + s1 + ") AND (" + s2 + ")";
                else
                    return s1 + s2;
            };

            newval.active_            = (active_ & other.active_);
            newval.application_index_ = application_index_;
            return newval;
        }

        /// Combining validators is a new validator. Type comes from left validator if function, otherwise only set if
        /// the same.
        Validator operator|(const Validator &other) const
        {
            Validator newval;

            newval._merge_description(*this, other, " OR ");

            // Give references (will make a copy in lambda function)
            const std::function<std::string(std::string &)> &f1 = func_;
            const std::function<std::string(std::string &)> &f2 = other.func_;

            newval.func_ = [f1, f2](std::string &input)
            {
                std::string s1 = f1(input);
                std::string s2 = f2(input);
                if(s1.empty() || s2.empty())
                    return std::string();

                return std::string("(") + s1 + ") OR (" + s2 + ")";
            };
            newval.active_            = (active_ & other.active_);
            newval.application_index_ = application_index_;
            return newval;
        }

        /// Create a validator that fails when a given validator succeeds
        Validator operator!() const
        {
            Validator newval;
            const std::function<std::string()> &dfunc1 = desc_function_;
            newval.desc_function_                      = [dfunc1]()
            {
                auto str = dfunc1();
                return (!str.empty()) ? std::string("NOT ") + str : std::string{};
            };
            // Give references (will make a copy in lambda function)
            const std::function<std::string(std::string & res)> &f1 = func_;

            newval.func_ = [f1, dfunc1](std::string &test) -> std::string
            {
                std::string s1 = f1(test);
                if(s1.empty())
                {
                    return std::string("check ") + dfunc1() + " succeeded improperly";
                }
                return std::string{};
            };
            newval.active_            = active_;
            newval.application_index_ = application_index_;
            return newval;
        }

       private:
        void _merge_description(const Validator &val1, const Validator &val2, const std::string &merger)
        {
            const std::function<std::string()> &dfunc1 = val1.desc_function_;
            const std::function<std::string()> &dfunc2 = val2.desc_function_;

            desc_function_ = [=]()
            {
                std::string f1 = dfunc1();
                std::string f2 = dfunc2();
                if((f1.empty()) || (f2.empty()))
                {
                    return f1 + f2;
                }
                return std::string(1, '(') + f1 + ')' + merger + '(' + f2 + ')';
            };
        }
    };  // namespace CLI

    /// Class wrapping some of the accessors of Validator
    class CustomValidator : public Validator
    {
       public:
    };
    // The implementation of the built in validators is using the Validator class;
    // the user is only expected to use the const (static) versions (since there's no setup).
    // Therefore, this is in detail.
    namespace detail
    {

        /// CLI enumeration of different file types
        enum class path_type
        {
            nonexistent,
            file,
            directory
        };

#if defined CLI11_HAS_FILESYSTEM && CLI11_HAS_FILESYSTEM > 0
        /// get the type of the path from a file name
        inline path_type check_path(const char *file) noexcept
        {
            std::error_code ec;
            auto stat = std::filesystem::status(file, ec);
            if(ec)
            {
                return path_type::nonexistent;
            }
            switch(stat.type())
            {
                case std::filesystem::file_type::none:
                case std::filesystem::file_type::not_found:
                    return path_type::nonexistent;
                case std::filesystem::file_type::directory:
                    return path_type::directory;
                case std::filesystem::file_type::symlink:
                case std::filesystem::file_type::block:
                case std::filesystem::file_type::character:
                case std::filesystem::file_type::fifo:
                case std::filesystem::file_type::socket:
                case std::filesystem::file_type::regular:
                case std::filesystem::file_type::unknown:
                default:
                    return path_type::file;
            }
        }
#else
        /// get the type of the path from a file name
        inline path_type check_path(const char *file) noexcept
        {
#    if defined(_MSC_VER)
            struct __stat64 buffer;
            if(_stat64(file, &buffer) == 0)
            {
                return ((buffer.st_mode & S_IFDIR) != 0) ? path_type::directory : path_type::file;
            }
#    else
            struct stat buffer;
            if(stat(file, &buffer) == 0)
            {
                return ((buffer.st_mode & S_IFDIR) != 0) ? path_type::directory : path_type::file;
            }
#    endif
            return path_type::nonexistent;
        }
#endif
        /// Check for an existing file (returns error message if check fails)
        class ExistingFileValidator : public Validator
        {
           public:
            ExistingFileValidator()
                : Validator("FILE")
            {
                func_ = [](std::string &filename)
                {
                    auto path_result = check_path(filename.c_str());
                    if(path_result == path_type::nonexistent)
                    {
                        return "File does not exist: " + filename;
                    }
                    if(path_result == path_type::directory)
                    {
                        return "File is actually a directory: " + filename;
                    }
                    return std::string();
                };
            }
        };

        /// Check for an existing directory (returns error message if check fails)
        class ExistingDirectoryValidator : public Validator
        {
           public:
            ExistingDirectoryValidator()
                : Validator("DIR")
            {
                func_ = [](std::string &filename)
                {
                    auto path_result = check_path(filename.c_str());
                    if(path_result == path_type::nonexistent)
                    {
                        return "Directory does not exist: " + filename;
                    }
                    if(path_result == path_type::file)
                    {
                        return "Directory is actually a file: " + filename;
                    }
                    return std::string();
                };
            }
        };

        /// Check for an existing path
        class ExistingPathValidator : public Validator
        {
           public:
            ExistingPathValidator()
                : Validator("PATH(existing)")
            {
                func_ = [](std::string &filename)
                {
                    auto path_result = check_path(filename.c_str());
                    if(path_result == path_type::nonexistent)
                    {
                        return "Path does not exist: " + filename;
                    }
                    return std::string();
                };
            }
        };

        /// Check for an non-existing path
        class NonexistentPathValidator : public Validator
        {
           public:
            NonexistentPathValidator()
                : Validator("PATH(non-existing)")
            {
                func_ = [](std::string &filename)
                {
                    auto path_result = check_path(filename.c_str());
                    if(path_result != path_type::nonexistent)
                    {
                        return "Path already exists: " + filename;
                    }
                    return std::string();
                };
            }
        };

        /// Validate the given string is a legal ipv4 address
        class IPV4Validator : public Validator
        {
           public:
            IPV4Validator()
                : Validator("IPV4")
            {
                func_ = [](std::string &ip_addr)
                {
                    auto result = CLI::detail::split(ip_addr, '.');
                    if(result.size() != 4)
                    {
                        return std::string("Invalid IPV4 address must have four parts (") + ip_addr + ')';
                    }
                    int num;
                    for(const auto &var: result)
                    {
                        bool retval = detail::lexical_cast(var, num);
                        if(!retval)
                        {
                            return std::string("Failed parsing number (") + var + ')';
                        }
                        if(num < 0 || num > 255)
                        {
                            return std::string("Each IP number must be between 0 and 255 ") + var;
                        }
                    }
                    return std::string();
                };
            }
        };

    }  // namespace detail

    // Static is not needed here, because global const implies static.

    /// Check for existing file (returns error message if check fails)
    const detail::ExistingFileValidator ExistingFile;

    /// Check for an existing directory (returns error message if check fails)
    const detail::ExistingDirectoryValidator ExistingDirectory;

    /// Check for an existing path
    const detail::ExistingPathValidator ExistingPath;

    /// Check for an non-existing path
    const detail::NonexistentPathValidator NonexistentPath;

    /// Check for an IP4 address
    const detail::IPV4Validator ValidIPV4;

    /// Validate the input as a particular type
    template <typename DesiredType>
    class TypeValidator : public Validator
    {
       public:
        explicit TypeValidator(const std::string &validator_name)
            : Validator(validator_name)
        {
            func_ = [](std::string &input_string)
            {
                auto val = DesiredType();
                if(!detail::lexical_cast(input_string, val))
                {
                    return std::string("Failed parsing ") + input_string + " as a " + detail::type_name<DesiredType>();
                }
                return std::string();
            };
        }
        TypeValidator()
            : TypeValidator(detail::type_name<DesiredType>())
        {}
    };

    /// Check for a number
    const TypeValidator<double> Number("NUMBER");

    /// Produce a range (factory). Min and max are inclusive.
    class Range : public Validator
    {
       public:
        /// This produces a range with min and max inclusive.
        ///
        /// Note that the constructor is templated, but the struct is not, so C++17 is not
        /// needed to provide nice syntax for Range(a,b).
        template <typename T>
        Range(T min, T max, const std::string &validator_name = std::string{})
            : Validator(validator_name)
        {
            if(validator_name.empty())
            {
                std::stringstream out;
                out << detail::type_name<T>() << " in [" << min << " - " << max << "]";
                description(out.str());
            }

            func_ = [min, max](std::string &input)
            {
                T val;
                bool converted = detail::lexical_cast(input, val);
                if((!converted) || (val < min || val > max))
                    return std::string("Value ") + input + " not in range " + std::to_string(min) + " to " +
                           std::to_string(max);

                return std::string();
            };
        }

        /// Range of one value is 0 to value
        template <typename T>
        explicit Range(T max, const std::string &validator_name = std::string{})
            : Range(static_cast<T>(0), max, validator_name)
        {}
    };

    /// Check for a non negative number
    const Range NonNegativeNumber(std::numeric_limits<double>::max(), "NONNEGATIVE");

    /// Check for a positive valued number (val>0.0), min() her is the smallest positive number
    const Range PositiveNumber(std::numeric_limits<double>::min(), std::numeric_limits<double>::max(), "POSITIVE");

    /// Produce a bounded range (factory). Min and max are inclusive.
    class Bound : public Validator
    {
       public:
        /// This bounds a value with min and max inclusive.
        ///
        /// Note that the constructor is templated, but the struct is not, so C++17 is not
        /// needed to provide nice syntax for Range(a,b).
        template <typename T>
        Bound(T min, T max)
        {
            std::stringstream out;
            out << detail::type_name<T>() << " bounded to [" << min << " - " << max << "]";
            description(out.str());

            func_ = [min, max](std::string &input)
            {
                T val;
                bool converted = detail::lexical_cast(input, val);
                if(!converted)
                {
                    return std::string("Value ") + input + " could not be converted";
                }
                if(val < min)
                    input = detail::to_string(min);
                else if(val > max)
                    input = detail::to_string(max);

                return std::string{};
            };
        }

        /// Range of one value is 0 to value
        template <typename T>
        explicit Bound(T max)
            : Bound(static_cast<T>(0), max)
        {}
    };

    namespace detail
    {
        template <typename T,
                  enable_if_t<is_copyable_ptr<typename std::remove_reference<T>::type>::value, detail::enabler> =
                      detail::dummy>
        auto smart_deref(T value) -> decltype(*value)
        {
            return *value;
        }

        template <typename T,
                  enable_if_t<!is_copyable_ptr<typename std::remove_reference<T>::type>::value, detail::enabler> =
                      detail::dummy>
        typename std::remove_reference<T>::type &smart_deref(T &value)
        {
            return value;
        }
        /// Generate a string representation of a set
        template <typename T>
        std::string generate_set(const T &set)
        {
            using element_t = typename detail::element_type<T>::type;
            using iteration_type_t =
                typename detail::pair_adaptor<element_t>::value_type;  // the type of the object pair
            std::string out(1, '{');
            out.append(detail::join(
                detail::smart_deref(set),
                [](const iteration_type_t &v)
                {
                    return detail::pair_adaptor<element_t>::first(v);
                },
                ","));
            out.push_back('}');
            return out;
        }

        /// Generate a string representation of a map
        template <typename T>
        std::string generate_map(const T &map, bool key_only = false)
        {
            using element_t = typename detail::element_type<T>::type;
            using iteration_type_t =
                typename detail::pair_adaptor<element_t>::value_type;  // the type of the object pair
            std::string out(1, '{');
            out.append(detail::join(
                detail::smart_deref(map),
                [key_only](const iteration_type_t &v)
                {
                    std::string res{detail::to_string(detail::pair_adaptor<element_t>::first(v))};

                    if(!key_only)
                    {
                        res.append("->");
                        res += detail::to_string(detail::pair_adaptor<element_t>::second(v));
                    }
                    return res;
                },
                ","));
            out.push_back('}');
            return out;
        }

        template <typename C, typename V>
        struct has_find
        {
            template <typename CC, typename VV>
            static auto test(int) -> decltype(std::declval<CC>().find(std::declval<VV>()), std::true_type());
            template <typename, typename>
            static auto test(...) -> decltype(std::false_type());

            static const auto value = decltype(test<C, V>(0))::value;
            using type              = std::integral_constant<bool, value>;
        };

        /// A search function
        template <typename T, typename V, enable_if_t<!has_find<T, V>::value, detail::enabler> = detail::dummy>
        auto search(const T &set, const V &val) -> std::pair<bool, decltype(std::begin(detail::smart_deref(set)))>
        {
            using element_t = typename detail::element_type<T>::type;
            auto &setref    = detail::smart_deref(set);
            auto it         = std::find_if(std::begin(setref),
                                   std::end(setref),
                                   [&val](decltype(*std::begin(setref)) v)
                                   {
                                       return (detail::pair_adaptor<element_t>::first(v) == val);
                                   });
            return {(it != std::end(setref)), it};
        }

        /// A search function that uses the built in find function
        template <typename T, typename V, enable_if_t<has_find<T, V>::value, detail::enabler> = detail::dummy>
        auto search(const T &set, const V &val) -> std::pair<bool, decltype(std::begin(detail::smart_deref(set)))>
        {
            auto &setref = detail::smart_deref(set);
            auto it      = setref.find(val);
            return {(it != std::end(setref)), it};
        }

        /// A search function with a filter function
        template <typename T, typename V>
        auto search(const T &set, const V &val, const std::function<V(V)> &filter_function)
            -> std::pair<bool, decltype(std::begin(detail::smart_deref(set)))>
        {
            using element_t = typename detail::element_type<T>::type;
            // do the potentially faster first search
            auto res = search(set, val);
            if((res.first) || (!(filter_function)))
            {
                return res;
            }
            // if we haven't found it do the longer linear search with all the element translations
            auto &setref = detail::smart_deref(set);
            auto it      = std::find_if(std::begin(setref),
                                   std::end(setref),
                                   [&](decltype(*std::begin(setref)) v)
                                   {
                                       V a{detail::pair_adaptor<element_t>::first(v)};
                                       a = filter_function(a);
                                       return (a == val);
                                   });
            return {(it != std::end(setref)), it};
        }

        // the following suggestion was made by Nikita Ofitserov(@himikof)
        // done in templates to prevent compiler warnings on negation of unsigned numbers

        /// Do a check for overflow on signed numbers
        template <typename T>
        inline typename std::enable_if<std::is_signed<T>::value, T>::type overflowCheck(const T &a, const T &b)
        {
            if((a > 0) == (b > 0))
            {
                return ((std::numeric_limits<T>::max)() / (std::abs)(a) < (std::abs)(b));
            }
            else
            {
                return ((std::numeric_limits<T>::min)() / (std::abs)(a) > -(std::abs)(b));
            }
        }
        /// Do a check for overflow on unsigned numbers
        template <typename T>
        inline typename std::enable_if<!std::is_signed<T>::value, T>::type overflowCheck(const T &a, const T &b)
        {
            return ((std::numeric_limits<T>::max)() / a < b);
        }

        /// Performs a *= b; if it doesn't cause integer overflow. Returns false otherwise.
        template <typename T>
        typename std::enable_if<std::is_integral<T>::value, bool>::type checked_multiply(T &a, T b)
        {
            if(a == 0 || b == 0 || a == 1 || b == 1)
            {
                a *= b;
                return true;
            }
            if(a == (std::numeric_limits<T>::min)() || b == (std::numeric_limits<T>::min)())
            {
                return false;
            }
            if(overflowCheck(a, b))
            {
                return false;
            }
            a *= b;
            return true;
        }

        /// Performs a *= b; if it doesn't equal infinity. Returns false otherwise.
        template <typename T>
        typename std::enable_if<std::is_floating_point<T>::value, bool>::type checked_multiply(T &a, T b)
        {
            T c = a * b;
            if(std::isinf(c) && !std::isinf(a) && !std::isinf(b))
            {
                return false;
            }
            a = c;
            return true;
        }

    }  // namespace detail
    /// Verify items are in a set
    class IsMember : public Validator
    {
       public:
        using filter_fn_t = std::function<std::string(std::string)>;

        /// This allows in-place construction using an initializer list
        template <typename T, typename... Args>
        IsMember(std::initializer_list<T> values, Args &&...args)
            : IsMember(std::vector<T>(values), std::forward<Args>(args)...)
        {}

        /// This checks to see if an item is in a set (empty function)
        template <typename T>
        explicit IsMember(T &&set)
            : IsMember(std::forward<T>(set), nullptr)
        {}

        /// This checks to see if an item is in a set: pointer or copy version. You can pass in a function that will
        /// filter both sides of the comparison before computing the comparison.
        template <typename T, typename F>
        explicit IsMember(T set, F filter_function)
        {
            // Get the type of the contained item - requires a container have ::value_type
            // if the type does not have first_type and second_type, these are both value_type
            using element_t = typename detail::element_type<T>::type;  // Removes (smart) pointers if needed
            using item_t    = typename detail::pair_adaptor<element_t>::first_type;  // Is value_type if not a map

            using local_item_t = typename IsMemberType<item_t>::type;  // This will convert bad types to good ones
            // (const char * to std::string)

            // Make a local copy of the filter function, using a std::function if not one already
            std::function<local_item_t(local_item_t)> filter_fn = filter_function;

            // This is the type name for help, it will take the current version of the set contents
            desc_function_ = [set]()
            {
                return detail::generate_set(detail::smart_deref(set));
            };

            // This is the function that validates
            // It stores a copy of the set pointer-like, so shared_ptr will stay alive
            func_ = [set, filter_fn](std::string &input)
            {
                local_item_t b;
                if(!detail::lexical_cast(input, b))
                {
                    throw ValidationError(input);  // name is added later
                }
                if(filter_fn)
                {
                    b = filter_fn(b);
                }
                auto res = detail::search(set, b, filter_fn);
                if(res.first)
                {
                    // Make sure the version in the input string is identical to the one in the set
                    if(filter_fn)
                    {
                        input = detail::value_string(detail::pair_adaptor<element_t>::first(*(res.second)));
                    }

                    // Return empty error string (success)
                    return std::string{};
                }

                // If you reach this point, the result was not found
                return input + " not in " + detail::generate_set(detail::smart_deref(set));
            };
        }

        /// You can pass in as many filter functions as you like, they nest (string only currently)
        template <typename T, typename... Args>
        IsMember(T &&set, filter_fn_t filter_fn_1, filter_fn_t filter_fn_2, Args &&...other)
            : IsMember(
                  std::forward<T>(set),
                  [filter_fn_1, filter_fn_2](std::string a)
                  {
                      return filter_fn_2(filter_fn_1(a));
                  },
                  other...)
        {}
    };

    /// definition of the default transformation object
    template <typename T>
    using TransformPairs = std::vector<std::pair<std::string, T>>;

    /// Translate named items to other or a value set
    class Transformer : public Validator
    {
       public:
        using filter_fn_t = std::function<std::string(std::string)>;

        /// This allows in-place construction
        template <typename... Args>
        Transformer(std::initializer_list<std::pair<std::string, std::string>> values, Args &&...args)
            : Transformer(TransformPairs<std::string>(values), std::forward<Args>(args)...)
        {}

        /// direct map of std::string to std::string
        template <typename T>
        explicit Transformer(T &&mapping)
            : Transformer(std::forward<T>(mapping), nullptr)
        {}

        /// This checks to see if an item is in a set: pointer or copy version. You can pass in a function that will
        /// filter both sides of the comparison before computing the comparison.
        template <typename T, typename F>
        explicit Transformer(T mapping, F filter_function)
        {
            static_assert(detail::pair_adaptor<typename detail::element_type<T>::type>::value,
                          "mapping must produce value pairs");
            // Get the type of the contained item - requires a container have ::value_type
            // if the type does not have first_type and second_type, these are both value_type
            using element_t    = typename detail::element_type<T>::type;  // Removes (smart) pointers if needed
            using item_t       = typename detail::pair_adaptor<element_t>::first_type;  // Is value_type if not a map
            using local_item_t = typename IsMemberType<item_t>::type;  // Will convert bad types to good ones
            // (const char * to std::string)

            // Make a local copy of the filter function, using a std::function if not one already
            std::function<local_item_t(local_item_t)> filter_fn = filter_function;

            // This is the type name for help, it will take the current version of the set contents
            desc_function_ = [mapping]()
            {
                return detail::generate_map(detail::smart_deref(mapping));
            };

            func_ = [mapping, filter_fn](std::string &input)
            {
                local_item_t b;
                if(!detail::lexical_cast(input, b))
                {
                    return std::string();
                    // there is no possible way we can match anything in the mapping if we can't convert so just return
                }
                if(filter_fn)
                {
                    b = filter_fn(b);
                }
                auto res = detail::search(mapping, b, filter_fn);
                if(res.first)
                {
                    input = detail::value_string(detail::pair_adaptor<element_t>::second(*res.second));
                }
                return std::string{};
            };
        }

        /// You can pass in as many filter functions as you like, they nest
        template <typename T, typename... Args>
        Transformer(T &&mapping, filter_fn_t filter_fn_1, filter_fn_t filter_fn_2, Args &&...other)
            : Transformer(
                  std::forward<T>(mapping),
                  [filter_fn_1, filter_fn_2](std::string a)
                  {
                      return filter_fn_2(filter_fn_1(a));
                  },
                  other...)
        {}
    };

    /// translate named items to other or a value set
    class CheckedTransformer : public Validator
    {
       public:
        using filter_fn_t = std::function<std::string(std::string)>;

        /// This allows in-place construction
        template <typename... Args>
        CheckedTransformer(std::initializer_list<std::pair<std::string, std::string>> values, Args &&...args)
            : CheckedTransformer(TransformPairs<std::string>(values), std::forward<Args>(args)...)
        {}

        /// direct map of std::string to std::string
        template <typename T>
        explicit CheckedTransformer(T mapping)
            : CheckedTransformer(std::move(mapping), nullptr)
        {}

        /// This checks to see if an item is in a set: pointer or copy version. You can pass in a function that will
        /// filter both sides of the comparison before computing the comparison.
        template <typename T, typename F>
        explicit CheckedTransformer(T mapping, F filter_function)
        {
            static_assert(detail::pair_adaptor<typename detail::element_type<T>::type>::value,
                          "mapping must produce value pairs");
            // Get the type of the contained item - requires a container have ::value_type
            // if the type does not have first_type and second_type, these are both value_type
            using element_t    = typename detail::element_type<T>::type;  // Removes (smart) pointers if needed
            using item_t       = typename detail::pair_adaptor<element_t>::first_type;  // Is value_type if not a map
            using local_item_t = typename IsMemberType<item_t>::type;  // Will convert bad types to good ones
            // (const char * to std::string)
            using iteration_type_t =
                typename detail::pair_adaptor<element_t>::value_type;  // the type of the object pair

            // Make a local copy of the filter function, using a std::function if not one already
            std::function<local_item_t(local_item_t)> filter_fn = filter_function;

            auto tfunc = [mapping]()
            {
                std::string out("value in ");
                out += detail::generate_map(detail::smart_deref(mapping)) + " OR {";
                out += detail::join(
                    detail::smart_deref(mapping),
                    [](const iteration_type_t &v)
                    {
                        return detail::to_string(detail::pair_adaptor<element_t>::second(v));
                    },
                    ",");
                out.push_back('}');
                return out;
            };

            desc_function_ = tfunc;

            func_ = [mapping, tfunc, filter_fn](std::string &input)
            {
                local_item_t b;
                bool converted = detail::lexical_cast(input, b);
                if(converted)
                {
                    if(filter_fn)
                    {
                        b = filter_fn(b);
                    }
                    auto res = detail::search(mapping, b, filter_fn);
                    if(res.first)
                    {
                        input = detail::value_string(detail::pair_adaptor<element_t>::second(*res.second));
                        return std::string{};
                    }
                }
                for(const auto &v: detail::smart_deref(mapping))
                {
                    auto output_string = detail::value_string(detail::pair_adaptor<element_t>::second(v));
                    if(output_string == input)
                    {
                        return std::string();
                    }
                }

                return "Check " + input + " " + tfunc() + " FAILED";
            };
        }

        /// You can pass in as many filter functions as you like, they nest
        template <typename T, typename... Args>
        CheckedTransformer(T &&mapping, filter_fn_t filter_fn_1, filter_fn_t filter_fn_2, Args &&...other)
            : CheckedTransformer(
                  std::forward<T>(mapping),
                  [filter_fn_1, filter_fn_2](std::string a)
                  {
                      return filter_fn_2(filter_fn_1(a));
                  },
                  other...)
        {}
    };

    /// Helper function to allow ignore_case to be passed to IsMember or Transform
    inline std::string ignore_case(std::string item)
    {
        return detail::to_lower(item);
    }

    /// Helper function to allow ignore_underscore to be passed to IsMember or Transform
    inline std::string ignore_underscore(std::string item)
    {
        return detail::remove_underscore(item);
    }

    /// Helper function to allow checks to ignore spaces to be passed to IsMember or Transform
    inline std::string ignore_space(std::string item)
    {
        item.erase(std::remove(std::begin(item), std::end(item), ' '), std::end(item));
        item.erase(std::remove(std::begin(item), std::end(item), '\t'), std::end(item));
        return item;
    }

    /// Multiply a number by a factor using given mapping.
    /// Can be used to write transforms for SIZE or DURATION inputs.
    ///
    /// Example:
    ///   With mapping = `{"b"->1, "kb"->1024, "mb"->1024*1024}`
    ///   one can recognize inputs like "100", "12kb", "100 MB",
    ///   that will be automatically transformed to 100, 14448, 104857600.
    ///
    /// Output number type matches the type in the provided mapping.
    /// Therefore, if it is required to interpret real inputs like "0.42 s",
    /// the mapping should be of a type <string, float> or <string, double>.
    class AsNumberWithUnit : public Validator
    {
       public:
        /// Adjust AsNumberWithUnit behavior.
        /// CASE_SENSITIVE/CASE_INSENSITIVE controls how units are matched.
        /// UNIT_OPTIONAL/UNIT_REQUIRED throws ValidationError
        ///   if UNIT_REQUIRED is set and unit literal is not found.
        enum Options
        {
            CASE_SENSITIVE   = 0,
            CASE_INSENSITIVE = 1,
            UNIT_OPTIONAL    = 0,
            UNIT_REQUIRED    = 2,
            DEFAULT          = CASE_INSENSITIVE | UNIT_OPTIONAL
        };

        template <typename Number>
        explicit AsNumberWithUnit(std::map<std::string, Number> mapping,
                                  Options opts                 = DEFAULT,
                                  const std::string &unit_name = "UNIT")
        {
            description(generate_description<Number>(unit_name, opts));
            validate_mapping(mapping, opts);

            // transform function
            func_ = [mapping, opts](std::string &input) -> std::string
            {
                Number num;

                detail::rtrim(input);
                if(input.empty())
                {
                    throw ValidationError("Input is empty");
                }

                // Find split position between number and prefix
                auto unit_begin = input.end();
                while(unit_begin > input.begin() && std::isalpha(*(unit_begin - 1), std::locale()))
                {
                    --unit_begin;
                }

                std::string unit{unit_begin, input.end()};
                input.resize(static_cast<std::size_t>(std::distance(input.begin(), unit_begin)));
                detail::trim(input);

                if(opts & UNIT_REQUIRED && unit.empty())
                {
                    throw ValidationError("Missing mandatory unit");
                }
                if(opts & CASE_INSENSITIVE)
                {
                    unit = detail::to_lower(unit);
                }
                if(unit.empty())
                {
                    if(!detail::lexical_cast(input, num))
                    {
                        throw ValidationError(std::string("Value ") + input + " could not be converted to " +
                                              detail::type_name<Number>());
                    }
                    // No need to modify input if no unit passed
                    return {};
                }

                // find corresponding factor
                auto it = mapping.find(unit);
                if(it == mapping.end())
                {
                    throw ValidationError(unit +
                                          " unit not recognized. "
                                          "Allowed values: " +
                                          detail::generate_map(mapping, true));
                }

                if(!input.empty())
                {
                    bool converted = detail::lexical_cast(input, num);
                    if(!converted)
                    {
                        throw ValidationError(std::string("Value ") + input + " could not be converted to " +
                                              detail::type_name<Number>());
                    }
                    // perform safe multiplication
                    bool ok = detail::checked_multiply(num, it->second);
                    if(!ok)
                    {
                        throw ValidationError(detail::to_string(num) + " multiplied by " + unit +
                                              " factor would cause number overflow. Use smaller value.");
                    }
                }
                else
                {
                    num = static_cast<Number>(it->second);
                }

                input = detail::to_string(num);

                return {};
            };
        }

       private:
        /// Check that mapping contains valid units.
        /// Update mapping for CASE_INSENSITIVE mode.
        template <typename Number>
        static void validate_mapping(std::map<std::string, Number> &mapping, Options opts)
        {
            for(auto &kv: mapping)
            {
                if(kv.first.empty())
                {
                    throw ValidationError("Unit must not be empty.");
                }
                if(!detail::isalpha(kv.first))
                {
                    throw ValidationError("Unit must contain only letters.");
                }
            }

            // make all units lowercase if CASE_INSENSITIVE
            if(opts & CASE_INSENSITIVE)
            {
                std::map<std::string, Number> lower_mapping;
                for(auto &kv: mapping)
                {
                    auto s = detail::to_lower(kv.first);
                    if(lower_mapping.count(s))
                    {
                        throw ValidationError(
                            std::string("Several matching lowercase unit representations are found: ") + s);
                    }
                    lower_mapping[detail::to_lower(kv.first)] = kv.second;
                }
                mapping = std::move(lower_mapping);
            }
        }

        /// Generate description like this: NUMBER [UNIT]
        template <typename Number>
        static std::string generate_description(const std::string &name, Options opts)
        {
            std::stringstream out;
            out << detail::type_name<Number>() << ' ';
            if(opts & UNIT_REQUIRED)
            {
                out << name;
            }
            else
            {
                out << '[' << name << ']';
            }
            return out.str();
        }
    };

    /// Converts a human-readable size string (with unit literal) to uin64_t size.
    /// Example:
    ///   "100" => 100
    ///   "1 b" => 100
    ///   "10Kb" => 10240 // you can configure this to be interpreted as kilobyte (*1000) or kibibyte (*1024)
    ///   "10 KB" => 10240
    ///   "10 kb" => 10240
    ///   "10 kib" => 10240 // *i, *ib are always interpreted as *bibyte (*1024)
    ///   "10kb" => 10240
    ///   "2 MB" => 2097152
    ///   "2 EiB" => 2^61 // Units up to exibyte are supported
    class AsSizeValue : public AsNumberWithUnit
    {
       public:
        using result_t = std::uint64_t;

        /// If kb_is_1000 is true,
        /// interpret 'kb', 'k' as 1000 and 'kib', 'ki' as 1024
        /// (same applies to higher order units as well).
        /// Otherwise, interpret all literals as factors of 1024.
        /// The first option is formally correct, but
        /// the second interpretation is more wide-spread
        /// (see https://en.wikipedia.org/wiki/Binary_prefix).
        explicit AsSizeValue(bool kb_is_1000)
            : AsNumberWithUnit(get_mapping(kb_is_1000))
        {
            if(kb_is_1000)
            {
                description("SIZE [b, kb(=1000b), kib(=1024b), ...]");
            }
            else
            {
                description("SIZE [b, kb(=1024b), ...]");
            }
        }

       private:
        /// Get <size unit, factor> mapping
        static std::map<std::string, result_t> init_mapping(bool kb_is_1000)
        {
            std::map<std::string, result_t> m;
            result_t k_factor  = kb_is_1000 ? 1000 : 1024;
            result_t ki_factor = 1024;
            result_t k         = 1;
            result_t ki        = 1;
            m["b"]             = 1;
            for(std::string p: {"k", "m", "g", "t", "p", "e"})
            {
                k *= k_factor;
                ki *= ki_factor;
                m[p]        = k;
                m[p + "b"]  = k;
                m[p + "i"]  = ki;
                m[p + "ib"] = ki;
            }
            return m;
        }

        /// Cache calculated mapping
        static std::map<std::string, result_t> get_mapping(bool kb_is_1000)
        {
            if(kb_is_1000)
            {
                static auto m = init_mapping(true);
                return m;
            }
            else
            {
                static auto m = init_mapping(false);
                return m;
            }
        }
    };

    namespace detail
    {
        /// Split a string into a program name and command line arguments
        /// the string is assumed to contain a file name followed by other arguments
        /// the return value contains is a pair with the first argument containing the program name and the second
        /// everything else.
        inline std::pair<std::string, std::string> split_program_name(std::string commandline)
        {
            // try to determine the programName
            std::pair<std::string, std::string> vals;
            trim(commandline);
            auto esp = commandline.find_first_of(' ', 1);
            while(detail::check_path(commandline.substr(0, esp).c_str()) != path_type::file)
            {
                esp = commandline.find_first_of(' ', esp + 1);
                if(esp == std::string::npos)
                {
                    // if we have reached the end and haven't found a valid file just assume the first argument is the
                    // program name
                    if(commandline[0] == '"' || commandline[0] == '\'' || commandline[0] == '`')
                    {
                        bool embeddedQuote = false;
                        auto keyChar       = commandline[0];
                        auto end           = commandline.find_first_of(keyChar, 1);
                        while((end != std::string::npos) && (commandline[end - 1] == '\\'))
                        {  // deal with escaped quotes
                            end           = commandline.find_first_of(keyChar, end + 1);
                            embeddedQuote = true;
                        }
                        if(end != std::string::npos)
                        {
                            vals.first = commandline.substr(1, end - 1);
                            esp        = end + 1;
                            if(embeddedQuote)
                            {
                                vals.first =
                                    find_and_replace(vals.first, std::string("\\") + keyChar, std::string(1, keyChar));
                                embeddedQuote = false;
                            }
                        }
                        else
                        {
                            esp = commandline.find_first_of(' ', 1);
                        }
                    }
                    else
                    {
                        esp = commandline.find_first_of(' ', 1);
                    }

                    break;
                }
            }
            if(vals.first.empty())
            {
                vals.first = commandline.substr(0, esp);
                rtrim(vals.first);
            }

            // strip the program name
            vals.second = (esp != std::string::npos) ? commandline.substr(esp + 1) : std::string{};
            ltrim(vals.second);
            return vals;
        }

    }  // namespace detail
    /// @}

    class Option;
    class App;

    /// This enum signifies the type of help requested
    ///
    /// This is passed in by App; all user classes must accept this as
    /// the second argument.

    enum class AppFormatMode
    {
        Normal,  ///< The normal, detailed help
        All,     ///< A fully expanded help
        Sub,     ///< Used when printed as part of expanded subcommand
    };

    /// This is the minimum requirements to run a formatter.
    ///
    /// A user can subclass this is if they do not care at all
    /// about the structure in CLI::Formatter.
    class FormatterBase
    {
       protected:
        /// @name Options
        ///@{

        /// The width of the first column
        std::size_t column_width_{30};

        /// @brief The required help printout labels (user changeable)
        /// Values are Needs, Excludes, etc.
        std::map<std::string, std::string> labels_{};

        ///@}
        /// @name Basic
        ///@{

       public:
        FormatterBase()                      = default;
        FormatterBase(const FormatterBase &) = default;
        FormatterBase(FormatterBase &&)      = default;

        /// Adding a destructor in this form to work around bug in GCC 4.7
        virtual ~FormatterBase() noexcept {}  // NOLINT(modernize-use-equals-default)

        /// This is the key method that puts together help
        virtual std::string make_help(const App *, std::string, AppFormatMode) const = 0;

        ///@}
        /// @name Setters
        ///@{

        /// Set the "REQUIRED" label
        void label(std::string key, std::string val)
        {
            labels_[key] = val;
        }

        /// Set the column width
        void column_width(std::size_t val)
        {
            column_width_ = val;
        }

        ///@}
        /// @name Getters
        ///@{

        /// Get the current value of a name (REQUIRED, etc.)
        std::string get_label(std::string key) const
        {
            if(labels_.find(key) == labels_.end())
                return key;
            else
                return labels_.at(key);
        }

        /// Get the current column width
        std::size_t get_column_width() const
        {
            return column_width_;
        }

        ///@}
    };

    /// This is a specialty override for lambda functions
    class FormatterLambda final : public FormatterBase
    {
        using funct_t = std::function<std::string(const App *, std::string, AppFormatMode)>;

        /// The lambda to hold and run
        funct_t lambda_;

       public:
        /// Create a FormatterLambda with a lambda function
        explicit FormatterLambda(funct_t funct)
            : lambda_(std::move(funct))
        {}

        /// Adding a destructor (mostly to make GCC 4.7 happy)
        ~FormatterLambda() noexcept override {}  // NOLINT(modernize-use-equals-default)

        /// This will simply call the lambda function
        std::string make_help(const App *app, std::string name, AppFormatMode mode) const override
        {
            return lambda_(app, name, mode);
        }
    };

    /// This is the default Formatter for CLI11. It pretty prints help output, and is broken into quite a few
    /// overridable methods, to be highly customizable with minimal effort.
    class Formatter : public FormatterBase
    {
       public:
        Formatter()                  = default;
        Formatter(const Formatter &) = default;
        Formatter(Formatter &&)      = default;

        /// @name Overridables
        ///@{

        /// This prints out a group of options with title
        ///
        virtual std::string make_group(std::string group, bool is_positional, std::vector<const Option *> opts) const;

        /// This prints out just the positionals "group"
        virtual std::string make_positionals(const App *app) const;

        /// This prints out all the groups of options
        std::string make_groups(const App *app, AppFormatMode mode) const;

        /// This prints out all the subcommands
        virtual std::string make_subcommands(const App *app, AppFormatMode mode) const;

        /// This prints out a subcommand
        virtual std::string make_subcommand(const App *sub) const;

        /// This prints out a subcommand in help-all
        virtual std::string make_expanded(const App *sub) const;

        /// This prints out all the groups of options
        virtual std::string make_footer(const App *app) const;

        /// This displays the description line
        virtual std::string make_description(const App *app) const;

        /// This displays the usage line
        virtual std::string make_usage(const App *app, std::string name) const;

        /// This puts everything together
        std::string make_help(const App * /*app*/, std::string, AppFormatMode) const override;

        ///@}
        /// @name Options
        ///@{

        /// This prints out an option help line, either positional or optional form
        virtual std::string make_option(const Option *opt, bool is_positional) const
        {
            std::stringstream out;
            detail::format_help(out,
                                make_option_name(opt, is_positional) + make_option_opts(opt),
                                make_option_desc(opt),
                                column_width_);
            return out.str();
        }

        /// @brief This is the name part of an option, Default: left column
        virtual std::string make_option_name(const Option *, bool) const;

        /// @brief This is the options part of the name, Default: combined into left column
        virtual std::string make_option_opts(const Option *) const;

        /// @brief This is the description. Default: Right column, on new line if left column too large
        virtual std::string make_option_desc(const Option *) const;

        /// @brief This is used to print the name on the USAGE line
        virtual std::string make_option_usage(const Option *opt) const;

        ///@}
    };

    using results_t = std::vector<std::string>;
    /// callback function definition
    using callback_t = std::function<bool(const results_t &)>;

    class Option;
    class App;

    using Option_p = std::unique_ptr<Option>;
    /// Enumeration of the multiOption Policy selection
    enum class MultiOptionPolicy : char
    {
        Throw,      //!< Throw an error if any extra arguments were given
        TakeLast,   //!< take only the last Expected number of arguments
        TakeFirst,  //!< take only the first Expected number of arguments
        Join,       //!< merge all the arguments together into a single string via the delimiter character default('\n')
        TakeAll     //!< just get all the passed argument regardless
    };

    /// This is the CRTP base class for Option and OptionDefaults. It was designed this way
    /// to share parts of the class; an OptionDefaults can copy to an Option.
    template <typename CRTP>
    class OptionBase
    {
        friend App;

       protected:
        /// The group membership
        std::string group_ = std::string("Options");

        /// True if this is a required option
        bool required_{false};

        /// Ignore the case when matching (option, not value)
        bool ignore_case_{false};

        /// Ignore underscores when matching (option, not value)
        bool ignore_underscore_{false};

        /// Allow this option to be given in a configuration file
        bool configurable_{true};

        /// Disable overriding flag values with '=value'
        bool disable_flag_override_{false};

        /// Specify a delimiter character for vector arguments
        char delimiter_{'\0'};

        /// Automatically capture default value
        bool always_capture_default_{false};

        /// Policy for handling multiple arguments beyond the expected Max
        MultiOptionPolicy multi_option_policy_{MultiOptionPolicy::Throw};

        /// Copy the contents to another similar class (one based on OptionBase)
        template <typename T>
        void copy_to(T *other) const
        {
            other->group(group_);
            other->required(required_);
            other->ignore_case(ignore_case_);
            other->ignore_underscore(ignore_underscore_);
            other->configurable(configurable_);
            other->disable_flag_override(disable_flag_override_);
            other->delimiter(delimiter_);
            other->always_capture_default(always_capture_default_);
            other->multi_option_policy(multi_option_policy_);
        }

       public:
        // setters

        /// Changes the group membership
        CRTP *group(const std::string &name)
        {
            group_ = name;
            return static_cast<CRTP *>(this);
        }

        /// Set the option as required
        CRTP *required(bool value = true)
        {
            required_ = value;
            return static_cast<CRTP *>(this);
        }

        /// Support Plumbum term
        CRTP *mandatory(bool value = true)
        {
            return required(value);
        }

        CRTP *always_capture_default(bool value = true)
        {
            always_capture_default_ = value;
            return static_cast<CRTP *>(this);
        }

        // Getters

        /// Get the group of this option
        const std::string &get_group() const
        {
            return group_;
        }

        /// True if this is a required option
        bool get_required() const
        {
            return required_;
        }

        /// The status of ignore case
        bool get_ignore_case() const
        {
            return ignore_case_;
        }

        /// The status of ignore_underscore
        bool get_ignore_underscore() const
        {
            return ignore_underscore_;
        }

        /// The status of configurable
        bool get_configurable() const
        {
            return configurable_;
        }

        /// The status of configurable
        bool get_disable_flag_override() const
        {
            return disable_flag_override_;
        }

        /// Get the current delimiter char
        char get_delimiter() const
        {
            return delimiter_;
        }

        /// Return true if this will automatically capture the default value for help printing
        bool get_always_capture_default() const
        {
            return always_capture_default_;
        }

        /// The status of the multi option policy
        MultiOptionPolicy get_multi_option_policy() const
        {
            return multi_option_policy_;
        }

        // Shortcuts for multi option policy

        /// Set the multi option policy to take last
        CRTP *take_last()
        {
            auto self = static_cast<CRTP *>(this);
            self->multi_option_policy(MultiOptionPolicy::TakeLast);
            return self;
        }

        /// Set the multi option policy to take last
        CRTP *take_first()
        {
            auto self = static_cast<CRTP *>(this);
            self->multi_option_policy(MultiOptionPolicy::TakeFirst);
            return self;
        }

        /// Set the multi option policy to take all arguments
        CRTP *take_all()
        {
            auto self = static_cast<CRTP *>(this);
            self->multi_option_policy(MultiOptionPolicy::TakeAll);
            return self;
        }

        /// Set the multi option policy to join
        CRTP *join()
        {
            auto self = static_cast<CRTP *>(this);
            self->multi_option_policy(MultiOptionPolicy::Join);
            return self;
        }

        /// Set the multi option policy to join with a specific delimiter
        CRTP *join(char delim)
        {
            auto self        = static_cast<CRTP *>(this);
            self->delimiter_ = delim;
            self->multi_option_policy(MultiOptionPolicy::Join);
            return self;
        }

        /// Allow in a configuration file
        CRTP *configurable(bool value = true)
        {
            configurable_ = value;
            return static_cast<CRTP *>(this);
        }

        /// Allow in a configuration file
        CRTP *delimiter(char value = '\0')
        {
            delimiter_ = value;
            return static_cast<CRTP *>(this);
        }
    };

    /// This is a version of OptionBase that only supports setting values,
    /// for defaults. It is stored as the default option in an App.
    class OptionDefaults : public OptionBase<OptionDefaults>
    {
       public:
        OptionDefaults() = default;

        // Methods here need a different implementation if they are Option vs. OptionDefault

        /// Take the last argument if given multiple times
        OptionDefaults *multi_option_policy(MultiOptionPolicy value = MultiOptionPolicy::Throw)
        {
            multi_option_policy_ = value;
            return this;
        }

        /// Ignore the case of the option name
        OptionDefaults *ignore_case(bool value = true)
        {
            ignore_case_ = value;
            return this;
        }

        /// Ignore underscores in the option name
        OptionDefaults *ignore_underscore(bool value = true)
        {
            ignore_underscore_ = value;
            return this;
        }

        /// Disable overriding flag values with an '=<value>' segment
        OptionDefaults *disable_flag_override(bool value = true)
        {
            disable_flag_override_ = value;
            return this;
        }

        /// set a delimiter character to split up single arguments to treat as multiple inputs
        OptionDefaults *delimiter(char value = '\0')
        {
            delimiter_ = value;
            return this;
        }
    };

    class Option : public OptionBase<Option>
    {
        friend App;

       protected:
        /// @name Names
        ///@{

        /// A list of the short names (`-a`) without the leading dashes
        std::vector<std::string> snames_{};

        /// A list of the long names (`--long`) without the leading dashes
        std::vector<std::string> lnames_{};

        /// A list of the flag names with the appropriate default value, the first part of the pair should be duplicates
        /// of what is in snames or lnames but will trigger a particular response on a flag
        std::vector<std::pair<std::string, std::string>> default_flag_values_{};

        /// a list of flag names with specified default values;
        std::vector<std::string> fnames_{};

        /// A positional name
        std::string pname_{};

        /// If given, check the environment for this option
        std::string envname_{};

        ///@}
        /// @name Help
        ///@{

        /// The description for help strings
        std::string description_{};

        /// A human readable default value, either manually set, captured, or captured by default
        std::string default_str_{};

        /// If given, replace the text that describes the option type and usage in the help text
        std::string option_text_{};

        /// A human readable type value, set when App creates this
        ///
        /// This is a lambda function so "types" can be dynamic, such as when a set prints its contents.
        std::function<std::string()> type_name_{[]()
                                                {
                                                    return std::string();
                                                }};

        /// Run this function to capture a default (ignore if empty)
        std::function<std::string()> default_function_{};

        ///@}
        /// @name Configuration
        ///@{

        /// The number of arguments that make up one option. max is the nominal type size, min is the minimum number of
        /// strings
        int type_size_max_{1};
        /// The minimum number of arguments an option should be expecting
        int type_size_min_{1};

        /// The minimum number of expected values
        int expected_min_{1};
        /// The maximum number of expected values
        int expected_max_{1};

        /// A list of Validators to run on each value parsed
        std::vector<Validator> validators_{};

        /// A list of options that are required with this option
        std::set<Option *> needs_{};

        /// A list of options that are excluded with this option
        std::set<Option *> excludes_{};

        ///@}
        /// @name Other
        ///@{

        /// link back up to the parent App for fallthrough
        App *parent_{nullptr};

        /// Options store a callback to do all the work
        callback_t callback_{};

        ///@}
        /// @name Parsing results
        ///@{

        /// complete Results of parsing
        results_t results_{};
        /// results after reduction
        results_t proc_results_{};
        /// enumeration for the option state machine
        enum class option_state : char
        {
            parsing      = 0,  //!< The option is currently collecting parsed results
            validated    = 2,  //!< the results have been validated
            reduced      = 4,  //!< a subset of results has been generated
            callback_run = 6,  //!< the callback has been executed
        };
        /// Whether the callback has run (needed for INI parsing)
        option_state current_option_state_{option_state::parsing};
        /// Specify that extra args beyond type_size_max should be allowed
        bool allow_extra_args_{false};
        /// Specify that the option should act like a flag vs regular option
        bool flag_like_{false};
        /// Control option to run the callback to set the default
        bool run_callback_for_default_{false};
        /// flag indicating a separator needs to be injected after each argument call
        bool inject_separator_{false};
        ///@}

        /// Making an option by hand is not defined, it must be made by the App class
        Option(std::string option_name, std::string option_description, callback_t callback, App *parent)
            : description_(std::move(option_description))
            , parent_(parent)
            , callback_(std::move(callback))
        {
            std::tie(snames_, lnames_, pname_) = detail::get_names(detail::split_names(option_name));
        }

       public:
        /// @name Basic
        ///@{

        Option(const Option &) = delete;
        Option &operator=(const Option &) = delete;

        /// Count the total number of times an option was passed
        std::size_t count() const
        {
            return results_.size();
        }

        /// True if the option was not passed
        bool empty() const
        {
            return results_.empty();
        }

        /// This class is true if option is passed.
        explicit operator bool() const
        {
            return !empty();
        }

        /// Clear the parsed results (mostly for testing)
        void clear()
        {
            results_.clear();
            current_option_state_ = option_state::parsing;
        }

        ///@}
        /// @name Setting options
        ///@{

        /// Set the number of expected arguments
        Option *expected(int value)
        {
            if(value < 0)
            {
                expected_min_ = -value;
                if(expected_max_ < expected_min_)
                {
                    expected_max_ = expected_min_;
                }
                allow_extra_args_ = true;
                flag_like_        = false;
            }
            else if(value == detail::expected_max_vector_size)
            {
                expected_min_     = 1;
                expected_max_     = detail::expected_max_vector_size;
                allow_extra_args_ = true;
                flag_like_        = false;
            }
            else
            {
                expected_min_ = value;
                expected_max_ = value;
                flag_like_    = (expected_min_ == 0);
            }
            return this;
        }

        /// Set the range of expected arguments
        Option *expected(int value_min, int value_max)
        {
            if(value_min < 0)
            {
                value_min = -value_min;
            }

            if(value_max < 0)
            {
                value_max = detail::expected_max_vector_size;
            }
            if(value_max < value_min)
            {
                expected_min_ = value_max;
                expected_max_ = value_min;
            }
            else
            {
                expected_max_ = value_max;
                expected_min_ = value_min;
            }

            return this;
        }
        /// Set the value of allow_extra_args which allows extra value arguments on the flag or option to be included
        /// with each instance
        Option *allow_extra_args(bool value = true)
        {
            allow_extra_args_ = value;
            return this;
        }
        /// Get the current value of allow extra args
        bool get_allow_extra_args() const
        {
            return allow_extra_args_;
        }

        /// Set the value of run_callback_for_default which controls whether the callback function should be called to
        /// set the default This is controlled automatically but could be manipulated by the user.
        Option *run_callback_for_default(bool value = true)
        {
            run_callback_for_default_ = value;
            return this;
        }
        /// Get the current value of run_callback_for_default
        bool get_run_callback_for_default() const
        {
            return run_callback_for_default_;
        }

        /// Adds a Validator with a built in type name
        Option *check(Validator validator, const std::string &validator_name = "")
        {
            validator.non_modifying();
            validators_.push_back(std::move(validator));
            if(!validator_name.empty())
                validators_.back().name(validator_name);
            return this;
        }

        /// Adds a Validator. Takes a const string& and returns an error message (empty if conversion/check is okay).
        Option *check(std::function<std::string(const std::string &)> Validator,
                      std::string Validator_description = "",
                      std::string Validator_name        = "")
        {
            validators_.emplace_back(Validator, std::move(Validator_description), std::move(Validator_name));
            validators_.back().non_modifying();
            return this;
        }

        /// Adds a transforming Validator with a built in type name
        Option *transform(Validator Validator, const std::string &Validator_name = "")
        {
            validators_.insert(validators_.begin(), std::move(Validator));
            if(!Validator_name.empty())
                validators_.front().name(Validator_name);
            return this;
        }

        /// Adds a Validator-like function that can change result
        Option *transform(const std::function<std::string(std::string)> &func,
                          std::string transform_description = "",
                          std::string transform_name        = "")
        {
            validators_.insert(validators_.begin(),
                               Validator(
                                   [func](std::string &val)
                                   {
                                       val = func(val);
                                       return std::string{};
                                   },
                                   std::move(transform_description),
                                   std::move(transform_name)));

            return this;
        }

        /// Adds a user supplied function to run on each item passed in (communicate though lambda capture)
        Option *each(const std::function<void(std::string)> &func)
        {
            validators_.emplace_back(
                [func](std::string &inout)
                {
                    func(inout);
                    return std::string{};
                },
                std::string{});
            return this;
        }
        /// Get a named Validator
        Validator *get_validator(const std::string &Validator_name = "")
        {
            for(auto &Validator: validators_)
            {
                if(Validator_name == Validator.get_name())
                {
                    return &Validator;
                }
            }
            if((Validator_name.empty()) && (!validators_.empty()))
            {
                return &(validators_.front());
            }
            throw OptionNotFound(std::string{"Validator "} + Validator_name + " Not Found");
        }

        /// Get a Validator by index NOTE: this may not be the order of definition
        Validator *get_validator(int index)
        {
            // This is an signed int so that it is not equivalent to a pointer.
            if(index >= 0 && index < static_cast<int>(validators_.size()))
            {
                return &(validators_[static_cast<decltype(validators_)::size_type>(index)]);
            }
            throw OptionNotFound("Validator index is not valid");
        }

        /// Sets required options
        Option *needs(Option *opt)
        {
            if(opt != this)
            {
                needs_.insert(opt);
            }
            return this;
        }

        /// Can find a string if needed
        template <typename T = App>
        Option *needs(std::string opt_name)
        {
            auto opt = static_cast<T *>(parent_)->get_option_no_throw(opt_name);
            if(opt == nullptr)
            {
                throw IncorrectConstruction::MissingOption(opt_name);
            }
            return needs(opt);
        }

        /// Any number supported, any mix of string and Opt
        template <typename A, typename B, typename... ARG>
        Option *needs(A opt, B opt1, ARG... args)
        {
            needs(opt);
            return needs(opt1, args...);
        }

        /// Remove needs link from an option. Returns true if the option really was in the needs list.
        bool remove_needs(Option *opt)
        {
            auto iterator = std::find(std::begin(needs_), std::end(needs_), opt);

            if(iterator == std::end(needs_))
            {
                return false;
            }
            needs_.erase(iterator);
            return true;
        }

        /// Sets excluded options
        Option *excludes(Option *opt)
        {
            if(opt == this)
            {
                throw(IncorrectConstruction("and option cannot exclude itself"));
            }
            excludes_.insert(opt);

            // Help text should be symmetric - excluding a should exclude b
            opt->excludes_.insert(this);

            // Ignoring the insert return value, excluding twice is now allowed.
            // (Mostly to allow both directions to be excluded by user, even though the library does it for you.)

            return this;
        }

        /// Can find a string if needed
        template <typename T = App>
        Option *excludes(std::string opt_name)
        {
            auto opt = static_cast<T *>(parent_)->get_option_no_throw(opt_name);
            if(opt == nullptr)
            {
                throw IncorrectConstruction::MissingOption(opt_name);
            }
            return excludes(opt);
        }

        /// Any number supported, any mix of string and Opt
        template <typename A, typename B, typename... ARG>
        Option *excludes(A opt, B opt1, ARG... args)
        {
            excludes(opt);
            return excludes(opt1, args...);
        }

        /// Remove needs link from an option. Returns true if the option really was in the needs list.
        bool remove_excludes(Option *opt)
        {
            auto iterator = std::find(std::begin(excludes_), std::end(excludes_), opt);

            if(iterator == std::end(excludes_))
            {
                return false;
            }
            excludes_.erase(iterator);
            return true;
        }

        /// Sets environment variable to read if no option given
        Option *envname(std::string name)
        {
            envname_ = std::move(name);
            return this;
        }

        /// Ignore case
        ///
        /// The template hides the fact that we don't have the definition of App yet.
        /// You are never expected to add an argument to the template here.
        template <typename T = App>
        Option *ignore_case(bool value = true)
        {
            if(!ignore_case_ && value)
            {
                ignore_case_ = value;
                auto *parent = static_cast<T *>(parent_);
                for(const Option_p &opt: parent->options_)
                {
                    if(opt.get() == this)
                    {
                        continue;
                    }
                    auto &omatch = opt->matching_name(*this);
                    if(!omatch.empty())
                    {
                        ignore_case_ = false;
                        throw OptionAlreadyAdded("adding ignore case caused a name conflict with " + omatch);
                    }
                }
            }
            else
            {
                ignore_case_ = value;
            }
            return this;
        }

        /// Ignore underscores in the option names
        ///
        /// The template hides the fact that we don't have the definition of App yet.
        /// You are never expected to add an argument to the template here.
        template <typename T = App>
        Option *ignore_underscore(bool value = true)
        {
            if(!ignore_underscore_ && value)
            {
                ignore_underscore_ = value;
                auto *parent       = static_cast<T *>(parent_);
                for(const Option_p &opt: parent->options_)
                {
                    if(opt.get() == this)
                    {
                        continue;
                    }
                    auto &omatch = opt->matching_name(*this);
                    if(!omatch.empty())
                    {
                        ignore_underscore_ = false;
                        throw OptionAlreadyAdded("adding ignore underscore caused a name conflict with " + omatch);
                    }
                }
            }
            else
            {
                ignore_underscore_ = value;
            }
            return this;
        }

        /// Take the last argument if given multiple times (or another policy)
        Option *multi_option_policy(MultiOptionPolicy value = MultiOptionPolicy::Throw)
        {
            if(value != multi_option_policy_)
            {
                if(multi_option_policy_ == MultiOptionPolicy::Throw &&
                   expected_max_ == detail::expected_max_vector_size && expected_min_ > 1)
                {  // this bizarre condition is to maintain backwards compatibility
                    // with the previous behavior of expected_ with vectors
                    expected_max_ = expected_min_;
                }
                multi_option_policy_  = value;
                current_option_state_ = option_state::parsing;
            }
            return this;
        }

        /// Disable flag overrides values, e.g. --flag=<value> is not allowed
        Option *disable_flag_override(bool value = true)
        {
            disable_flag_override_ = value;
            return this;
        }
        ///@}
        /// @name Accessors
        ///@{

        /// The number of arguments the option expects
        int get_type_size() const
        {
            return type_size_min_;
        }

        /// The minimum number of arguments the option expects
        int get_type_size_min() const
        {
            return type_size_min_;
        }
        /// The maximum number of arguments the option expects
        int get_type_size_max() const
        {
            return type_size_max_;
        }

        /// The number of arguments the option expects
        int get_inject_separator() const
        {
            return inject_separator_;
        }

        /// The environment variable associated to this value
        std::string get_envname() const
        {
            return envname_;
        }

        /// The set of options needed
        std::set<Option *> get_needs() const
        {
            return needs_;
        }

        /// The set of options excluded
        std::set<Option *> get_excludes() const
        {
            return excludes_;
        }

        /// The default value (for help printing)
        std::string get_default_str() const
        {
            return default_str_;
        }

        /// Get the callback function
        callback_t get_callback() const
        {
            return callback_;
        }

        /// Get the long names
        const std::vector<std::string> &get_lnames() const
        {
            return lnames_;
        }

        /// Get the short names
        const std::vector<std::string> &get_snames() const
        {
            return snames_;
        }

        /// Get the flag names with specified default values
        const std::vector<std::string> &get_fnames() const
        {
            return fnames_;
        }
        /// Get a single name for the option, first of lname, pname, sname, envname
        const std::string &get_single_name() const
        {
            if(!lnames_.empty())
            {
                return lnames_[0];
            }
            if(!pname_.empty())
            {
                return pname_;
            }
            if(!snames_.empty())
            {
                return snames_[0];
            }
            return envname_;
        }
        /// The number of times the option expects to be included
        int get_expected() const
        {
            return expected_min_;
        }

        /// The number of times the option expects to be included
        int get_expected_min() const
        {
            return expected_min_;
        }
        /// The max number of times the option expects to be included
        int get_expected_max() const
        {
            return expected_max_;
        }

        /// The total min number of expected  string values to be used
        int get_items_expected_min() const
        {
            return type_size_min_ * expected_min_;
        }

        /// Get the maximum number of items expected to be returned and used for the callback
        int get_items_expected_max() const
        {
            int t = type_size_max_;
            return detail::checked_multiply(t, expected_max_) ? t : detail::expected_max_vector_size;
        }
        /// The total min number of expected  string values to be used
        int get_items_expected() const
        {
            return get_items_expected_min();
        }

        /// True if the argument can be given directly
        bool get_positional() const
        {
            return pname_.length() > 0;
        }

        /// True if option has at least one non-positional name
        bool nonpositional() const
        {
            return (snames_.size() + lnames_.size()) > 0;
        }

        /// True if option has description
        bool has_description() const
        {
            return description_.length() > 0;
        }

        /// Get the description
        const std::string &get_description() const
        {
            return description_;
        }

        /// Set the description
        Option *description(std::string option_description)
        {
            description_ = std::move(option_description);
            return this;
        }

        Option *option_text(std::string text)
        {
            option_text_ = std::move(text);
            return this;
        }

        const std::string &get_option_text() const
        {
            return option_text_;
        }

        ///@}
        /// @name Help tools
        ///@{

        /// \brief Gets a comma separated list of names.
        /// Will include / prefer the positional name if positional is true.
        /// If all_options is false, pick just the most descriptive name to show.
        /// Use `get_name(true)` to get the positional name (replaces `get_pname`)
        std::string get_name(bool positional  = false,  ///< Show the positional name
                             bool all_options = false   ///< Show every option
        ) const
        {
            if(get_group().empty())
                return {};  // Hidden

            if(all_options)
            {
                std::vector<std::string> name_list;

                /// The all list will never include a positional unless asked or that's the only name.
                if((positional && (!pname_.empty())) || (snames_.empty() && lnames_.empty()))
                {
                    name_list.push_back(pname_);
                }
                if((get_items_expected() == 0) && (!fnames_.empty()))
                {
                    for(const std::string &sname: snames_)
                    {
                        name_list.push_back("-" + sname);
                        if(check_fname(sname))
                        {
                            name_list.back() += "{" + get_flag_value(sname, "") + "}";
                        }
                    }

                    for(const std::string &lname: lnames_)
                    {
                        name_list.push_back("--" + lname);
                        if(check_fname(lname))
                        {
                            name_list.back() += "{" + get_flag_value(lname, "") + "}";
                        }
                    }
                }
                else
                {
                    for(const std::string &sname: snames_)
                        name_list.push_back("-" + sname);

                    for(const std::string &lname: lnames_)
                        name_list.push_back("--" + lname);
                }

                return detail::join(name_list);
            }

            // This returns the positional name no matter what
            if(positional)
                return pname_;

            // Prefer long name
            if(!lnames_.empty())
                return std::string(2, '-') + lnames_[0];

            // Or short name if no long name
            if(!snames_.empty())
                return std::string(1, '-') + snames_[0];

            // If positional is the only name, it's okay to use that
            return pname_;
        }

        ///@}
        /// @name Parser tools
        ///@{

        /// Process the callback
        void run_callback()
        {
            if(current_option_state_ == option_state::parsing)
            {
                _validate_results(results_);
                current_option_state_ = option_state::validated;
            }

            if(current_option_state_ < option_state::reduced)
            {
                _reduce_results(proc_results_, results_);
                current_option_state_ = option_state::reduced;
            }
            if(current_option_state_ >= option_state::reduced)
            {
                current_option_state_ = option_state::callback_run;
                if(!(callback_))
                {
                    return;
                }
                const results_t &send_results = proc_results_.empty() ? results_ : proc_results_;
                bool local_result             = callback_(send_results);

                if(!local_result)
                    throw ConversionError(get_name(), results_);
            }
        }

        /// If options share any of the same names, find it
        const std::string &matching_name(const Option &other) const
        {
            static const std::string estring;
            for(const std::string &sname: snames_)
                if(other.check_sname(sname))
                    return sname;
            for(const std::string &lname: lnames_)
                if(other.check_lname(lname))
                    return lname;

            if(ignore_case_ || ignore_underscore_)
            {  // We need to do the inverse, in case we are ignore_case or ignore underscore
                for(const std::string &sname: other.snames_)
                    if(check_sname(sname))
                        return sname;
                for(const std::string &lname: other.lnames_)
                    if(check_lname(lname))
                        return lname;
            }
            return estring;
        }
        /// If options share any of the same names, they are equal (not counting positional)
        bool operator==(const Option &other) const
        {
            return !matching_name(other).empty();
        }

        /// Check a name. Requires "-" or "--" for short / long, supports positional name
        bool check_name(const std::string &name) const
        {
            if(name.length() > 2 && name[0] == '-' && name[1] == '-')
                return check_lname(name.substr(2));
            if(name.length() > 1 && name.front() == '-')
                return check_sname(name.substr(1));
            if(!pname_.empty())
            {
                std::string local_pname = pname_;
                std::string local_name  = name;
                if(ignore_underscore_)
                {
                    local_pname = detail::remove_underscore(local_pname);
                    local_name  = detail::remove_underscore(local_name);
                }
                if(ignore_case_)
                {
                    local_pname = detail::to_lower(local_pname);
                    local_name  = detail::to_lower(local_name);
                }
                if(local_name == local_pname)
                {
                    return true;
                }
            }

            if(!envname_.empty())
            {
                // this needs to be the original since envname_ shouldn't match on case insensitivity
                return (name == envname_);
            }
            return false;
        }

        /// Requires "-" to be removed from string
        bool check_sname(std::string name) const
        {
            return (detail::find_member(std::move(name), snames_, ignore_case_) >= 0);
        }

        /// Requires "--" to be removed from string
        bool check_lname(std::string name) const
        {
            return (detail::find_member(std::move(name), lnames_, ignore_case_, ignore_underscore_) >= 0);
        }

        /// Requires "--" to be removed from string
        bool check_fname(std::string name) const
        {
            if(fnames_.empty())
            {
                return false;
            }
            return (detail::find_member(std::move(name), fnames_, ignore_case_, ignore_underscore_) >= 0);
        }

        /// Get the value that goes for a flag, nominally gets the default value but allows for overrides if not
        /// disabled
        std::string get_flag_value(const std::string &name, std::string input_value) const
        {
            static const std::string trueString{"true"};
            static const std::string falseString{"false"};
            static const std::string emptyString{"{}"};
            // check for disable flag override_
            if(disable_flag_override_)
            {
                if(!((input_value.empty()) || (input_value == emptyString)))
                {
                    auto default_ind = detail::find_member(name, fnames_, ignore_case_, ignore_underscore_);
                    if(default_ind >= 0)
                    {
                        // We can static cast this to std::size_t because it is more than 0 in this block
                        if(default_flag_values_[static_cast<std::size_t>(default_ind)].second != input_value)
                        {
                            throw(ArgumentMismatch::FlagOverride(name));
                        }
                    }
                    else
                    {
                        if(input_value != trueString)
                        {
                            throw(ArgumentMismatch::FlagOverride(name));
                        }
                    }
                }
            }
            auto ind = detail::find_member(name, fnames_, ignore_case_, ignore_underscore_);
            if((input_value.empty()) || (input_value == emptyString))
            {
                if(flag_like_)
                {
                    return (ind < 0) ? trueString : default_flag_values_[static_cast<std::size_t>(ind)].second;
                }
                else
                {
                    return (ind < 0) ? default_str_ : default_flag_values_[static_cast<std::size_t>(ind)].second;
                }
            }
            if(ind < 0)
            {
                return input_value;
            }
            if(default_flag_values_[static_cast<std::size_t>(ind)].second == falseString)
            {
                try
                {
                    auto val = detail::to_flag_value(input_value);
                    return (val == 1) ? falseString : (val == (-1) ? trueString : std::to_string(-val));
                }
                catch(const std::invalid_argument &)
                {
                    return input_value;
                }
            }
            else
            {
                return input_value;
            }
        }

        /// Puts a result at the end
        Option *add_result(std::string s)
        {
            _add_result(std::move(s), results_);
            current_option_state_ = option_state::parsing;
            return this;
        }

        /// Puts a result at the end and get a count of the number of arguments actually added
        Option *add_result(std::string s, int &results_added)
        {
            results_added         = _add_result(std::move(s), results_);
            current_option_state_ = option_state::parsing;
            return this;
        }

        /// Puts a result at the end
        Option *add_result(std::vector<std::string> s)
        {
            for(auto &str: s)
            {
                _add_result(std::move(str), results_);
            }
            current_option_state_ = option_state::parsing;
            return this;
        }

        /// Get the current complete results set
        const results_t &results() const
        {
            return results_;
        }

        /// Get a copy of the results
        results_t reduced_results() const
        {
            results_t res = proc_results_.empty() ? results_ : proc_results_;
            if(current_option_state_ < option_state::reduced)
            {
                if(current_option_state_ == option_state::parsing)
                {
                    res = results_;
                    _validate_results(res);
                }
                if(!res.empty())
                {
                    results_t extra;
                    _reduce_results(extra, res);
                    if(!extra.empty())
                    {
                        res = std::move(extra);
                    }
                }
            }
            return res;
        }

        /// Get the results as a specified type
        template <typename T>
        void results(T &output) const
        {
            bool retval;
            if(current_option_state_ >= option_state::reduced || (results_.size() == 1 && validators_.empty()))
            {
                const results_t &res = (proc_results_.empty()) ? results_ : proc_results_;
                retval               = detail::lexical_conversion<T, T>(res, output);
            }
            else
            {
                results_t res;
                if(results_.empty())
                {
                    if(!default_str_.empty())
                    {
                        // _add_results takes an rvalue only
                        _add_result(std::string(default_str_), res);
                        _validate_results(res);
                        results_t extra;
                        _reduce_results(extra, res);
                        if(!extra.empty())
                        {
                            res = std::move(extra);
                        }
                    }
                    else
                    {
                        res.emplace_back();
                    }
                }
                else
                {
                    res = reduced_results();
                }
                retval = detail::lexical_conversion<T, T>(res, output);
            }
            if(!retval)
            {
                throw ConversionError(get_name(), results_);
            }
        }

        /// Return the results as the specified type
        template <typename T>
        T as() const
        {
            T output;
            results(output);
            return output;
        }

        /// See if the callback has been run already
        bool get_callback_run() const
        {
            return (current_option_state_ == option_state::callback_run);
        }

        ///@}
        /// @name Custom options
        ///@{

        /// Set the type function to run when displayed on this option
        Option *type_name_fn(std::function<std::string()> typefun)
        {
            type_name_ = std::move(typefun);
            return this;
        }

        /// Set a custom option typestring
        Option *type_name(std::string typeval)
        {
            type_name_fn(
                [typeval]()
                {
                    return typeval;
                });
            return this;
        }

        /// Set a custom option size
        Option *type_size(int option_type_size)
        {
            if(option_type_size < 0)
            {
                // this section is included for backwards compatibility
                type_size_max_ = -option_type_size;
                type_size_min_ = -option_type_size;
                expected_max_  = detail::expected_max_vector_size;
            }
            else
            {
                type_size_max_ = option_type_size;
                if(type_size_max_ < detail::expected_max_vector_size)
                {
                    type_size_min_ = option_type_size;
                }
                else
                {
                    inject_separator_ = true;
                }
                if(type_size_max_ == 0)
                    required_ = false;
            }
            return this;
        }
        /// Set a custom option type size range
        Option *type_size(int option_type_size_min, int option_type_size_max)
        {
            if(option_type_size_min < 0 || option_type_size_max < 0)
            {
                // this section is included for backwards compatibility
                expected_max_        = detail::expected_max_vector_size;
                option_type_size_min = (std::abs)(option_type_size_min);
                option_type_size_max = (std::abs)(option_type_size_max);
            }

            if(option_type_size_min > option_type_size_max)
            {
                type_size_max_ = option_type_size_min;
                type_size_min_ = option_type_size_max;
            }
            else
            {
                type_size_min_ = option_type_size_min;
                type_size_max_ = option_type_size_max;
            }
            if(type_size_max_ == 0)
            {
                required_ = false;
            }
            if(type_size_max_ >= detail::expected_max_vector_size)
            {
                inject_separator_ = true;
            }
            return this;
        }

        /// Set the value of the separator injection flag
        void inject_separator(bool value = true)
        {
            inject_separator_ = value;
        }

        /// Set a capture function for the default. Mostly used by App.
        Option *default_function(const std::function<std::string()> &func)
        {
            default_function_ = func;
            return this;
        }

        /// Capture the default value from the original value (if it can be captured)
        Option *capture_default_str()
        {
            if(default_function_)
            {
                default_str_ = default_function_();
            }
            return this;
        }

        /// Set the default value string representation (does not change the contained value)
        Option *default_str(std::string val)
        {
            default_str_ = std::move(val);
            return this;
        }

        /// Set the default value and validate the results and run the callback if appropriate to set the value into the
        /// bound value only available for types that can be converted to a string
        template <typename X>
        Option *default_val(const X &val)
        {
            std::string val_str   = detail::to_string(val);
            auto old_option_state = current_option_state_;
            results_t old_results{std::move(results_)};
            results_.clear();
            try
            {
                add_result(val_str);
                if(run_callback_for_default_)
                {
                    run_callback();  // run callback sets the state we need to reset it again
                    current_option_state_ = option_state::parsing;
                }
                else
                {
                    _validate_results(results_);
                    current_option_state_ = old_option_state;
                }
            }
            catch(const CLI::Error &)
            {
                // this should be done
                results_              = std::move(old_results);
                current_option_state_ = old_option_state;
                throw;
            }
            results_     = std::move(old_results);
            default_str_ = std::move(val_str);
            return this;
        }

        /// Get the full typename for this option
        std::string get_type_name() const
        {
            std::string full_type_name = type_name_();
            if(!validators_.empty())
            {
                for(auto &Validator: validators_)
                {
                    std::string vtype = Validator.get_description();
                    if(!vtype.empty())
                    {
                        full_type_name += ":" + vtype;
                    }
                }
            }
            return full_type_name;
        }

       private:
        /// Run the results through the Validators
        void _validate_results(results_t &res) const
        {
            // Run the Validators (can change the string)
            if(!validators_.empty())
            {
                if(type_size_max_ > 1)
                {  // in this context index refers to the index in the type
                    int index = 0;
                    if(get_items_expected_max() < static_cast<int>(res.size()) &&
                       multi_option_policy_ == CLI::MultiOptionPolicy::TakeLast)
                    {
                        // create a negative index for the earliest ones
                        index = get_items_expected_max() - static_cast<int>(res.size());
                    }

                    for(std::string &result: res)
                    {
                        if(detail::is_separator(result) && type_size_max_ != type_size_min_ && index >= 0)
                        {
                            index = 0;  // reset index for variable size chunks
                            continue;
                        }
                        auto err_msg = _validate(result, (index >= 0) ? (index % type_size_max_) : index);
                        if(!err_msg.empty())
                            throw ValidationError(get_name(), err_msg);
                        ++index;
                    }
                }
                else
                {
                    int index = 0;
                    if(expected_max_ < static_cast<int>(res.size()) &&
                       multi_option_policy_ == CLI::MultiOptionPolicy::TakeLast)
                    {
                        // create a negative index for the earliest ones
                        index = expected_max_ - static_cast<int>(res.size());
                    }
                    for(std::string &result: res)
                    {
                        auto err_msg = _validate(result, index);
                        ++index;
                        if(!err_msg.empty())
                            throw ValidationError(get_name(), err_msg);
                    }
                }
            }
        }

        /** reduce the results in accordance with the MultiOptionPolicy
        @param[out] res results are assigned to res if there if they are different
        */
        void _reduce_results(results_t &res, const results_t &original) const
        {
            // max num items expected or length of vector, always at least 1
            // Only valid for a trimming policy

            res.clear();
            // Operation depends on the policy setting
            switch(multi_option_policy_)
            {
                case MultiOptionPolicy::TakeAll:
                    break;
                case MultiOptionPolicy::TakeLast:
                {
                    // Allow multi-option sizes (including 0)
                    std::size_t trim_size =
                        std::min<std::size_t>(static_cast<std::size_t>(std::max<int>(get_items_expected_max(), 1)),
                                              original.size());
                    if(original.size() != trim_size)
                    {
                        res.assign(original.end() - static_cast<results_t::difference_type>(trim_size), original.end());
                    }
                }
                break;
                case MultiOptionPolicy::TakeFirst:
                {
                    std::size_t trim_size =
                        std::min<std::size_t>(static_cast<std::size_t>(std::max<int>(get_items_expected_max(), 1)),
                                              original.size());
                    if(original.size() != trim_size)
                    {
                        res.assign(original.begin(),
                                   original.begin() + static_cast<results_t::difference_type>(trim_size));
                    }
                }
                break;
                case MultiOptionPolicy::Join:
                    if(results_.size() > 1)
                    {
                        res.push_back(detail::join(original, std::string(1, (delimiter_ == '\0') ? '\n' : delimiter_)));
                    }
                    break;
                case MultiOptionPolicy::Throw:
                default:
                {
                    auto num_min = static_cast<std::size_t>(get_items_expected_min());
                    auto num_max = static_cast<std::size_t>(get_items_expected_max());
                    if(num_min == 0)
                    {
                        num_min = 1;
                    }
                    if(num_max == 0)
                    {
                        num_max = 1;
                    }
                    if(original.size() < num_min)
                    {
                        throw ArgumentMismatch::AtLeast(get_name(), static_cast<int>(num_min), original.size());
                    }
                    if(original.size() > num_max)
                    {
                        throw ArgumentMismatch::AtMost(get_name(), static_cast<int>(num_max), original.size());
                    }
                    break;
                }
            }
        }

        // Run a result through the Validators
        std::string _validate(std::string &result, int index) const
        {
            std::string err_msg;
            if(result.empty() && expected_min_ == 0)
            {
                // an empty with nothing expected is allowed
                return err_msg;
            }
            for(const auto &vali: validators_)
            {
                auto v = vali.get_application_index();
                if(v == -1 || v == index)
                {
                    try
                    {
                        err_msg = vali(result);
                    }
                    catch(const ValidationError &err)
                    {
                        err_msg = err.what();
                    }
                    if(!err_msg.empty())
                        break;
                }
            }

            return err_msg;
        }

        /// Add a single result to the result set, taking into account delimiters
        int _add_result(std::string &&result, std::vector<std::string> &res) const
        {
            int result_count = 0;
            if(allow_extra_args_ && !result.empty() && result.front() == '[' && result.back() == ']')
            {  // this is now a vector string likely from the default or user entry
                result.pop_back();

                for(auto &var: CLI::detail::split(result.substr(1), ','))
                {
                    if(!var.empty())
                    {
                        result_count += _add_result(std::move(var), res);
                    }
                }
                return result_count;
            }
            if(delimiter_ == '\0')
            {
                res.push_back(std::move(result));
                ++result_count;
            }
            else
            {
                if((result.find_first_of(delimiter_) != std::string::npos))
                {
                    for(const auto &var: CLI::detail::split(result, delimiter_))
                    {
                        if(!var.empty())
                        {
                            res.push_back(var);
                            ++result_count;
                        }
                    }
                }
                else
                {
                    res.push_back(std::move(result));
                    ++result_count;
                }
            }
            return result_count;
        }
    };  // namespace CLI

#ifndef CLI11_PARSE
#    define CLI11_PARSE(app, argc, argv)                                                                               \
        try                                                                                                            \
        {                                                                                                              \
            (app).parse((argc), (argv));                                                                               \
        }                                                                                                              \
        catch(const CLI::ParseError &e)                                                                                \
        {                                                                                                              \
            return (app).exit(e);                                                                                      \
        }
#endif

    namespace detail
    {
        enum class Classifier
        {
            NONE,
            POSITIONAL_MARK,
            SHORT,
            LONG,
            WINDOWS_STYLE,
            SUBCOMMAND,
            SUBCOMMAND_TERMINATOR
        };
        struct AppFriend;
    }  // namespace detail

    namespace FailureMessage
    {
        std::string simple(const App *app, const Error &e);
        std::string help(const App *app, const Error &e);
    }  // namespace FailureMessage

    /// enumeration of modes of how to deal with extras in config files

    enum class config_extras_mode : char
    {
        error = 0,
        ignore,
        capture
    };

    class App;

    using App_p = std::shared_ptr<App>;

    class Option_group;
    /// Creates a command line program, with very few defaults.
    /** To use, create a new `Program()` instance with `argc`, `argv`, and a help description. The templated
     *  add_option methods make it easy to prepare options. Remember to call `.start` before starting your
     * program, so that the options can be evaluated and the help option doesn't accidentally run your program. */
    class App
    {
        friend Option;
        friend detail::AppFriend;

       protected:
        // This library follows the Google style guide for member names ending in underscores

        /// @name Basics
        ///@{

        /// Subcommand name or program name (from parser if name is empty)
        std::string name_{};

        /// Description of the current program/subcommand
        std::string description_{};

        /// If true, allow extra arguments (ie, don't throw an error). INHERITABLE
        bool allow_extras_{false};

        /// If ignore, allow extra arguments in the ini file (ie, don't throw an error). INHERITABLE
        /// if error error on an extra argument, and if capture feed it to the app
        config_extras_mode allow_config_extras_{config_extras_mode::ignore};

        ///  If true, return immediately on an unrecognized option (implies allow_extras) INHERITABLE
        bool prefix_command_{false};

        /// If set to true the name was automatically generated from the command line vs a user set name
        bool has_automatic_name_{false};

        /// If set to true the subcommand is required to be processed and used, ignored for main app
        bool required_{false};

        /// If set to true the subcommand is disabled and cannot be used, ignored for main app
        bool disabled_{false};

        /// Flag indicating that the pre_parse_callback has been triggered
        bool pre_parse_called_{false};

        /// Flag indicating that the callback for the subcommand should be executed immediately on parse completion
        /// which is before help or ini files are processed. INHERITABLE
        bool immediate_callback_{false};

        /// This is a function that runs prior to the start of parsing
        std::function<void(std::size_t)> pre_parse_callback_{};

        /// This is a function that runs when parsing has finished.
        std::function<void()> parse_complete_callback_{};

        /// This is a function that runs when all processing has completed
        std::function<void()> final_callback_{};

        ///@}
        /// @name Options
        ///@{

        /// The default values for options, customizable and changeable INHERITABLE
        OptionDefaults option_defaults_{};

        /// The list of options, stored locally
        std::vector<Option_p> options_{};

        ///@}
        /// @name Help
        ///@{

        /// Footer to put after all options in the help output INHERITABLE
        std::string footer_{};

        /// This is a function that generates a footer to put after all other options in help output
        std::function<std::string()> footer_callback_{};

        /// A pointer to the help flag if there is one INHERITABLE
        Option *help_ptr_{nullptr};

        /// A pointer to the help all flag if there is one INHERITABLE
        Option *help_all_ptr_{nullptr};

        /// A pointer to a version flag if there is one
        Option *version_ptr_{nullptr};

        /// This is the formatter for help printing. Default provided. INHERITABLE (same pointer)
        std::shared_ptr<FormatterBase> formatter_{new Formatter()};

        /// The error message printing function INHERITABLE
        std::function<std::string(const App *, const Error &e)> failure_message_{FailureMessage::simple};

        ///@}
        /// @name Parsing
        ///@{

        using missing_t = std::vector<std::pair<detail::Classifier, std::string>>;

        /// Pair of classifier, string for missing options. (extra detail is removed on returning from parse)
        ///
        /// This is faster and cleaner than storing just a list of strings and reparsing. This may contain the --
        /// separator.
        missing_t missing_{};

        /// This is a list of pointers to options with the original parse order
        std::vector<Option *> parse_order_{};

        /// This is a list of the subcommands collected, in order
        std::vector<App *> parsed_subcommands_{};

        /// this is a list of subcommands that are exclusionary to this one
        std::set<App *> exclude_subcommands_{};

        /// This is a list of options which are exclusionary to this App, if the options were used this subcommand
        /// should not be
        std::set<Option *> exclude_options_{};

        /// this is a list of subcommands or option groups that are required by this one, the list is not mutual,  the
        /// listed subcommands do not require this one
        std::set<App *> need_subcommands_{};

        /// This is a list of options which are required by this app, the list is not mutual, listed options do not need
        /// the subcommand not be
        std::set<Option *> need_options_{};

        ///@}
        /// @name Subcommands
        ///@{

        /// Storage for subcommand list
        std::vector<App_p> subcommands_{};

        /// If true, the program name is not case sensitive INHERITABLE
        bool ignore_case_{false};

        /// If true, the program should ignore underscores INHERITABLE
        bool ignore_underscore_{false};

        /// Allow subcommand fallthrough, so that parent commands can collect commands after subcommand.  INHERITABLE
        bool fallthrough_{false};

        /// Allow '/' for options for Windows like options. Defaults to true on Windows, false otherwise. INHERITABLE
        bool allow_windows_style_options_{
#ifdef _WIN32
            true
#else
            false
#endif
        };
        /// specify that positional arguments come at the end of the argument sequence not inheritable
        bool positionals_at_end_{false};

        enum class startup_mode : char
        {
            stable,
            enabled,
            disabled
        };
        /// specify the startup mode for the app
        /// stable=no change, enabled= startup enabled, disabled=startup disabled
        startup_mode default_startup{startup_mode::stable};

        /// if set to true the subcommand can be triggered via configuration files INHERITABLE
        bool configurable_{false};

        /// If set to true positional options are validated before assigning INHERITABLE
        bool validate_positionals_{false};

        /// indicator that the subcommand is silent and won't show up in subcommands list
        /// This is potentially useful as a modifier subcommand
        bool silent_{false};

        /// Counts the number of times this command/subcommand was parsed
        std::uint32_t parsed_{0U};

        /// Minimum required subcommands (not inheritable!)
        std::size_t require_subcommand_min_{0};

        /// Max number of subcommands allowed (parsing stops after this number). 0 is unlimited INHERITABLE
        std::size_t require_subcommand_max_{0};

        /// Minimum required options (not inheritable!)
        std::size_t require_option_min_{0};

        /// Max number of options allowed. 0 is unlimited (not inheritable)
        std::size_t require_option_max_{0};

        /// A pointer to the parent if this is a subcommand
        App *parent_{nullptr};

        /// The group membership INHERITABLE
        std::string group_{"Subcommands"};

        /// Alias names for the subcommand
        std::vector<std::string> aliases_{};

        ///@}
        /// @name Config
        ///@{

        /// Pointer to the config option
        Option *config_ptr_{nullptr};

        /// This is the formatter for help printing. Default provided. INHERITABLE (same pointer)
        std::shared_ptr<Config> config_formatter_{new ConfigTOML()};

        ///@}

        /// Special private constructor for subcommand
        App(std::string app_description, std::string app_name, App *parent)
            : name_(std::move(app_name))
            , description_(std::move(app_description))
            , parent_(parent)
        {
            // Inherit if not from a nullptr
            if(parent_ != nullptr)
            {
                if(parent_->help_ptr_ != nullptr)
                    set_help_flag(parent_->help_ptr_->get_name(false, true), parent_->help_ptr_->get_description());
                if(parent_->help_all_ptr_ != nullptr)
                    set_help_all_flag(parent_->help_all_ptr_->get_name(false, true),
                                      parent_->help_all_ptr_->get_description());

                /// OptionDefaults
                option_defaults_ = parent_->option_defaults_;

                // INHERITABLE
                failure_message_             = parent_->failure_message_;
                allow_extras_                = parent_->allow_extras_;
                allow_config_extras_         = parent_->allow_config_extras_;
                prefix_command_              = parent_->prefix_command_;
                immediate_callback_          = parent_->immediate_callback_;
                ignore_case_                 = parent_->ignore_case_;
                ignore_underscore_           = parent_->ignore_underscore_;
                fallthrough_                 = parent_->fallthrough_;
                validate_positionals_        = parent_->validate_positionals_;
                configurable_                = parent_->configurable_;
                allow_windows_style_options_ = parent_->allow_windows_style_options_;
                group_                       = parent_->group_;
                footer_                      = parent_->footer_;
                formatter_                   = parent_->formatter_;
                config_formatter_            = parent_->config_formatter_;
                require_subcommand_max_      = parent_->require_subcommand_max_;
            }
        }

       public:
        /// @name Basic
        ///@{

        /// Create a new program. Pass in the same arguments as main(), along with a help string.
        explicit App(std::string app_description = "", std::string app_name = "")
            : App(app_description, app_name, nullptr)
        {
            set_help_flag("-h,--help", "Print this help message and exit");
        }

        App(const App &) = delete;
        App &operator=(const App &) = delete;

        /// virtual destructor
        virtual ~App() = default;

        /// Set a callback for execution when all parsing and processing has completed
        ///
        /// Due to a bug in c++11,
        /// it is not possible to overload on std::function (fixed in c++14
        /// and backported to c++11 on newer compilers). Use capture by reference
        /// to get a pointer to App if needed.
        App *callback(std::function<void()> app_callback)
        {
            if(immediate_callback_)
            {
                parse_complete_callback_ = std::move(app_callback);
            }
            else
            {
                final_callback_ = std::move(app_callback);
            }
            return this;
        }

        /// Set a callback for execution when all parsing and processing has completed
        /// aliased as callback
        App *final_callback(std::function<void()> app_callback)
        {
            final_callback_ = std::move(app_callback);
            return this;
        }

        /// Set a callback to execute when parsing has completed for the app
        ///
        App *parse_complete_callback(std::function<void()> pc_callback)
        {
            parse_complete_callback_ = std::move(pc_callback);
            return this;
        }

        /// Set a callback to execute prior to parsing.
        ///
        App *preparse_callback(std::function<void(std::size_t)> pp_callback)
        {
            pre_parse_callback_ = std::move(pp_callback);
            return this;
        }

        /// Set a name for the app (empty will use parser to set the name)
        App *name(std::string app_name = "")
        {
            if(parent_ != nullptr)
            {
                auto oname = name_;
                name_      = app_name;
                auto &res  = _compare_subcommand_names(*this, *_get_fallthrough_parent());
                if(!res.empty())
                {
                    name_ = oname;
                    throw(OptionAlreadyAdded(app_name + " conflicts with existing subcommand names"));
                }
            }
            else
            {
                name_ = app_name;
            }
            has_automatic_name_ = false;
            return this;
        }

        /// Set an alias for the app
        App *alias(std::string app_name)
        {
            if(!detail::valid_name_string(app_name))
            {
                if(app_name.empty())
                {
                    throw IncorrectConstruction("Empty aliases are not allowed");
                }
                if(!detail::valid_first_char(app_name[0]))
                {
                    throw IncorrectConstruction(
                        "Alias starts with invalid character, allowed characters are [a-zA-z0-9]+'_','?','@' ");
                }
                for(auto c: app_name)
                {
                    if(!detail::valid_later_char(c))
                    {
                        throw IncorrectConstruction(std::string("Alias contains invalid character ('") + c +
                                                    "'), allowed characters are "
                                                    "[a-zA-z0-9]+'_','?','@','.','-' ");
                    }
                }
            }

            if(parent_ != nullptr)
            {
                aliases_.push_back(app_name);
                auto &res = _compare_subcommand_names(*this, *_get_fallthrough_parent());
                if(!res.empty())
                {
                    aliases_.pop_back();
                    throw(OptionAlreadyAdded("alias already matches an existing subcommand: " + app_name));
                }
            }
            else
            {
                aliases_.push_back(app_name);
            }

            return this;
        }

        /// Remove the error when extras are left over on the command line.
        App *allow_extras(bool allow = true)
        {
            allow_extras_ = allow;
            return this;
        }

        /// Remove the error when extras are left over on the command line.
        App *required(bool require = true)
        {
            required_ = require;
            return this;
        }

        /// Disable the subcommand or option group
        App *disabled(bool disable = true)
        {
            disabled_ = disable;
            return this;
        }

        /// silence the subcommand from showing up in the processed list
        App *silent(bool silence = true)
        {
            silent_ = silence;
            return this;
        }

        /// Set the subcommand to be disabled by default, so on clear(), at the start of each parse it is disabled
        App *disabled_by_default(bool disable = true)
        {
            if(disable)
            {
                default_startup = startup_mode::disabled;
            }
            else
            {
                default_startup =
                    (default_startup == startup_mode::enabled) ? startup_mode::enabled : startup_mode::stable;
            }
            return this;
        }

        /// Set the subcommand to be enabled by default, so on clear(), at the start of each parse it is enabled (not
        /// disabled)
        App *enabled_by_default(bool enable = true)
        {
            if(enable)
            {
                default_startup = startup_mode::enabled;
            }
            else
            {
                default_startup =
                    (default_startup == startup_mode::disabled) ? startup_mode::disabled : startup_mode::stable;
            }
            return this;
        }

        /// Set the subcommand callback to be executed immediately on subcommand completion
        App *immediate_callback(bool immediate = true)
        {
            immediate_callback_ = immediate;
            if(immediate_callback_)
            {
                if(final_callback_ && !(parse_complete_callback_))
                {
                    std::swap(final_callback_, parse_complete_callback_);
                }
            }
            else if(!(final_callback_) && parse_complete_callback_)
            {
                std::swap(final_callback_, parse_complete_callback_);
            }
            return this;
        }

        /// Set the subcommand to validate positional arguments before assigning
        App *validate_positionals(bool validate = true)
        {
            validate_positionals_ = validate;
            return this;
        }

        /// ignore extras in config files
        App *allow_config_extras(bool allow = true)
        {
            if(allow)
            {
                allow_config_extras_ = config_extras_mode::capture;
                allow_extras_        = true;
            }
            else
            {
                allow_config_extras_ = config_extras_mode::error;
            }
            return this;
        }

        /// ignore extras in config files
        App *allow_config_extras(config_extras_mode mode)
        {
            allow_config_extras_ = mode;
            return this;
        }

        /// Do not parse anything after the first unrecognized option and return
        App *prefix_command(bool allow = true)
        {
            prefix_command_ = allow;
            return this;
        }

        /// Ignore case. Subcommands inherit value.
        App *ignore_case(bool value = true)
        {
            if(value && !ignore_case_)
            {
                ignore_case_ = true;
                auto *p      = (parent_ != nullptr) ? _get_fallthrough_parent() : this;
                auto &match  = _compare_subcommand_names(*this, *p);
                if(!match.empty())
                {
                    ignore_case_ = false;  // we are throwing so need to be exception invariant
                    throw OptionAlreadyAdded("ignore case would cause subcommand name conflicts: " + match);
                }
            }
            ignore_case_ = value;
            return this;
        }

        /// Allow windows style options, such as `/opt`. First matching short or long name used. Subcommands inherit
        /// value.
        App *allow_windows_style_options(bool value = true)
        {
            allow_windows_style_options_ = value;
            return this;
        }

        /// Specify that the positional arguments are only at the end of the sequence
        App *positionals_at_end(bool value = true)
        {
            positionals_at_end_ = value;
            return this;
        }

        /// Specify that the subcommand can be triggered by a config file
        App *configurable(bool value = true)
        {
            configurable_ = value;
            return this;
        }

        /// Ignore underscore. Subcommands inherit value.
        App *ignore_underscore(bool value = true)
        {
            if(value && !ignore_underscore_)
            {
                ignore_underscore_ = true;
                auto *p            = (parent_ != nullptr) ? _get_fallthrough_parent() : this;
                auto &match        = _compare_subcommand_names(*this, *p);
                if(!match.empty())
                {
                    ignore_underscore_ = false;
                    throw OptionAlreadyAdded("ignore underscore would cause subcommand name conflicts: " + match);
                }
            }
            ignore_underscore_ = value;
            return this;
        }

        /// Set the help formatter
        App *formatter(std::shared_ptr<FormatterBase> fmt)
        {
            formatter_ = fmt;
            return this;
        }

        /// Set the help formatter
        App *formatter_fn(std::function<std::string(const App *, std::string, AppFormatMode)> fmt)
        {
            formatter_ = std::make_shared<FormatterLambda>(fmt);
            return this;
        }

        /// Set the config formatter
        App *config_formatter(std::shared_ptr<Config> fmt)
        {
            config_formatter_ = fmt;
            return this;
        }

        /// Check to see if this subcommand was parsed, true only if received on command line.
        bool parsed() const
        {
            return parsed_ > 0;
        }

        /// Get the OptionDefault object, to set option defaults
        OptionDefaults *option_defaults()
        {
            return &option_defaults_;
        }

        ///@}
        /// @name Adding options
        ///@{

        /// Add an option, will automatically understand the type for common types.
        ///
        /// To use, create a variable with the expected type, and pass it in after the name.
        /// After start is called, you can use count to see if the value was passed, and
        /// the value will be initialized properly. Numbers, vectors, and strings are supported.
        ///
        /// ->required(), ->default, and the validators are options,
        /// The positional options take an optional number of arguments.
        ///
        /// For example,
        ///
        ///     std::string filename;
        ///     program.add_option("filename", filename, "description of filename");
        ///
        Option *add_option(std::string option_name,
                           callback_t option_callback,
                           std::string option_description    = "",
                           bool defaulted                    = false,
                           std::function<std::string()> func = {})
        {
            Option myopt{option_name, option_description, option_callback, this};

            if(std::find_if(std::begin(options_),
                            std::end(options_),
                            [&myopt](const Option_p &v)
                            {
                                return *v == myopt;
                            }) == std::end(options_))
            {
                options_.emplace_back();
                Option_p &option = options_.back();
                option.reset(new Option(option_name, option_description, option_callback, this));

                // Set the default string capture function
                option->default_function(func);

                // For compatibility with CLI11 1.7 and before, capture the default string here
                if(defaulted)
                    option->capture_default_str();

                // Transfer defaults to the new option
                option_defaults_.copy_to(option.get());

                // Don't bother to capture if we already did
                if(!defaulted && option->get_always_capture_default())
                    option->capture_default_str();

                return option.get();
            }
            // we know something matches now find what it is so we can produce more error information
            for(auto &opt: options_)
            {
                auto &matchname = opt->matching_name(myopt);
                if(!matchname.empty())
                {
                    throw(OptionAlreadyAdded("added option matched existing option name: " + matchname));
                }
            }
            // this line should not be reached the above loop should trigger the throw
            throw(OptionAlreadyAdded("added option matched existing option name"));  // LCOV_EXCL_LINE
        }

        /// Add option for assigning to a variable
        template <typename AssignTo,
                  typename ConvertTo                                             = AssignTo,
                  enable_if_t<!std::is_const<ConvertTo>::value, detail::enabler> = detail::dummy>
        Option *add_option(std::string option_name,
                           AssignTo &variable,  ///< The variable to set
                           std::string option_description = "")
        {
            auto fun = [&variable](const CLI::results_t &res) {  // comment for spacing
                return detail::lexical_conversion<AssignTo, ConvertTo>(res, variable);
            };

            Option *opt = add_option(option_name,
                                     fun,
                                     option_description,
                                     false,
                                     [&variable]()
                                     {
                                         return CLI::detail::checked_to_string<AssignTo, ConvertTo>(variable);
                                     });
            opt->type_name(detail::type_name<ConvertTo>());
            // these must be actual lvalues since (std::max) sometimes is defined in terms of references and references
            // to structs used in the evaluation can be temporary so that would cause issues.
            auto Tcount  = detail::type_count<AssignTo>::value;
            auto XCcount = detail::type_count<ConvertTo>::value;
            opt->type_size(detail::type_count_min<ConvertTo>::value, (std::max)(Tcount, XCcount));
            opt->expected(detail::expected_count<ConvertTo>::value);
            opt->run_callback_for_default();
            return opt;
        }

        /// Add option for assigning to a variable
        template <typename AssignTo, enable_if_t<!std::is_const<AssignTo>::value, detail::enabler> = detail::dummy>
        Option *add_option_no_stream(std::string option_name,
                                     AssignTo &variable,  ///< The variable to set
                                     std::string option_description = "")
        {
            auto fun = [&variable](const CLI::results_t &res) {  // comment for spacing
                return detail::lexical_conversion<AssignTo, AssignTo>(res, variable);
            };

            Option *opt = add_option(option_name,
                                     fun,
                                     option_description,
                                     false,
                                     []()
                                     {
                                         return std::string{};
                                     });
            opt->type_name(detail::type_name<AssignTo>());
            opt->type_size(detail::type_count_min<AssignTo>::value, detail::type_count<AssignTo>::value);
            opt->expected(detail::expected_count<AssignTo>::value);
            opt->run_callback_for_default();
            return opt;
        }

        /// Add option for a callback of a specific type
        template <typename ArgType>
        Option *add_option_function(std::string option_name,
                                    const std::function<void(const ArgType &)> &func,  ///< the callback to execute
                                    std::string option_description = "")
        {
            auto fun = [func](const CLI::results_t &res)
            {
                ArgType variable;
                bool result = detail::lexical_conversion<ArgType, ArgType>(res, variable);
                if(result)
                {
                    func(variable);
                }
                return result;
            };

            Option *opt = add_option(option_name, std::move(fun), option_description, false);
            opt->type_name(detail::type_name<ArgType>());
            opt->type_size(detail::type_count_min<ArgType>::value, detail::type_count<ArgType>::value);
            opt->expected(detail::expected_count<ArgType>::value);
            return opt;
        }

        /// Add option with no description or variable assignment
        Option *add_option(std::string option_name)
        {
            return add_option(option_name, CLI::callback_t{}, std::string{}, false);
        }

        /// Add option with description but with no variable assignment or callback
        template <typename T,
                  enable_if_t<std::is_const<T>::value && std::is_constructible<std::string, T>::value,
                              detail::enabler> = detail::dummy>
        Option *add_option(std::string option_name, T &option_description)
        {
            return add_option(option_name, CLI::callback_t(), option_description, false);
        }

        /// Set a help flag, replace the existing one if present
        Option *set_help_flag(std::string flag_name = "", const std::string &help_description = "")
        {
            // take flag_description by const reference otherwise add_flag tries to assign to help_description
            if(help_ptr_ != nullptr)
            {
                remove_option(help_ptr_);
                help_ptr_ = nullptr;
            }

            // Empty name will simply remove the help flag
            if(!flag_name.empty())
            {
                help_ptr_ = add_flag(flag_name, help_description);
                help_ptr_->configurable(false);
            }

            return help_ptr_;
        }

        /// Set a help all flag, replaced the existing one if present
        Option *set_help_all_flag(std::string help_name = "", const std::string &help_description = "")
        {
            // take flag_description by const reference otherwise add_flag tries to assign to flag_description
            if(help_all_ptr_ != nullptr)
            {
                remove_option(help_all_ptr_);
                help_all_ptr_ = nullptr;
            }

            // Empty name will simply remove the help all flag
            if(!help_name.empty())
            {
                help_all_ptr_ = add_flag(help_name, help_description);
                help_all_ptr_->configurable(false);
            }

            return help_all_ptr_;
        }

        /// Set a version flag and version display string, replace the existing one if present
        Option *set_version_flag(std::string flag_name            = "",
                                 const std::string &versionString = "",
                                 const std::string &version_help  = "Display program version information and exit")
        {
            // take flag_description by const reference otherwise add_flag tries to assign to version_description
            if(version_ptr_ != nullptr)
            {
                remove_option(version_ptr_);
                version_ptr_ = nullptr;
            }

            // Empty name will simply remove the version flag
            if(!flag_name.empty())
            {
                version_ptr_ = add_flag_callback(
                    flag_name,
                    [versionString]()
                    {
                        throw(CLI::CallForVersion(versionString, 0));
                    },
                    version_help);
                version_ptr_->configurable(false);
            }

            return version_ptr_;
        }
        /// Generate the version string through a callback function
        Option *set_version_flag(std::string flag_name,
                                 std::function<std::string()> vfunc,
                                 const std::string &version_help = "Display program version information and exit")
        {
            if(version_ptr_ != nullptr)
            {
                remove_option(version_ptr_);
                version_ptr_ = nullptr;
            }

            // Empty name will simply remove the version flag
            if(!flag_name.empty())
            {
                version_ptr_ = add_flag_callback(
                    flag_name,
                    [vfunc]()
                    {
                        throw(CLI::CallForVersion(vfunc(), 0));
                    },
                    version_help);
                version_ptr_->configurable(false);
            }

            return version_ptr_;
        }

       private:
        /// Internal function for adding a flag
        Option *_add_flag_internal(std::string flag_name, CLI::callback_t fun, std::string flag_description)
        {
            Option *opt;
            if(detail::has_default_flag_values(flag_name))
            {
                // check for default values and if it has them
                auto flag_defaults = detail::get_default_flag_values(flag_name);
                detail::remove_default_flag_values(flag_name);
                opt = add_option(std::move(flag_name), std::move(fun), std::move(flag_description), false);
                for(const auto &fname: flag_defaults)
                    opt->fnames_.push_back(fname.first);
                opt->default_flag_values_ = std::move(flag_defaults);
            }
            else
            {
                opt = add_option(std::move(flag_name), std::move(fun), std::move(flag_description), false);
            }
            // flags cannot have positional values
            if(opt->get_positional())
            {
                auto pos_name = opt->get_name(true);
                remove_option(opt);
                throw IncorrectConstruction::PositionalFlag(pos_name);
            }
            opt->multi_option_policy(MultiOptionPolicy::TakeLast);
            opt->expected(0);
            opt->required(false);
            return opt;
        }

       public:
        /// Add a flag with no description or variable assignment
        Option *add_flag(std::string flag_name)
        {
            return _add_flag_internal(flag_name, CLI::callback_t(), std::string{});
        }

        /// Add flag with description but with no variable assignment or callback
        /// takes a constant string,  if a variable string is passed that variable will be assigned the results from the
        /// flag
        template <typename T,
                  enable_if_t<std::is_const<T>::value && std::is_constructible<std::string, T>::value,
                              detail::enabler> = detail::dummy>
        Option *add_flag(std::string flag_name, T &flag_description)
        {
            return _add_flag_internal(flag_name, CLI::callback_t(), flag_description);
        }

        /// Add option for flag with integer result - defaults to allowing multiple passings, but can be forced to one
        /// if `multi_option_policy(CLI::MultiOptionPolicy::Throw)` is used.
        template <typename T,
                  enable_if_t<std::is_constructible<T, std::int64_t>::value && !is_bool<T>::value, detail::enabler> =
                      detail::dummy>
        Option *add_flag(std::string flag_name,
                         T &flag_count,  ///< A variable holding the count
                         std::string flag_description = "")
        {
            flag_count          = 0;
            CLI::callback_t fun = [&flag_count](const CLI::results_t &res)
            {
                try
                {
                    detail::sum_flag_vector(res, flag_count);
                }
                catch(const std::invalid_argument &)
                {
                    return false;
                }
                return true;
            };
            return _add_flag_internal(flag_name, std::move(fun), std::move(flag_description))
                ->multi_option_policy(MultiOptionPolicy::TakeAll);
        }

        /// Other type version accepts all other types that are not vectors such as bool, enum, string or other classes
        /// that can be converted from a string
        template <typename T,
                  enable_if_t<!detail::is_mutable_container<T>::value && !std::is_const<T>::value &&
                                  (!std::is_constructible<T, std::int64_t>::value || is_bool<T>::value) &&
                                  !std::is_constructible<std::function<void(int)>, T>::value,
                              detail::enabler> = detail::dummy>
        Option *add_flag(std::string flag_name,
                         T &flag_result,  ///< A variable holding true if passed
                         std::string flag_description = "")
        {
            CLI::callback_t fun = [&flag_result](const CLI::results_t &res)
            {
                return CLI::detail::lexical_cast(res[0], flag_result);
            };
            return _add_flag_internal(flag_name, std::move(fun), std::move(flag_description))
                ->run_callback_for_default();
        }

        /// Vector version to capture multiple flags.
        template <typename T,
                  enable_if_t<!std::is_assignable<std::function<void(std::int64_t)> &, T>::value, detail::enabler> =
                      detail::dummy>
        Option *add_flag(std::string flag_name,
                         std::vector<T> &flag_results,  ///< A vector of values with the flag results
                         std::string flag_description = "")
        {
            CLI::callback_t fun = [&flag_results](const CLI::results_t &res)
            {
                bool retval = true;
                for(const auto &elem: res)
                {
                    flag_results.emplace_back();
                    retval &= detail::lexical_cast(elem, flag_results.back());
                }
                return retval;
            };
            return _add_flag_internal(flag_name, std::move(fun), std::move(flag_description))
                ->multi_option_policy(MultiOptionPolicy::TakeAll)
                ->run_callback_for_default();
        }

        /// Add option for callback that is triggered with a true flag and takes no arguments
        Option *add_flag_callback(std::string flag_name,
                                  std::function<void(void)> function,  ///< A function to call, void(void)
                                  std::string flag_description = "")
        {
            CLI::callback_t fun = [function](const CLI::results_t &res)
            {
                bool trigger{false};
                auto result = CLI::detail::lexical_cast(res[0], trigger);
                if(result && trigger)
                {
                    function();
                }
                return result;
            };
            return _add_flag_internal(flag_name, std::move(fun), std::move(flag_description));
        }

        /// Add option for callback with an integer value
        Option *add_flag_function(std::string flag_name,
                                  std::function<void(std::int64_t)> function,  ///< A function to call, void(int)
                                  std::string flag_description = "")
        {
            CLI::callback_t fun = [function](const CLI::results_t &res)
            {
                std::int64_t flag_count = 0;
                detail::sum_flag_vector(res, flag_count);
                function(flag_count);
                return true;
            };
            return _add_flag_internal(flag_name, std::move(fun), std::move(flag_description))
                ->multi_option_policy(MultiOptionPolicy::TakeAll);
        }

#ifdef CLI11_CPP14
        /// Add option for callback (C++14 or better only)
        Option *add_flag(std::string flag_name,
                         std::function<void(std::int64_t)> function,  ///< A function to call, void(std::int64_t)
                         std::string flag_description = "")
        {
            return add_flag_function(std::move(flag_name), std::move(function), std::move(flag_description));
        }
#endif

        /// Set a configuration ini file option, or clear it if no name passed
        Option *set_config(std::string option_name         = "",
                           std::string default_filename    = "",
                           const std::string &help_message = "Read an ini file",
                           bool config_required            = false)
        {
            // Remove existing config if present
            if(config_ptr_ != nullptr)
            {
                remove_option(config_ptr_);
                config_ptr_ = nullptr;  // need to remove the config_ptr completely
            }

            // Only add config if option passed
            if(!option_name.empty())
            {
                config_ptr_ = add_option(option_name, help_message);
                if(config_required)
                {
                    config_ptr_->required();
                }
                if(!default_filename.empty())
                {
                    config_ptr_->default_str(std::move(default_filename));
                }
                config_ptr_->configurable(false);
            }

            return config_ptr_;
        }

        /// Removes an option from the App. Takes an option pointer. Returns true if found and removed.
        bool remove_option(Option *opt)
        {
            // Make sure no links exist
            for(Option_p &op: options_)
            {
                op->remove_needs(opt);
                op->remove_excludes(opt);
            }

            if(help_ptr_ == opt)
                help_ptr_ = nullptr;
            if(help_all_ptr_ == opt)
                help_all_ptr_ = nullptr;

            auto iterator = std::find_if(std::begin(options_),
                                         std::end(options_),
                                         [opt](const Option_p &v)
                                         {
                                             return v.get() == opt;
                                         });
            if(iterator != std::end(options_))
            {
                options_.erase(iterator);
                return true;
            }
            return false;
        }

        /// creates an option group as part of the given app
        template <typename T = Option_group>
        T *add_option_group(std::string group_name, std::string group_description = "")
        {
            auto option_group = std::make_shared<T>(std::move(group_description), group_name, this);
            auto ptr          = option_group.get();
            // move to App_p for overload resolution on older gcc versions
            App_p app_ptr = std::dynamic_pointer_cast<App>(option_group);
            add_subcommand(std::move(app_ptr));
            return ptr;
        }

        ///@}
        /// @name Subcommands
        ///@{

        /// Add a subcommand. Inherits INHERITABLE and OptionDefaults, and help flag
        App *add_subcommand(std::string subcommand_name = "", std::string subcommand_description = "")
        {
            if(!subcommand_name.empty() && !detail::valid_name_string(subcommand_name))
            {
                if(!detail::valid_first_char(subcommand_name[0]))
                {
                    throw IncorrectConstruction("Subcommand name starts with invalid character, allowed characters are "
                                                "[a-zA-z0-9]+'_','?','@' ");
                }
                for(auto c: subcommand_name)
                {
                    if(!detail::valid_later_char(c))
                    {
                        throw IncorrectConstruction(std::string("Subcommand name contains invalid character ('") + c +
                                                    "'), allowed characters are "
                                                    "[a-zA-z0-9]+'_','?','@','.','-' ");
                    }
                }
            }
            CLI::App_p subcom = std::shared_ptr<App>(new App(std::move(subcommand_description), subcommand_name, this));
            return add_subcommand(std::move(subcom));
        }

        /// Add a previously created app as a subcommand
        App *add_subcommand(CLI::App_p subcom)
        {
            if(!subcom)
                throw IncorrectConstruction("passed App is not valid");
            auto ckapp  = (name_.empty() && parent_ != nullptr) ? _get_fallthrough_parent() : this;
            auto &mstrg = _compare_subcommand_names(*subcom, *ckapp);
            if(!mstrg.empty())
            {
                throw(OptionAlreadyAdded("subcommand name or alias matches existing subcommand: " + mstrg));
            }
            subcom->parent_ = this;
            subcommands_.push_back(std::move(subcom));
            return subcommands_.back().get();
        }

        /// Removes a subcommand from the App. Takes a subcommand pointer. Returns true if found and removed.
        bool remove_subcommand(App *subcom)
        {
            // Make sure no links exist
            for(App_p &sub: subcommands_)
            {
                sub->remove_excludes(subcom);
                sub->remove_needs(subcom);
            }

            auto iterator = std::find_if(std::begin(subcommands_),
                                         std::end(subcommands_),
                                         [subcom](const App_p &v)
                                         {
                                             return v.get() == subcom;
                                         });
            if(iterator != std::end(subcommands_))
            {
                subcommands_.erase(iterator);
                return true;
            }
            return false;
        }
        /// Check to see if a subcommand is part of this command (doesn't have to be in command line)
        /// returns the first subcommand if passed a nullptr
        App *get_subcommand(const App *subcom) const
        {
            if(subcom == nullptr)
                throw OptionNotFound("nullptr passed");
            for(const App_p &subcomptr: subcommands_)
                if(subcomptr.get() == subcom)
                    return subcomptr.get();
            throw OptionNotFound(subcom->get_name());
        }

        /// Check to see if a subcommand is part of this command (text version)
        App *get_subcommand(std::string subcom) const
        {
            auto subc = _find_subcommand(subcom, false, false);
            if(subc == nullptr)
                throw OptionNotFound(subcom);
            return subc;
        }
        /// Get a pointer to subcommand by index
        App *get_subcommand(int index = 0) const
        {
            if(index >= 0)
            {
                auto uindex = static_cast<unsigned>(index);
                if(uindex < subcommands_.size())
                    return subcommands_[uindex].get();
            }
            throw OptionNotFound(std::to_string(index));
        }

        /// Check to see if a subcommand is part of this command and get a shared_ptr to it
        CLI::App_p get_subcommand_ptr(App *subcom) const
        {
            if(subcom == nullptr)
                throw OptionNotFound("nullptr passed");
            for(const App_p &subcomptr: subcommands_)
                if(subcomptr.get() == subcom)
                    return subcomptr;
            throw OptionNotFound(subcom->get_name());
        }

        /// Check to see if a subcommand is part of this command (text version)
        CLI::App_p get_subcommand_ptr(std::string subcom) const
        {
            for(const App_p &subcomptr: subcommands_)
                if(subcomptr->check_name(subcom))
                    return subcomptr;
            throw OptionNotFound(subcom);
        }

        /// Get an owning pointer to subcommand by index
        CLI::App_p get_subcommand_ptr(int index = 0) const
        {
            if(index >= 0)
            {
                auto uindex = static_cast<unsigned>(index);
                if(uindex < subcommands_.size())
                    return subcommands_[uindex];
            }
            throw OptionNotFound(std::to_string(index));
        }

        /// Check to see if an option group is part of this App
        App *get_option_group(std::string group_name) const
        {
            for(const App_p &app: subcommands_)
            {
                if(app->name_.empty() && app->group_ == group_name)
                {
                    return app.get();
                }
            }
            throw OptionNotFound(group_name);
        }

        /// No argument version of count counts the number of times this subcommand was
        /// passed in. The main app will return 1. Unnamed subcommands will also return 1 unless
        /// otherwise modified in a callback
        std::size_t count() const
        {
            return parsed_;
        }

        /// Get a count of all the arguments processed in options and subcommands, this excludes arguments which were
        /// treated as extras.
        std::size_t count_all() const
        {
            std::size_t cnt{0};
            for(auto &opt: options_)
            {
                cnt += opt->count();
            }
            for(auto &sub: subcommands_)
            {
                cnt += sub->count_all();
            }
            if(!get_name().empty())
            {  // for named subcommands add the number of times the subcommand was called
                cnt += parsed_;
            }
            return cnt;
        }

        /// Changes the group membership
        App *group(std::string group_name)
        {
            group_ = group_name;
            return this;
        }

        /// The argumentless form of require subcommand requires 1 or more subcommands
        App *require_subcommand()
        {
            require_subcommand_min_ = 1;
            require_subcommand_max_ = 0;
            return this;
        }

        /// Require a subcommand to be given (does not affect help call)
        /// The number required can be given. Negative values indicate maximum
        /// number allowed (0 for any number). Max number inheritable.
        App *require_subcommand(int value)
        {
            if(value < 0)
            {
                require_subcommand_min_ = 0;
                require_subcommand_max_ = static_cast<std::size_t>(-value);
            }
            else
            {
                require_subcommand_min_ = static_cast<std::size_t>(value);
                require_subcommand_max_ = static_cast<std::size_t>(value);
            }
            return this;
        }

        /// Explicitly control the number of subcommands required. Setting 0
        /// for the max means unlimited number allowed. Max number inheritable.
        App *require_subcommand(std::size_t min, std::size_t max)
        {
            require_subcommand_min_ = min;
            require_subcommand_max_ = max;
            return this;
        }

        /// The argumentless form of require option requires 1 or more options be used
        App *require_option()
        {
            require_option_min_ = 1;
            require_option_max_ = 0;
            return this;
        }

        /// Require an option to be given (does not affect help call)
        /// The number required can be given. Negative values indicate maximum
        /// number allowed (0 for any number).
        App *require_option(int value)
        {
            if(value < 0)
            {
                require_option_min_ = 0;
                require_option_max_ = static_cast<std::size_t>(-value);
            }
            else
            {
                require_option_min_ = static_cast<std::size_t>(value);
                require_option_max_ = static_cast<std::size_t>(value);
            }
            return this;
        }

        /// Explicitly control the number of options required. Setting 0
        /// for the max means unlimited number allowed. Max number inheritable.
        App *require_option(std::size_t min, std::size_t max)
        {
            require_option_min_ = min;
            require_option_max_ = max;
            return this;
        }

        /// Stop subcommand fallthrough, so that parent commands cannot collect commands after subcommand.
        /// Default from parent, usually set on parent.
        App *fallthrough(bool value = true)
        {
            fallthrough_ = value;
            return this;
        }

        /// Check to see if this subcommand was parsed, true only if received on command line.
        /// This allows the subcommand to be directly checked.
        explicit operator bool() const
        {
            return parsed_ > 0;
        }

        ///@}
        /// @name Extras for subclassing
        ///@{

        /// This allows subclasses to inject code before callbacks but after parse.
        ///
        /// This does not run if any errors or help is thrown.
        virtual void pre_callback() {}

        ///@}
        /// @name Parsing
        ///@{
        //
        /// Reset the parsed data
        void clear()
        {
            parsed_           = 0;
            pre_parse_called_ = false;

            missing_.clear();
            parsed_subcommands_.clear();
            for(const Option_p &opt: options_)
            {
                opt->clear();
            }
            for(const App_p &subc: subcommands_)
            {
                subc->clear();
            }
        }

        /// Parses the command line - throws errors.
        /// This must be called after the options are in but before the rest of the program.
        void parse(int argc, const char *const *argv)
        {
            // If the name is not set, read from command line
            if(name_.empty() || has_automatic_name_)
            {
                has_automatic_name_ = true;
                name_               = argv[0];
            }

            std::vector<std::string> args;
            args.reserve(static_cast<std::size_t>(argc) - 1);
            for(int i = argc - 1; i > 0; i--)
                args.emplace_back(argv[i]);
            parse(std::move(args));
        }

        /// Parse a single string as if it contained command line arguments.
        /// This function splits the string into arguments then calls parse(std::vector<std::string> &)
        /// the function takes an optional boolean argument specifying if the programName is included in the string to
        /// process
        void parse(std::string commandline, bool program_name_included = false)
        {
            if(program_name_included)
            {
                auto nstr = detail::split_program_name(commandline);
                if((name_.empty()) || (has_automatic_name_))
                {
                    has_automatic_name_ = true;
                    name_               = nstr.first;
                }
                commandline = std::move(nstr.second);
            }
            else
            {
                detail::trim(commandline);
            }
            // the next section of code is to deal with quoted arguments after an '=' or ':' for windows like operations
            if(!commandline.empty())
            {
                commandline = detail::find_and_modify(commandline, "=", detail::escape_detect);
                if(allow_windows_style_options_)
                    commandline = detail::find_and_modify(commandline, ":", detail::escape_detect);
            }

            auto args = detail::split_up(std::move(commandline));
            // remove all empty strings
            args.erase(std::remove(args.begin(), args.end(), std::string{}), args.end());
            std::reverse(args.begin(), args.end());

            parse(std::move(args));
        }

        /// The real work is done here. Expects a reversed vector.
        /// Changes the vector to the remaining options.
        void parse(std::vector<std::string> &args)
        {
            // Clear if parsed
            if(parsed_ > 0)
                clear();

            // parsed_ is incremented in commands/subcommands,
            // but placed here to make sure this is cleared when
            // running parse after an error is thrown, even by _validate or _configure.
            parsed_ = 1;
            _validate();
            _configure();
            // set the parent as nullptr as this object should be the top now
            parent_ = nullptr;
            parsed_ = 0;

            _parse(args);
            run_callback();
        }

        /// The real work is done here. Expects a reversed vector.
        void parse(std::vector<std::string> &&args)
        {
            // Clear if parsed
            if(parsed_ > 0)
                clear();

            // parsed_ is incremented in commands/subcommands,
            // but placed here to make sure this is cleared when
            // running parse after an error is thrown, even by _validate or _configure.
            parsed_ = 1;
            _validate();
            _configure();
            // set the parent as nullptr as this object should be the top now
            parent_ = nullptr;
            parsed_ = 0;

            _parse(std::move(args));
            run_callback();
        }

        /// Provide a function to print a help message. The function gets access to the App pointer and error.
        void failure_message(std::function<std::string(const App *, const Error &e)> function)
        {
            failure_message_ = function;
        }

        /// Print a nice error message and return the exit code
        int exit(const Error &e, std::ostream &out = std::cout, std::ostream &err = std::cerr) const
        {
            /// Avoid printing anything if this is a CLI::RuntimeError
            if(e.get_name() == "RuntimeError")
                return e.get_exit_code();

            if(e.get_name() == "CallForHelp")
            {
                out << help();
                return e.get_exit_code();
            }

            if(e.get_name() == "CallForAllHelp")
            {
                out << help("", AppFormatMode::All);
                return e.get_exit_code();
            }

            if(e.get_name() == "CallForVersion")
            {
                out << e.what() << std::endl;
                return e.get_exit_code();
            }

            if(e.get_exit_code() != static_cast<int>(ExitCodes::Success))
            {
                if(failure_message_)
                    err << failure_message_(this, e) << std::flush;
            }

            return e.get_exit_code();
        }

        ///@}
        /// @name Post parsing
        ///@{

        /// Counts the number of times the given option was passed.
        std::size_t count(std::string option_name) const
        {
            return get_option(option_name)->count();
        }

        /// Get a subcommand pointer list to the currently selected subcommands (after parsing by default, in command
        /// line order; use parsed = false to get the original definition list.)
        std::vector<App *> get_subcommands() const
        {
            return parsed_subcommands_;
        }

        /// Get a filtered subcommand pointer list from the original definition list. An empty function will provide all
        /// subcommands (const)
        std::vector<const App *> get_subcommands(const std::function<bool(const App *)> &filter) const
        {
            std::vector<const App *> subcomms(subcommands_.size());
            std::transform(std::begin(subcommands_),
                           std::end(subcommands_),
                           std::begin(subcomms),
                           [](const App_p &v)
                           {
                               return v.get();
                           });

            if(filter)
            {
                subcomms.erase(std::remove_if(std::begin(subcomms),
                                              std::end(subcomms),
                                              [&filter](const App *app)
                                              {
                                                  return !filter(app);
                                              }),
                               std::end(subcomms));
            }

            return subcomms;
        }

        /// Get a filtered subcommand pointer list from the original definition list. An empty function will provide all
        /// subcommands
        std::vector<App *> get_subcommands(const std::function<bool(App *)> &filter)
        {
            std::vector<App *> subcomms(subcommands_.size());
            std::transform(std::begin(subcommands_),
                           std::end(subcommands_),
                           std::begin(subcomms),
                           [](const App_p &v)
                           {
                               return v.get();
                           });

            if(filter)
            {
                subcomms.erase(std::remove_if(std::begin(subcomms),
                                              std::end(subcomms),
                                              [&filter](App *app)
                                              {
                                                  return !filter(app);
                                              }),
                               std::end(subcomms));
            }

            return subcomms;
        }

        /// Check to see if given subcommand was selected
        bool got_subcommand(const App *subcom) const
        {
            // get subcom needed to verify that this was a real subcommand
            return get_subcommand(subcom)->parsed_ > 0;
        }

        /// Check with name instead of pointer to see if subcommand was selected
        bool got_subcommand(std::string subcommand_name) const
        {
            return get_subcommand(subcommand_name)->parsed_ > 0;
        }

        /// Sets excluded options for the subcommand
        App *excludes(Option *opt)
        {
            if(opt == nullptr)
            {
                throw OptionNotFound("nullptr passed");
            }
            exclude_options_.insert(opt);
            return this;
        }

        /// Sets excluded subcommands for the subcommand
        App *excludes(App *app)
        {
            if(app == nullptr)
            {
                throw OptionNotFound("nullptr passed");
            }
            if(app == this)
            {
                throw OptionNotFound("cannot self reference in needs");
            }
            auto res = exclude_subcommands_.insert(app);
            // subcommand exclusion should be symmetric
            if(res.second)
            {
                app->exclude_subcommands_.insert(this);
            }
            return this;
        }

        App *needs(Option *opt)
        {
            if(opt == nullptr)
            {
                throw OptionNotFound("nullptr passed");
            }
            need_options_.insert(opt);
            return this;
        }

        App *needs(App *app)
        {
            if(app == nullptr)
            {
                throw OptionNotFound("nullptr passed");
            }
            if(app == this)
            {
                throw OptionNotFound("cannot self reference in needs");
            }
            need_subcommands_.insert(app);
            return this;
        }

        /// Removes an option from the excludes list of this subcommand
        bool remove_excludes(Option *opt)
        {
            auto iterator = std::find(std::begin(exclude_options_), std::end(exclude_options_), opt);
            if(iterator == std::end(exclude_options_))
            {
                return false;
            }
            exclude_options_.erase(iterator);
            return true;
        }

        /// Removes a subcommand from the excludes list of this subcommand
        bool remove_excludes(App *app)
        {
            auto iterator = std::find(std::begin(exclude_subcommands_), std::end(exclude_subcommands_), app);
            if(iterator == std::end(exclude_subcommands_))
            {
                return false;
            }
            auto other_app = *iterator;
            exclude_subcommands_.erase(iterator);
            other_app->remove_excludes(this);
            return true;
        }

        /// Removes an option from the needs list of this subcommand
        bool remove_needs(Option *opt)
        {
            auto iterator = std::find(std::begin(need_options_), std::end(need_options_), opt);
            if(iterator == std::end(need_options_))
            {
                return false;
            }
            need_options_.erase(iterator);
            return true;
        }

        /// Removes a subcommand from the needs list of this subcommand
        bool remove_needs(App *app)
        {
            auto iterator = std::find(std::begin(need_subcommands_), std::end(need_subcommands_), app);
            if(iterator == std::end(need_subcommands_))
            {
                return false;
            }
            need_subcommands_.erase(iterator);
            return true;
        }

        ///@}
        /// @name Help
        ///@{

        /// Set footer.
        App *footer(std::string footer_string)
        {
            footer_ = std::move(footer_string);
            return this;
        }
        /// Set footer.
        App *footer(std::function<std::string()> footer_function)
        {
            footer_callback_ = std::move(footer_function);
            return this;
        }
        /// Produce a string that could be read in as a config of the current values of the App. Set default_also to
        /// include default arguments. write_descriptions will print a description for the App and for each option.
        std::string config_to_str(bool default_also = false, bool write_description = false) const
        {
            return config_formatter_->to_config(this, default_also, write_description, "");
        }

        /// Makes a help message, using the currently configured formatter
        /// Will only do one subcommand at a time
        std::string help(std::string prev = "", AppFormatMode mode = AppFormatMode::Normal) const
        {
            if(prev.empty())
                prev = get_name();
            else
                prev += " " + get_name();

            // Delegate to subcommand if needed
            auto selected_subcommands = get_subcommands();
            if(!selected_subcommands.empty())
            {
                return selected_subcommands.at(0)->help(prev, mode);
            }
            return formatter_->make_help(this, prev, mode);
        }

        /// Displays a version string
        std::string version() const
        {
            std::string val;
            if(version_ptr_ != nullptr)
            {
                auto rv = version_ptr_->results();
                version_ptr_->clear();
                version_ptr_->add_result("true");
                try
                {
                    version_ptr_->run_callback();
                }
                catch(const CLI::CallForVersion &cfv)
                {
                    val = cfv.what();
                }
                version_ptr_->clear();
                version_ptr_->add_result(rv);
            }
            return val;
        }
        ///@}
        /// @name Getters
        ///@{

        /// Access the formatter
        std::shared_ptr<FormatterBase> get_formatter() const
        {
            return formatter_;
        }

        /// Access the config formatter
        std::shared_ptr<Config> get_config_formatter() const
        {
            return config_formatter_;
        }

        /// Access the config formatter as a configBase pointer
        std::shared_ptr<ConfigBase> get_config_formatter_base() const
        {
            // This is safer as a dynamic_cast if we have RTTI, as Config -> ConfigBase
#if defined(__cpp_rtti) || (defined(__GXX_RTTI) && __GXX_RTTI) || (defined(_HAS_STATIC_RTTI) && (_HAS_STATIC_RTTI == 0))
            return std::dynamic_pointer_cast<ConfigBase>(config_formatter_);
#else
            return std::static_pointer_cast<ConfigBase>(config_formatter_);
#endif
        }

        /// Get the app or subcommand description
        std::string get_description() const
        {
            return description_;
        }

        /// Set the description of the app
        App *description(std::string app_description)
        {
            description_ = std::move(app_description);
            return this;
        }

        /// Get the list of options (user facing function, so returns raw pointers), has optional filter function
        std::vector<const Option *> get_options(const std::function<bool(const Option *)> filter = {}) const
        {
            std::vector<const Option *> options(options_.size());
            std::transform(std::begin(options_),
                           std::end(options_),
                           std::begin(options),
                           [](const Option_p &val)
                           {
                               return val.get();
                           });

            if(filter)
            {
                options.erase(std::remove_if(std::begin(options),
                                             std::end(options),
                                             [&filter](const Option *opt)
                                             {
                                                 return !filter(opt);
                                             }),
                              std::end(options));
            }

            return options;
        }

        /// Non-const version of the above
        std::vector<Option *> get_options(const std::function<bool(Option *)> filter = {})
        {
            std::vector<Option *> options(options_.size());
            std::transform(std::begin(options_),
                           std::end(options_),
                           std::begin(options),
                           [](const Option_p &val)
                           {
                               return val.get();
                           });

            if(filter)
            {
                options.erase(std::remove_if(std::begin(options),
                                             std::end(options),
                                             [&filter](Option *opt)
                                             {
                                                 return !filter(opt);
                                             }),
                              std::end(options));
            }

            return options;
        }

        /// Get an option by name (noexcept non-const version)
        Option *get_option_no_throw(std::string option_name) noexcept
        {
            for(Option_p &opt: options_)
            {
                if(opt->check_name(option_name))
                {
                    return opt.get();
                }
            }
            for(auto &subc: subcommands_)
            {
                // also check down into nameless subcommands
                if(subc->get_name().empty())
                {
                    auto opt = subc->get_option_no_throw(option_name);
                    if(opt != nullptr)
                    {
                        return opt;
                    }
                }
            }
            return nullptr;
        }

        /// Get an option by name (noexcept const version)
        const Option *get_option_no_throw(std::string option_name) const noexcept
        {
            for(const Option_p &opt: options_)
            {
                if(opt->check_name(option_name))
                {
                    return opt.get();
                }
            }
            for(const auto &subc: subcommands_)
            {
                // also check down into nameless subcommands
                if(subc->get_name().empty())
                {
                    auto opt = subc->get_option_no_throw(option_name);
                    if(opt != nullptr)
                    {
                        return opt;
                    }
                }
            }
            return nullptr;
        }

        /// Get an option by name
        const Option *get_option(std::string option_name) const
        {
            auto opt = get_option_no_throw(option_name);
            if(opt == nullptr)
            {
                throw OptionNotFound(option_name);
            }
            return opt;
        }

        /// Get an option by name (non-const version)
        Option *get_option(std::string option_name)
        {
            auto opt = get_option_no_throw(option_name);
            if(opt == nullptr)
            {
                throw OptionNotFound(option_name);
            }
            return opt;
        }

        /// Shortcut bracket operator for getting a pointer to an option
        const Option *operator[](const std::string &option_name) const
        {
            return get_option(option_name);
        }

        /// Shortcut bracket operator for getting a pointer to an option
        const Option *operator[](const char *option_name) const
        {
            return get_option(option_name);
        }

        /// Check the status of ignore_case
        bool get_ignore_case() const
        {
            return ignore_case_;
        }

        /// Check the status of ignore_underscore
        bool get_ignore_underscore() const
        {
            return ignore_underscore_;
        }

        /// Check the status of fallthrough
        bool get_fallthrough() const
        {
            return fallthrough_;
        }

        /// Check the status of the allow windows style options
        bool get_allow_windows_style_options() const
        {
            return allow_windows_style_options_;
        }

        /// Check the status of the allow windows style options
        bool get_positionals_at_end() const
        {
            return positionals_at_end_;
        }

        /// Check the status of the allow windows style options
        bool get_configurable() const
        {
            return configurable_;
        }

        /// Get the group of this subcommand
        const std::string &get_group() const
        {
            return group_;
        }

        /// Generate and return the footer.
        std::string get_footer() const
        {
            return (footer_callback_) ? footer_callback_() + '\n' + footer_ : footer_;
        }

        /// Get the required min subcommand value
        std::size_t get_require_subcommand_min() const
        {
            return require_subcommand_min_;
        }

        /// Get the required max subcommand value
        std::size_t get_require_subcommand_max() const
        {
            return require_subcommand_max_;
        }

        /// Get the required min option value
        std::size_t get_require_option_min() const
        {
            return require_option_min_;
        }

        /// Get the required max option value
        std::size_t get_require_option_max() const
        {
            return require_option_max_;
        }

        /// Get the prefix command status
        bool get_prefix_command() const
        {
            return prefix_command_;
        }

        /// Get the status of allow extras
        bool get_allow_extras() const
        {
            return allow_extras_;
        }

        /// Get the status of required
        bool get_required() const
        {
            return required_;
        }

        /// Get the status of disabled
        bool get_disabled() const
        {
            return disabled_;
        }

        /// Get the status of silence
        bool get_silent() const
        {
            return silent_;
        }

        /// Get the status of disabled
        bool get_immediate_callback() const
        {
            return immediate_callback_;
        }

        /// Get the status of disabled by default
        bool get_disabled_by_default() const
        {
            return (default_startup == startup_mode::disabled);
        }

        /// Get the status of disabled by default
        bool get_enabled_by_default() const
        {
            return (default_startup == startup_mode::enabled);
        }
        /// Get the status of validating positionals
        bool get_validate_positionals() const
        {
            return validate_positionals_;
        }

        /// Get the status of allow extras
        config_extras_mode get_allow_config_extras() const
        {
            return allow_config_extras_;
        }

        /// Get a pointer to the help flag.
        Option *get_help_ptr()
        {
            return help_ptr_;
        }

        /// Get a pointer to the help flag. (const)
        const Option *get_help_ptr() const
        {
            return help_ptr_;
        }

        /// Get a pointer to the help all flag. (const)
        const Option *get_help_all_ptr() const
        {
            return help_all_ptr_;
        }

        /// Get a pointer to the config option.
        Option *get_config_ptr()
        {
            return config_ptr_;
        }

        /// Get a pointer to the config option. (const)
        const Option *get_config_ptr() const
        {
            return config_ptr_;
        }

        /// Get a pointer to the version option.
        Option *get_version_ptr()
        {
            return version_ptr_;
        }

        /// Get a pointer to the version option. (const)
        const Option *get_version_ptr() const
        {
            return version_ptr_;
        }

        /// Get the parent of this subcommand (or nullptr if master app)
        App *get_parent()
        {
            return parent_;
        }

        /// Get the parent of this subcommand (or nullptr if master app) (const version)
        const App *get_parent() const
        {
            return parent_;
        }

        /// Get the name of the current app
        const std::string &get_name() const
        {
            return name_;
        }

        /// Get the aliases of the current app
        const std::vector<std::string> &get_aliases() const
        {
            return aliases_;
        }

        /// clear all the aliases of the current App
        App *clear_aliases()
        {
            aliases_.clear();
            return this;
        }

        /// Get a display name for an app
        std::string get_display_name(bool with_aliases = false) const
        {
            if(name_.empty())
            {
                return std::string("[Option Group: ") + get_group() + "]";
            }
            if(aliases_.empty() || !with_aliases || aliases_.empty())
            {
                return name_;
            }
            std::string dispname = name_;
            for(const auto &lalias: aliases_)
            {
                dispname.push_back(',');
                dispname.push_back(' ');
                dispname.append(lalias);
            }
            return dispname;
        }

        /// Check the name, case insensitive and underscore insensitive if set
        bool check_name(std::string name_to_check) const
        {
            std::string local_name = name_;
            if(ignore_underscore_)
            {
                local_name    = detail::remove_underscore(name_);
                name_to_check = detail::remove_underscore(name_to_check);
            }
            if(ignore_case_)
            {
                local_name    = detail::to_lower(name_);
                name_to_check = detail::to_lower(name_to_check);
            }

            if(local_name == name_to_check)
            {
                return true;
            }
            for(auto les: aliases_)
            {
                if(ignore_underscore_)
                {
                    les = detail::remove_underscore(les);
                }
                if(ignore_case_)
                {
                    les = detail::to_lower(les);
                }
                if(les == name_to_check)
                {
                    return true;
                }
            }
            return false;
        }

        /// Get the groups available directly from this option (in order)
        std::vector<std::string> get_groups() const
        {
            std::vector<std::string> groups;

            for(const Option_p &opt: options_)
            {
                // Add group if it is not already in there
                if(std::find(groups.begin(), groups.end(), opt->get_group()) == groups.end())
                {
                    groups.push_back(opt->get_group());
                }
            }

            return groups;
        }

        /// This gets a vector of pointers with the original parse order
        const std::vector<Option *> &parse_order() const
        {
            return parse_order_;
        }

        /// This returns the missing options from the current subcommand
        std::vector<std::string> remaining(bool recurse = false) const
        {
            std::vector<std::string> miss_list;
            for(const std::pair<detail::Classifier, std::string> &miss: missing_)
            {
                miss_list.push_back(std::get<1>(miss));
            }
            // Get from a subcommand that may allow extras
            if(recurse)
            {
                if(!allow_extras_)
                {
                    for(const auto &sub: subcommands_)
                    {
                        if(sub->name_.empty() && !sub->missing_.empty())
                        {
                            for(const std::pair<detail::Classifier, std::string> &miss: sub->missing_)
                            {
                                miss_list.push_back(std::get<1>(miss));
                            }
                        }
                    }
                }
                // Recurse into subcommands

                for(const App *sub: parsed_subcommands_)
                {
                    std::vector<std::string> output = sub->remaining(recurse);
                    std::copy(std::begin(output), std::end(output), std::back_inserter(miss_list));
                }
            }
            return miss_list;
        }

        /// This returns the missing options in a form ready for processing by another command line program
        std::vector<std::string> remaining_for_passthrough(bool recurse = false) const
        {
            std::vector<std::string> miss_list = remaining(recurse);
            std::reverse(std::begin(miss_list), std::end(miss_list));
            return miss_list;
        }

        /// This returns the number of remaining options, minus the -- separator
        std::size_t remaining_size(bool recurse = false) const
        {
            auto remaining_options =
                static_cast<std::size_t>(std::count_if(std::begin(missing_),
                                                       std::end(missing_),
                                                       [](const std::pair<detail::Classifier, std::string> &val)
                                                       {
                                                           return val.first != detail::Classifier::POSITIONAL_MARK;
                                                       }));

            if(recurse)
            {
                for(const App_p &sub: subcommands_)
                {
                    remaining_options += sub->remaining_size(recurse);
                }
            }
            return remaining_options;
        }

        ///@}

       protected:
        /// Check the options to make sure there are no conflicts.
        ///
        /// Currently checks to see if multiple positionals exist with unlimited args and checks if the min and max
        /// options are feasible
        void _validate() const
        {
            // count the number of positional only args
            auto pcount = std::count_if(std::begin(options_),
                                        std::end(options_),
                                        [](const Option_p &opt)
                                        {
                                            return opt->get_items_expected_max() >= detail::expected_max_vector_size &&
                                                   !opt->nonpositional();
                                        });
            if(pcount > 1)
            {
                auto pcount_req =
                    std::count_if(std::begin(options_),
                                  std::end(options_),
                                  [](const Option_p &opt)
                                  {
                                      return opt->get_items_expected_max() >= detail::expected_max_vector_size &&
                                             !opt->nonpositional() && opt->get_required();
                                  });
                if(pcount - pcount_req > 1)
                {
                    throw InvalidError(name_);
                }
            }

            std::size_t nameless_subs{0};
            for(const App_p &app: subcommands_)
            {
                app->_validate();
                if(app->get_name().empty())
                    ++nameless_subs;
            }

            if(require_option_min_ > 0)
            {
                if(require_option_max_ > 0)
                {
                    if(require_option_max_ < require_option_min_)
                    {
                        throw(InvalidError("Required min options greater than required max options",
                                           ExitCodes::InvalidError));
                    }
                }
                if(require_option_min_ > (options_.size() + nameless_subs))
                {
                    throw(InvalidError("Required min options greater than number of available options",
                                       ExitCodes::InvalidError));
                }
            }
        }

        /// configure subcommands to enable parsing through the current object
        /// set the correct fallthrough and prefix for nameless subcommands and manage the automatic enable or disable
        /// makes sure parent is set correctly
        void _configure()
        {
            if(default_startup == startup_mode::enabled)
            {
                disabled_ = false;
            }
            else if(default_startup == startup_mode::disabled)
            {
                disabled_ = true;
            }
            for(const App_p &app: subcommands_)
            {
                if(app->has_automatic_name_)
                {
                    app->name_.clear();
                }
                if(app->name_.empty())
                {
                    app->fallthrough_    = false;  // make sure fallthrough_ is false to prevent infinite loop
                    app->prefix_command_ = false;
                }
                // make sure the parent is set to be this object in preparation for parse
                app->parent_ = this;
                app->_configure();
            }
        }

        /// Internal function to run (App) callback, bottom up
        void run_callback(bool final_mode = false, bool suppress_final_callback = false)
        {
            pre_callback();
            // in the main app if immediate_callback_ is set it runs the main callback before the used subcommands
            if(!final_mode && parse_complete_callback_)
            {
                parse_complete_callback_();
            }
            // run the callbacks for the received subcommands
            for(App *subc: get_subcommands())
            {
                subc->run_callback(true, suppress_final_callback);
            }
            // now run callbacks for option_groups
            for(auto &subc: subcommands_)
            {
                if(subc->name_.empty() && subc->count_all() > 0)
                {
                    subc->run_callback(true, suppress_final_callback);
                }
            }

            // finally run the main callback
            if(final_callback_ && (parsed_ > 0) && (!suppress_final_callback))
            {
                if(!name_.empty() || count_all() > 0 || parent_ == nullptr)
                {
                    final_callback_();
                }
            }
        }

        /// Check to see if a subcommand is valid. Give up immediately if subcommand max has been reached.
        bool _valid_subcommand(const std::string &current, bool ignore_used = true) const
        {
            // Don't match if max has been reached - but still check parents
            if(require_subcommand_max_ != 0 && parsed_subcommands_.size() >= require_subcommand_max_)
            {
                return parent_ != nullptr && parent_->_valid_subcommand(current, ignore_used);
            }
            auto com = _find_subcommand(current, true, ignore_used);
            if(com != nullptr)
            {
                return true;
            }
            // Check parent if exists, else return false
            return parent_ != nullptr && parent_->_valid_subcommand(current, ignore_used);
        }

        /// Selects a Classifier enum based on the type of the current argument
        detail::Classifier _recognize(const std::string &current, bool ignore_used_subcommands = true) const
        {
            std::string dummy1, dummy2;

            if(current == "--")
                return detail::Classifier::POSITIONAL_MARK;
            if(_valid_subcommand(current, ignore_used_subcommands))
                return detail::Classifier::SUBCOMMAND;
            if(detail::split_long(current, dummy1, dummy2))
                return detail::Classifier::LONG;
            if(detail::split_short(current, dummy1, dummy2))
            {
                if(dummy1[0] >= '0' && dummy1[0] <= '9')
                {
                    if(get_option_no_throw(std::string{'-', dummy1[0]}) == nullptr)
                    {
                        return detail::Classifier::NONE;
                    }
                }
                return detail::Classifier::SHORT;
            }
            if((allow_windows_style_options_) && (detail::split_windows_style(current, dummy1, dummy2)))
                return detail::Classifier::WINDOWS_STYLE;
            if((current == "++") && !name_.empty() && parent_ != nullptr)
                return detail::Classifier::SUBCOMMAND_TERMINATOR;
            return detail::Classifier::NONE;
        }

        // The parse function is now broken into several parts, and part of process

        /// Read and process a configuration file (main app only)
        void _process_config_file()
        {
            if(config_ptr_ != nullptr)
            {
                bool config_required = config_ptr_->get_required();
                auto file_given      = config_ptr_->count() > 0;
                auto config_files    = config_ptr_->as<std::vector<std::string>>();
                if(config_files.empty() || config_files.front().empty())
                {
                    if(config_required)
                    {
                        throw FileError::Missing("no specified config file");
                    }
                    return;
                }
                for(auto rit = config_files.rbegin(); rit != config_files.rend(); ++rit)
                {
                    const auto &config_file = *rit;
                    auto path_result        = detail::check_path(config_file.c_str());
                    if(path_result == detail::path_type::file)
                    {
                        try
                        {
                            std::vector<ConfigItem> values = config_formatter_->from_file(config_file);
                            _parse_config(values);
                            if(!file_given)
                            {
                                config_ptr_->add_result(config_file);
                            }
                        }
                        catch(const FileError &)
                        {
                            if(config_required || file_given)
                                throw;
                        }
                    }
                    else if(config_required || file_given)
                    {
                        throw FileError::Missing(config_file);
                    }
                }
            }
        }

        /// Get envname options if not yet passed. Runs on *all* subcommands.
        void _process_env()
        {
            for(const Option_p &opt: options_)
            {
                if(opt->count() == 0 && !opt->envname_.empty())
                {
                    char *buffer = nullptr;
                    std::string ename_string;

#ifdef _MSC_VER
                    // Windows version
                    std::size_t sz = 0;
                    if(_dupenv_s(&buffer, &sz, opt->envname_.c_str()) == 0 && buffer != nullptr)
                    {
                        ename_string = std::string(buffer);
                        free(buffer);
                    }
#else
                    // This also works on Windows, but gives a warning
                    buffer = std::getenv(opt->envname_.c_str());
                    if(buffer != nullptr)
                        ename_string = std::string(buffer);
#endif

                    if(!ename_string.empty())
                    {
                        opt->add_result(ename_string);
                    }
                }
            }

            for(App_p &sub: subcommands_)
            {
                if(sub->get_name().empty() || !sub->parse_complete_callback_)
                    sub->_process_env();
            }
        }

        /// Process callbacks. Runs on *all* subcommands.
        void _process_callbacks()
        {
            for(App_p &sub: subcommands_)
            {
                // process the priority option_groups first
                if(sub->get_name().empty() && sub->parse_complete_callback_)
                {
                    if(sub->count_all() > 0)
                    {
                        sub->_process_callbacks();
                        sub->run_callback();
                    }
                }
            }

            for(const Option_p &opt: options_)
            {
                if(opt->count() > 0 && !opt->get_callback_run())
                {
                    opt->run_callback();
                }
            }
            for(App_p &sub: subcommands_)
            {
                if(!sub->parse_complete_callback_)
                {
                    sub->_process_callbacks();
                }
            }
        }

        /// Run help flag processing if any are found.
        ///
        /// The flags allow recursive calls to remember if there was a help flag on a parent.
        void _process_help_flags(bool trigger_help = false, bool trigger_all_help = false) const
        {
            const Option *help_ptr     = get_help_ptr();
            const Option *help_all_ptr = get_help_all_ptr();

            if(help_ptr != nullptr && help_ptr->count() > 0)
                trigger_help = true;
            if(help_all_ptr != nullptr && help_all_ptr->count() > 0)
                trigger_all_help = true;

            // If there were parsed subcommands, call those. First subcommand wins if there are multiple ones.
            if(!parsed_subcommands_.empty())
            {
                for(const App *sub: parsed_subcommands_)
                    sub->_process_help_flags(trigger_help, trigger_all_help);

                // Only the final subcommand should call for help. All help wins over help.
            }
            else if(trigger_all_help)
            {
                throw CallForAllHelp();
            }
            else if(trigger_help)
            {
                throw CallForHelp();
            }
        }

        /// Verify required options and cross requirements. Subcommands too (only if selected).
        void _process_requirements()
        {
            // check excludes
            bool excluded{false};
            std::string excluder;
            for(auto &opt: exclude_options_)
            {
                if(opt->count() > 0)
                {
                    excluded = true;
                    excluder = opt->get_name();
                }
            }
            for(auto &subc: exclude_subcommands_)
            {
                if(subc->count_all() > 0)
                {
                    excluded = true;
                    excluder = subc->get_display_name();
                }
            }
            if(excluded)
            {
                if(count_all() > 0)
                {
                    throw ExcludesError(get_display_name(), excluder);
                }
                // if we are excluded but didn't receive anything, just return
                return;
            }

            // check excludes
            bool missing_needed{false};
            std::string missing_need;
            for(auto &opt: need_options_)
            {
                if(opt->count() == 0)
                {
                    missing_needed = true;
                    missing_need   = opt->get_name();
                }
            }
            for(auto &subc: need_subcommands_)
            {
                if(subc->count_all() == 0)
                {
                    missing_needed = true;
                    missing_need   = subc->get_display_name();
                }
            }
            if(missing_needed)
            {
                if(count_all() > 0)
                {
                    throw RequiresError(get_display_name(), missing_need);
                }
                // if we missing something but didn't have any options, just return
                return;
            }

            std::size_t used_options = 0;
            for(const Option_p &opt: options_)
            {
                if(opt->count() != 0)
                {
                    ++used_options;
                }
                // Required but empty
                if(opt->get_required() && opt->count() == 0)
                {
                    throw RequiredError(opt->get_name());
                }
                // Requires
                for(const Option *opt_req: opt->needs_)
                    if(opt->count() > 0 && opt_req->count() == 0)
                        throw RequiresError(opt->get_name(), opt_req->get_name());
                // Excludes
                for(const Option *opt_ex: opt->excludes_)
                    if(opt->count() > 0 && opt_ex->count() != 0)
                        throw ExcludesError(opt->get_name(), opt_ex->get_name());
            }
            // check for the required number of subcommands
            if(require_subcommand_min_ > 0)
            {
                auto selected_subcommands = get_subcommands();
                if(require_subcommand_min_ > selected_subcommands.size())
                    throw RequiredError::Subcommand(require_subcommand_min_);
            }

            // Max error cannot occur, the extra subcommand will parse as an ExtrasError or a remaining item.

            // run this loop to check how many unnamed subcommands were actually used since they are considered options
            // from the perspective of an App
            for(App_p &sub: subcommands_)
            {
                if(sub->disabled_)
                    continue;
                if(sub->name_.empty() && sub->count_all() > 0)
                {
                    ++used_options;
                }
            }

            if(require_option_min_ > used_options || (require_option_max_ > 0 && require_option_max_ < used_options))
            {
                auto option_list = detail::join(options_,
                                                [this](const Option_p &ptr)
                                                {
                                                    if(ptr.get() == help_ptr_ || ptr.get() == help_all_ptr_)
                                                    {
                                                        return std::string{};
                                                    }
                                                    return ptr->get_name(false, true);
                                                });

                auto subc_list = get_subcommands(
                    [](App *app)
                    {
                        return ((app->get_name().empty()) && (!app->disabled_));
                    });
                if(!subc_list.empty())
                {
                    option_list += "," + detail::join(subc_list,
                                                      [](const App *app)
                                                      {
                                                          return app->get_display_name();
                                                      });
                }
                throw RequiredError::Option(require_option_min_, require_option_max_, used_options, option_list);
            }

            // now process the requirements for subcommands if needed
            for(App_p &sub: subcommands_)
            {
                if(sub->disabled_)
                    continue;
                if(sub->name_.empty() && sub->required_ == false)
                {
                    if(sub->count_all() == 0)
                    {
                        if(require_option_min_ > 0 && require_option_min_ <= used_options)
                        {
                            continue;
                            // if we have met the requirement and there is nothing in this option group skip checking
                            // requirements
                        }
                        if(require_option_max_ > 0 && used_options >= require_option_min_)
                        {
                            continue;
                            // if we have met the requirement and there is nothing in this option group skip checking
                            // requirements
                        }
                    }
                }
                if(sub->count() > 0 || sub->name_.empty())
                {
                    sub->_process_requirements();
                }

                if(sub->required_ && sub->count_all() == 0)
                {
                    throw(CLI::RequiredError(sub->get_display_name()));
                }
            }
        }

        /// Process callbacks and such.
        void _process()
        {
            CLI::FileError fe("ne");
            bool caught_error{false};
            try
            {
                // the config file might generate a FileError but that should not be processed until later in the
                // process to allow for help, version and other errors to generate first.
                _process_config_file();
                // process env shouldn't throw but no reason to process it if config generated an error
                _process_env();
            }
            catch(const CLI::FileError &fe2)
            {
                fe           = fe2;
                caught_error = true;
            }
            // callbacks and help_flags can generate exceptions which should take priority over the config file error if
            // one exists
            _process_callbacks();
            _process_help_flags();

            if(caught_error)
            {
                throw CLI::FileError(std::move(fe));
            }

            _process_requirements();
        }

        /// Throw an error if anything is left over and should not be.
        void _process_extras()
        {
            if(!(allow_extras_ || prefix_command_))
            {
                std::size_t num_left_over = remaining_size();
                if(num_left_over > 0)
                {
                    throw ExtrasError(name_, remaining(false));
                }
            }

            for(App_p &sub: subcommands_)
            {
                if(sub->count() > 0)
                    sub->_process_extras();
            }
        }

        /// Throw an error if anything is left over and should not be.
        /// Modifies the args to fill in the missing items before throwing.
        void _process_extras(std::vector<std::string> &args)
        {
            if(!(allow_extras_ || prefix_command_))
            {
                std::size_t num_left_over = remaining_size();
                if(num_left_over > 0)
                {
                    args = remaining(false);
                    throw ExtrasError(name_, args);
                }
            }

            for(App_p &sub: subcommands_)
            {
                if(sub->count() > 0)
                    sub->_process_extras(args);
            }
        }

        /// Internal function to recursively increment the parsed counter on the current app as well unnamed subcommands
        void increment_parsed()
        {
            ++parsed_;
            for(App_p &sub: subcommands_)
            {
                if(sub->get_name().empty())
                    sub->increment_parsed();
            }
        }
        /// Internal parse function
        void _parse(std::vector<std::string> &args)
        {
            increment_parsed();
            _trigger_pre_parse(args.size());
            bool positional_only = false;

            while(!args.empty())
            {
                if(!_parse_single(args, positional_only))
                {
                    break;
                }
            }

            if(parent_ == nullptr)
            {
                _process();

                // Throw error if any items are left over (depending on settings)
                _process_extras(args);

                // Convert missing (pairs) to extras (string only) ready for processing in another app
                args = remaining_for_passthrough(false);
            }
            else if(parse_complete_callback_)
            {
                _process_env();
                _process_callbacks();
                _process_help_flags();
                _process_requirements();
                run_callback(false, true);
            }
        }

        /// Internal parse function
        void _parse(std::vector<std::string> &&args)
        {
            // this can only be called by the top level in which case parent == nullptr by definition
            // operation is simplified
            increment_parsed();
            _trigger_pre_parse(args.size());
            bool positional_only = false;

            while(!args.empty())
            {
                _parse_single(args, positional_only);
            }
            _process();

            // Throw error if any items are left over (depending on settings)
            _process_extras();
        }

        /// Parse one config param, return false if not found in any subcommand, remove if it is
        ///
        /// If this has more than one dot.separated.name, go into the subcommand matching it
        /// Returns true if it managed to find the option, if false you'll need to remove the arg manually.
        void _parse_config(const std::vector<ConfigItem> &args)
        {
            for(const ConfigItem &item: args)
            {
                if(!_parse_single_config(item) && allow_config_extras_ == config_extras_mode::error)
                    throw ConfigError::Extras(item.fullname());
            }
        }

        /// Fill in a single config option
        bool _parse_single_config(const ConfigItem &item, std::size_t level = 0)
        {
            if(level < item.parents.size())
            {
                try
                {
                    auto subcom = get_subcommand(item.parents.at(level));
                    auto result = subcom->_parse_single_config(item, level + 1);

                    return result;
                }
                catch(const OptionNotFound &)
                {
                    return false;
                }
            }
            // check for section open
            if(item.name == "++")
            {
                if(configurable_)
                {
                    increment_parsed();
                    _trigger_pre_parse(2);
                    if(parent_ != nullptr)
                    {
                        parent_->parsed_subcommands_.push_back(this);
                    }
                }
                return true;
            }
            // check for section close
            if(item.name == "--")
            {
                if(configurable_)
                {
                    _process_callbacks();
                    _process_requirements();
                    run_callback();
                }
                return true;
            }
            Option *op = get_option_no_throw("--" + item.name);
            if(op == nullptr)
            {
                if(item.name.size() == 1)
                {
                    op = get_option_no_throw("-" + item.name);
                }
            }
            if(op == nullptr)
            {
                op = get_option_no_throw(item.name);
            }
            if(op == nullptr)
            {
                // If the option was not present
                if(get_allow_config_extras() == config_extras_mode::capture)
                    // Should we worry about classifying the extras properly?
                    missing_.emplace_back(detail::Classifier::NONE, item.fullname());
                return false;
            }

            if(!op->get_configurable())
                throw ConfigError::NotConfigurable(item.fullname());

            if(op->empty())
            {
                // Flag parsing
                if(op->get_expected_min() == 0)
                {
                    auto res = config_formatter_->to_flag(item);
                    res      = op->get_flag_value(item.name, res);

                    op->add_result(res);
                }
                else
                {
                    op->add_result(item.inputs);
                    op->run_callback();
                }
            }

            return true;
        }

        /// Parse "one" argument (some may eat more than one), delegate to parent if fails, add to missing if missing
        /// from master return false if the parse has failed and needs to return to parent
        bool _parse_single(std::vector<std::string> &args, bool &positional_only)
        {
            bool retval                   = true;
            detail::Classifier classifier = positional_only ? detail::Classifier::NONE : _recognize(args.back());
            switch(classifier)
            {
                case detail::Classifier::POSITIONAL_MARK:
                    args.pop_back();
                    positional_only = true;
                    if((!_has_remaining_positionals()) && (parent_ != nullptr))
                    {
                        retval = false;
                    }
                    else
                    {
                        _move_to_missing(classifier, "--");
                    }
                    break;
                case detail::Classifier::SUBCOMMAND_TERMINATOR:
                    // treat this like a positional mark if in the parent app
                    args.pop_back();
                    retval = false;
                    break;
                case detail::Classifier::SUBCOMMAND:
                    retval = _parse_subcommand(args);
                    break;
                case detail::Classifier::LONG:
                case detail::Classifier::SHORT:
                case detail::Classifier::WINDOWS_STYLE:
                    // If already parsed a subcommand, don't accept options_
                    _parse_arg(args, classifier);
                    break;
                case detail::Classifier::NONE:
                    // Probably a positional or something for a parent (sub)command
                    retval = _parse_positional(args, false);
                    if(retval && positionals_at_end_)
                    {
                        positional_only = true;
                    }
                    break;
                // LCOV_EXCL_START
                default:
                    throw HorribleError("unrecognized classifier (you should not see this!)");
                    // LCOV_EXCL_STOP
            }
            return retval;
        }

        /// Count the required remaining positional arguments
        std::size_t _count_remaining_positionals(bool required_only = false) const
        {
            std::size_t retval = 0;
            for(const Option_p &opt: options_)
            {
                if(opt->get_positional() && (!required_only || opt->get_required()))
                {
                    if(opt->get_items_expected_min() > 0 &&
                       static_cast<int>(opt->count()) < opt->get_items_expected_min())
                    {
                        retval += static_cast<std::size_t>(opt->get_items_expected_min()) - opt->count();
                    }
                }
            }
            return retval;
        }

        /// Count the required remaining positional arguments
        bool _has_remaining_positionals() const
        {
            for(const Option_p &opt: options_)
            {
                if(opt->get_positional() && ((static_cast<int>(opt->count()) < opt->get_items_expected_min())))
                {
                    return true;
                }
            }

            return false;
        }

        /// Parse a positional, go up the tree to check
        /// @param haltOnSubcommand if set to true the operation will not process subcommands merely return false
        /// Return true if the positional was used false otherwise
        bool _parse_positional(std::vector<std::string> &args, bool haltOnSubcommand)
        {
            const std::string &positional = args.back();

            if(positionals_at_end_)
            {
                // deal with the case of required arguments at the end which should take precedence over other arguments
                auto arg_rem = args.size();
                auto remreq  = _count_remaining_positionals(true);
                if(arg_rem <= remreq)
                {
                    for(const Option_p &opt: options_)
                    {
                        if(opt->get_positional() && opt->required_)
                        {
                            if(static_cast<int>(opt->count()) < opt->get_items_expected_min())
                            {
                                if(validate_positionals_)
                                {
                                    std::string pos = positional;
                                    pos             = opt->_validate(pos, 0);
                                    if(!pos.empty())
                                    {
                                        continue;
                                    }
                                }
                                opt->add_result(positional);
                                parse_order_.push_back(opt.get());
                                args.pop_back();
                                return true;
                            }
                        }
                    }
                }
            }
            for(const Option_p &opt: options_)
            {
                // Eat options, one by one, until done
                if(opt->get_positional() &&
                   (static_cast<int>(opt->count()) < opt->get_items_expected_min() || opt->get_allow_extra_args()))
                {
                    if(validate_positionals_)
                    {
                        std::string pos = positional;
                        pos             = opt->_validate(pos, 0);
                        if(!pos.empty())
                        {
                            continue;
                        }
                    }
                    opt->add_result(positional);
                    parse_order_.push_back(opt.get());
                    args.pop_back();
                    return true;
                }
            }

            for(auto &subc: subcommands_)
            {
                if((subc->name_.empty()) && (!subc->disabled_))
                {
                    if(subc->_parse_positional(args, false))
                    {
                        if(!subc->pre_parse_called_)
                        {
                            subc->_trigger_pre_parse(args.size());
                        }
                        return true;
                    }
                }
            }
            // let the parent deal with it if possible
            if(parent_ != nullptr && fallthrough_)
                return _get_fallthrough_parent()->_parse_positional(args, static_cast<bool>(parse_complete_callback_));

            /// Try to find a local subcommand that is repeated
            auto com = _find_subcommand(args.back(), true, false);
            if(com != nullptr && (require_subcommand_max_ == 0 || require_subcommand_max_ > parsed_subcommands_.size()))
            {
                if(haltOnSubcommand)
                {
                    return false;
                }
                args.pop_back();
                com->_parse(args);
                return true;
            }
            /// now try one last gasp at subcommands that have been executed before, go to root app and try to find a
            /// subcommand in a broader way, if one exists let the parent deal with it
            auto parent_app = (parent_ != nullptr) ? _get_fallthrough_parent() : this;
            com             = parent_app->_find_subcommand(args.back(), true, false);
            if(com != nullptr && (com->parent_->require_subcommand_max_ == 0 ||
                                  com->parent_->require_subcommand_max_ > com->parent_->parsed_subcommands_.size()))
            {
                return false;
            }

            if(positionals_at_end_)
            {
                throw CLI::ExtrasError(name_, args);
            }
            /// If this is an option group don't deal with it
            if(parent_ != nullptr && name_.empty())
            {
                return false;
            }
            /// We are out of other options this goes to missing
            _move_to_missing(detail::Classifier::NONE, positional);
            args.pop_back();
            if(prefix_command_)
            {
                while(!args.empty())
                {
                    _move_to_missing(detail::Classifier::NONE, args.back());
                    args.pop_back();
                }
            }

            return true;
        }

        /// Locate a subcommand by name with two conditions, should disabled subcommands be ignored, and should used
        /// subcommands be ignored
        App *_find_subcommand(const std::string &subc_name, bool ignore_disabled, bool ignore_used) const noexcept
        {
            for(const App_p &com: subcommands_)
            {
                if(com->disabled_ && ignore_disabled)
                    continue;
                if(com->get_name().empty())
                {
                    auto subc = com->_find_subcommand(subc_name, ignore_disabled, ignore_used);
                    if(subc != nullptr)
                    {
                        return subc;
                    }
                }
                if(com->check_name(subc_name))
                {
                    if((!*com) || !ignore_used)
                        return com.get();
                }
            }
            return nullptr;
        }

        /// Parse a subcommand, modify args and continue
        ///
        /// Unlike the others, this one will always allow fallthrough
        /// return true if the subcommand was processed false otherwise
        bool _parse_subcommand(std::vector<std::string> &args)
        {
            if(_count_remaining_positionals(/* required */ true) > 0)
            {
                _parse_positional(args, false);
                return true;
            }
            auto com = _find_subcommand(args.back(), true, true);
            if(com != nullptr)
            {
                args.pop_back();
                if(!com->silent_)
                {
                    parsed_subcommands_.push_back(com);
                }
                com->_parse(args);
                auto parent_app = com->parent_;
                while(parent_app != this)
                {
                    parent_app->_trigger_pre_parse(args.size());
                    if(!com->silent_)
                    {
                        parent_app->parsed_subcommands_.push_back(com);
                    }
                    parent_app = parent_app->parent_;
                }
                return true;
            }

            if(parent_ == nullptr)
                throw HorribleError("Subcommand " + args.back() + " missing");
            return false;
        }

        /// Parse a short (false) or long (true) argument, must be at the top of the list
        /// return true if the argument was processed or false if nothing was done
        bool _parse_arg(std::vector<std::string> &args, detail::Classifier current_type)
        {
            std::string current = args.back();

            std::string arg_name;
            std::string value;
            std::string rest;

            switch(current_type)
            {
                case detail::Classifier::LONG:
                    if(!detail::split_long(current, arg_name, value))
                        throw HorribleError("Long parsed but missing (you should not see this):" + args.back());
                    break;
                case detail::Classifier::SHORT:
                    if(!detail::split_short(current, arg_name, rest))
                        throw HorribleError("Short parsed but missing! You should not see this");
                    break;
                case detail::Classifier::WINDOWS_STYLE:
                    if(!detail::split_windows_style(current, arg_name, value))
                        throw HorribleError("windows option parsed but missing! You should not see this");
                    break;
                case detail::Classifier::SUBCOMMAND:
                case detail::Classifier::SUBCOMMAND_TERMINATOR:
                case detail::Classifier::POSITIONAL_MARK:
                case detail::Classifier::NONE:
                default:
                    throw HorribleError("parsing got called with invalid option! You should not see this");
            }

            auto op_ptr = std::find_if(std::begin(options_),
                                       std::end(options_),
                                       [arg_name, current_type](const Option_p &opt)
                                       {
                                           if(current_type == detail::Classifier::LONG)
                                               return opt->check_lname(arg_name);
                                           if(current_type == detail::Classifier::SHORT)
                                               return opt->check_sname(arg_name);
                                           // this will only get called for detail::Classifier::WINDOWS_STYLE
                                           return opt->check_lname(arg_name) || opt->check_sname(arg_name);
                                       });

            // Option not found
            if(op_ptr == std::end(options_))
            {
                for(auto &subc: subcommands_)
                {
                    if(subc->name_.empty() && !subc->disabled_)
                    {
                        if(subc->_parse_arg(args, current_type))
                        {
                            if(!subc->pre_parse_called_)
                            {
                                subc->_trigger_pre_parse(args.size());
                            }
                            return true;
                        }
                    }
                }
                // If a subcommand, try the master command
                if(parent_ != nullptr && fallthrough_)
                    return _get_fallthrough_parent()->_parse_arg(args, current_type);
                // don't capture missing if this is a nameless subcommand
                if(parent_ != nullptr && name_.empty())
                {
                    return false;
                }
                // Otherwise, add to missing
                args.pop_back();
                _move_to_missing(current_type, current);
                return true;
            }

            args.pop_back();

            // Get a reference to the pointer to make syntax bearable
            Option_p &op = *op_ptr;
            /// if we require a separator add it here
            if(op->get_inject_separator())
            {
                if(!op->results().empty() && !op->results().back().empty())
                {
                    op->add_result(std::string{});
                }
            }
            int min_num = (std::min)(op->get_type_size_min(), op->get_items_expected_min());
            int max_num = op->get_items_expected_max();
            // check container like options to limit the argument size to a single type if the allow_extra_flags
            // argument is set. 16 is somewhat arbitrary (needs to be at least 4)
            if(max_num >= detail::expected_max_vector_size / 16 && !op->get_allow_extra_args())
            {
                auto tmax = op->get_type_size_max();
                max_num =
                    detail::checked_multiply(tmax, op->get_expected_min()) ? tmax : detail::expected_max_vector_size;
            }
            // Make sure we always eat the minimum for unlimited vectors
            int collected    = 0;  // total number of arguments collected
            int result_count = 0;  // local variable for number of results in a single arg string
            // deal with purely flag like things
            if(max_num == 0)
            {
                auto res = op->get_flag_value(arg_name, value);
                op->add_result(res);
                parse_order_.push_back(op.get());
            }
            else if(!value.empty())
            {  // --this=value
                op->add_result(value, result_count);
                parse_order_.push_back(op.get());
                collected += result_count;
                // -Trest
            }
            else if(!rest.empty())
            {
                op->add_result(rest, result_count);
                parse_order_.push_back(op.get());
                rest = "";
                collected += result_count;
            }

            // gather the minimum number of arguments
            while(min_num > collected && !args.empty())
            {
                std::string current_ = args.back();
                args.pop_back();
                op->add_result(current_, result_count);
                parse_order_.push_back(op.get());
                collected += result_count;
            }

            if(min_num > collected)
            {  // if we have run out of arguments and the minimum was not met
                throw ArgumentMismatch::TypedAtLeast(op->get_name(), min_num, op->get_type_name());
            }

            if(max_num > collected || op->get_allow_extra_args())
            {  // we allow optional arguments
                auto remreqpos = _count_remaining_positionals(true);
                // we have met the minimum now optionally check up to the maximum
                while((collected < max_num || op->get_allow_extra_args()) && !args.empty() &&
                      _recognize(args.back(), false) == detail::Classifier::NONE)
                {
                    // If any required positionals remain, don't keep eating
                    if(remreqpos >= args.size())
                    {
                        break;
                    }

                    op->add_result(args.back(), result_count);
                    parse_order_.push_back(op.get());
                    args.pop_back();
                    collected += result_count;
                }

                // Allow -- to end an unlimited list and "eat" it
                if(!args.empty() && _recognize(args.back()) == detail::Classifier::POSITIONAL_MARK)
                    args.pop_back();
                // optional flag that didn't receive anything now get the default value
                if(min_num == 0 && max_num > 0 && collected == 0)
                {
                    auto res = op->get_flag_value(arg_name, std::string{});
                    op->add_result(res);
                    parse_order_.push_back(op.get());
                }
            }

            // if we only partially completed a type then add an empty string for later processing
            if(min_num > 0 && op->get_type_size_max() != min_num && (collected % op->get_type_size_max()) != 0)
            {
                op->add_result(std::string{});
            }

            if(!rest.empty())
            {
                rest = "-" + rest;
                args.push_back(rest);
            }
            return true;
        }

        /// Trigger the pre_parse callback if needed
        void _trigger_pre_parse(std::size_t remaining_args)
        {
            if(!pre_parse_called_)
            {
                pre_parse_called_ = true;
                if(pre_parse_callback_)
                {
                    pre_parse_callback_(remaining_args);
                }
            }
            else if(immediate_callback_)
            {
                if(!name_.empty())
                {
                    auto pcnt   = parsed_;
                    auto extras = std::move(missing_);
                    clear();
                    parsed_           = pcnt;
                    pre_parse_called_ = true;
                    missing_          = std::move(extras);
                }
            }
        }

        /// Get the appropriate parent to fallthrough to which is the first one that has a name or the main app
        App *_get_fallthrough_parent()
        {
            if(parent_ == nullptr)
            {
                throw(HorribleError("No Valid parent"));
            }
            auto fallthrough_parent = parent_;
            while((fallthrough_parent->parent_ != nullptr) && (fallthrough_parent->get_name().empty()))
            {
                fallthrough_parent = fallthrough_parent->parent_;
            }
            return fallthrough_parent;
        }

        /// Helper function to run through all possible comparisons of subcommand names to check there is no overlap
        const std::string &_compare_subcommand_names(const App &subcom, const App &base) const
        {
            static const std::string estring;
            if(subcom.disabled_)
            {
                return estring;
            }
            for(auto &subc: base.subcommands_)
            {
                if(subc.get() != &subcom)
                {
                    if(subc->disabled_)
                    {
                        continue;
                    }
                    if(!subcom.get_name().empty())
                    {
                        if(subc->check_name(subcom.get_name()))
                        {
                            return subcom.get_name();
                        }
                    }
                    if(!subc->get_name().empty())
                    {
                        if(subcom.check_name(subc->get_name()))
                        {
                            return subc->get_name();
                        }
                    }
                    for(const auto &les: subcom.aliases_)
                    {
                        if(subc->check_name(les))
                        {
                            return les;
                        }
                    }
                    // this loop is needed in case of ignore_underscore or ignore_case on one but not the other
                    for(const auto &les: subc->aliases_)
                    {
                        if(subcom.check_name(les))
                        {
                            return les;
                        }
                    }
                    // if the subcommand is an option group we need to check deeper
                    if(subc->get_name().empty())
                    {
                        auto &cmpres = _compare_subcommand_names(subcom, *subc);
                        if(!cmpres.empty())
                        {
                            return cmpres;
                        }
                    }
                    // if the test subcommand is an option group we need to check deeper
                    if(subcom.get_name().empty())
                    {
                        auto &cmpres = _compare_subcommand_names(*subc, subcom);
                        if(!cmpres.empty())
                        {
                            return cmpres;
                        }
                    }
                }
            }
            return estring;
        }
        /// Helper function to place extra values in the most appropriate position
        void _move_to_missing(detail::Classifier val_type, const std::string &val)
        {
            if(allow_extras_ || subcommands_.empty())
            {
                missing_.emplace_back(val_type, val);
                return;
            }
            // allow extra arguments to be places in an option group if it is allowed there
            for(auto &subc: subcommands_)
            {
                if(subc->name_.empty() && subc->allow_extras_)
                {
                    subc->missing_.emplace_back(val_type, val);
                    return;
                }
            }
            // if we haven't found any place to put them yet put them in missing
            missing_.emplace_back(val_type, val);
        }

       public:
        /// function that could be used by subclasses of App to shift options around into subcommands
        void _move_option(Option *opt, App *app)
        {
            if(opt == nullptr)
            {
                throw OptionNotFound("the option is NULL");
            }
            // verify that the give app is actually a subcommand
            bool found = false;
            for(auto &subc: subcommands_)
            {
                if(app == subc.get())
                {
                    found = true;
                }
            }
            if(!found)
            {
                throw OptionNotFound("The Given app is not a subcommand");
            }

            if((help_ptr_ == opt) || (help_all_ptr_ == opt))
                throw OptionAlreadyAdded("cannot move help options");

            if(config_ptr_ == opt)
                throw OptionAlreadyAdded("cannot move config file options");

            auto iterator = std::find_if(std::begin(options_),
                                         std::end(options_),
                                         [opt](const Option_p &v)
                                         {
                                             return v.get() == opt;
                                         });
            if(iterator != std::end(options_))
            {
                const auto &opt_p = *iterator;
                if(std::find_if(std::begin(app->options_),
                                std::end(app->options_),
                                [&opt_p](const Option_p &v)
                                {
                                    return (*v == *opt_p);
                                }) == std::end(app->options_))
                {
                    // only erase after the insertion was successful
                    app->options_.push_back(std::move(*iterator));
                    options_.erase(iterator);
                }
                else
                {
                    throw OptionAlreadyAdded("option was not located: " + opt->get_name());
                }
            }
            else
            {
                throw OptionNotFound("could not locate the given Option");
            }
        }
    };  // namespace CLI

    /// Extension of App to better manage groups of options
    class Option_group : public App
    {
       public:
        Option_group(std::string group_description, std::string group_name, App *parent)
            : App(std::move(group_description), "", parent)
        {
            group(group_name);
            // option groups should have automatic fallthrough
        }
        using App::add_option;
        /// Add an existing option to the Option_group
        Option *add_option(Option *opt)
        {
            if(get_parent() == nullptr)
            {
                throw OptionNotFound("Unable to locate the specified option");
            }
            get_parent()->_move_option(opt, this);
            return opt;
        }
        /// Add an existing option to the Option_group
        void add_options(Option *opt)
        {
            add_option(opt);
        }
        /// Add a bunch of options to the group
        template <typename... Args>
        void add_options(Option *opt, Args... args)
        {
            add_option(opt);
            add_options(args...);
        }
        using App::add_subcommand;
        /// Add an existing subcommand to be a member of an option_group
        App *add_subcommand(App *subcom)
        {
            App_p subc = subcom->get_parent()->get_subcommand_ptr(subcom);
            subc->get_parent()->remove_subcommand(subcom);
            add_subcommand(std::move(subc));
            return subcom;
        }
    };
    /// Helper function to enable one option group/subcommand when another is used
    inline void TriggerOn(App *trigger_app, App *app_to_enable)
    {
        app_to_enable->enabled_by_default(false);
        app_to_enable->disabled_by_default();
        trigger_app->preparse_callback(
            [app_to_enable](std::size_t)
            {
                app_to_enable->disabled(false);
            });
    }

    /// Helper function to enable one option group/subcommand when another is used
    inline void TriggerOn(App *trigger_app, std::vector<App *> apps_to_enable)
    {
        for(auto &app: apps_to_enable)
        {
            app->enabled_by_default(false);
            app->disabled_by_default();
        }

        trigger_app->preparse_callback(
            [apps_to_enable](std::size_t)
            {
                for(auto &app: apps_to_enable)
                {
                    app->disabled(false);
                }
            });
    }

    /// Helper function to disable one option group/subcommand when another is used
    inline void TriggerOff(App *trigger_app, App *app_to_enable)
    {
        app_to_enable->disabled_by_default(false);
        app_to_enable->enabled_by_default();
        trigger_app->preparse_callback(
            [app_to_enable](std::size_t)
            {
                app_to_enable->disabled();
            });
    }

    /// Helper function to disable one option group/subcommand when another is used
    inline void TriggerOff(App *trigger_app, std::vector<App *> apps_to_enable)
    {
        for(auto &app: apps_to_enable)
        {
            app->disabled_by_default(false);
            app->enabled_by_default();
        }

        trigger_app->preparse_callback(
            [apps_to_enable](std::size_t)
            {
                for(auto &app: apps_to_enable)
                {
                    app->disabled();
                }
            });
    }

    /// Helper function to mark an option as deprecated
    inline void deprecate_option(Option *opt, const std::string &replacement = "")
    {
        Validator deprecate_warning{[opt, replacement](std::string &)
                                    {
                                        std::cout << opt->get_name() << " is deprecated please use '" << replacement
                                                  << "' instead\n";
                                        return std::string();
                                    },
                                    "DEPRECATED"};
        deprecate_warning.application_index(0);
        opt->check(deprecate_warning);
        if(!replacement.empty())
        {
            opt->description(opt->get_description() + " DEPRECATED: please use '" + replacement + "' instead");
        }
    }

    /// Helper function to mark an option as deprecated
    inline void deprecate_option(App *app, const std::string &option_name, const std::string &replacement = "")
    {
        auto opt = app->get_option(option_name);
        deprecate_option(opt, replacement);
    }

    /// Helper function to mark an option as deprecated
    inline void deprecate_option(App &app, const std::string &option_name, const std::string &replacement = "")
    {
        auto opt = app.get_option(option_name);
        deprecate_option(opt, replacement);
    }

    /// Helper function to mark an option as retired
    inline void retire_option(App *app, Option *opt)
    {
        App temp;
        auto option_copy = temp.add_option(opt->get_name(false, true))
                               ->type_size(opt->get_type_size_min(), opt->get_type_size_max())
                               ->expected(opt->get_expected_min(), opt->get_expected_max())
                               ->allow_extra_args(opt->get_allow_extra_args());

        app->remove_option(opt);
        auto opt2 = app->add_option(option_copy->get_name(false, true), "option has been retired and has no effect")
                        ->type_name("RETIRED")
                        ->default_str("RETIRED")
                        ->type_size(option_copy->get_type_size_min(), option_copy->get_type_size_max())
                        ->expected(option_copy->get_expected_min(), option_copy->get_expected_max())
                        ->allow_extra_args(option_copy->get_allow_extra_args());

        Validator retired_warning{[opt2](std::string &)
                                  {
                                      std::cout << "WARNING " << opt2->get_name() << " is retired and has no effect\n";
                                      return std::string();
                                  },
                                  ""};
        retired_warning.application_index(0);
        opt2->check(retired_warning);
    }

    /// Helper function to mark an option as retired
    inline void retire_option(App &app, Option *opt)
    {
        retire_option(&app, opt);
    }

    /// Helper function to mark an option as retired
    inline void retire_option(App *app, const std::string &option_name)
    {
        auto opt = app->get_option_no_throw(option_name);
        if(opt != nullptr)
        {
            retire_option(app, opt);
            return;
        }
        auto opt2 = app->add_option(option_name, "option has been retired and has no effect")
                        ->type_name("RETIRED")
                        ->expected(0, 1)
                        ->default_str("RETIRED");
        Validator retired_warning{[opt2](std::string &)
                                  {
                                      std::cout << "WARNING " << opt2->get_name() << " is retired and has no effect\n";
                                      return std::string();
                                  },
                                  ""};
        retired_warning.application_index(0);
        opt2->check(retired_warning);
    }

    /// Helper function to mark an option as retired
    inline void retire_option(App &app, const std::string &option_name)
    {
        retire_option(&app, option_name);
    }

    namespace FailureMessage
    {

        /// Printout a clean, simple message on error (the default in CLI11 1.5+)
        inline std::string simple(const App *app, const Error &e)
        {
            std::string header = std::string(e.what()) + "\n";
            std::vector<std::string> names;

            // Collect names
            if(app->get_help_ptr() != nullptr)
                names.push_back(app->get_help_ptr()->get_name());

            if(app->get_help_all_ptr() != nullptr)
                names.push_back(app->get_help_all_ptr()->get_name());

            // If any names found, suggest those
            if(!names.empty())
                header += "Run with " + detail::join(names, " or ") + " for more information.\n";

            return header;
        }

        /// Printout the full help string on error (if this fn is set, the old default for CLI11)
        inline std::string help(const App *app, const Error &e)
        {
            std::string header = std::string("ERROR: ") + e.get_name() + ": " + e.what() + "\n";
            header += app->help();
            return header;
        }

    }  // namespace FailureMessage

    namespace detail
    {
        /// This class is simply to allow tests access to App's protected functions
        struct AppFriend
        {
#ifdef CLI11_CPP14

            /// Wrap _parse_short, perfectly forward arguments and return
            template <typename... Args>
            static decltype(auto) parse_arg(App *app, Args &&...args)
            {
                return app->_parse_arg(std::forward<Args>(args)...);
            }

            /// Wrap _parse_subcommand, perfectly forward arguments and return
            template <typename... Args>
            static decltype(auto) parse_subcommand(App *app, Args &&...args)
            {
                return app->_parse_subcommand(std::forward<Args>(args)...);
            }
#else
            /// Wrap _parse_short, perfectly forward arguments and return
            template <typename... Args>
            static auto parse_arg(App *app, Args &&...args) ->
                typename std::result_of<decltype (&App::_parse_arg)(App, Args...)>::type
            {
                return app->_parse_arg(std::forward<Args>(args)...);
            }

            /// Wrap _parse_subcommand, perfectly forward arguments and return
            template <typename... Args>
            static auto parse_subcommand(App *app, Args &&...args) ->
                typename std::result_of<decltype (&App::_parse_subcommand)(App, Args...)>::type
            {
                return app->_parse_subcommand(std::forward<Args>(args)...);
            }
#endif
            /// Wrap the fallthrough parent function to make sure that is working correctly
            static App *get_fallthrough_parent(App *app)
            {
                return app->_get_fallthrough_parent();
            }
        };
    }  // namespace detail

    namespace detail
    {

        inline std::string convert_arg_for_ini(const std::string &arg,
                                               char stringQuote    = '"',
                                               char characterQuote = '\'')
        {
            if(arg.empty())
            {
                return std::string(2, stringQuote);
            }
            // some specifically supported strings
            if(arg == "true" || arg == "false" || arg == "nan" || arg == "inf")
            {
                return arg;
            }
            // floating point conversion can convert some hex codes, but don't try that here
            if(arg.compare(0, 2, "0x") != 0 && arg.compare(0, 2, "0X") != 0)
            {
                double val;
                if(detail::lexical_cast(arg, val))
                {
                    return arg;
                }
            }
            // just quote a single non numeric character
            if(arg.size() == 1)
            {
                return std::string(1, characterQuote) + arg + characterQuote;
            }
            // handle hex, binary or octal arguments
            if(arg.front() == '0')
            {
                if(arg[1] == 'x')
                {
                    if(std::all_of(arg.begin() + 2,
                                   arg.end(),
                                   [](char x)
                                   {
                                       return (x >= '0' && x <= '9') || (x >= 'A' && x <= 'F') ||
                                              (x >= 'a' && x <= 'f');
                                   }))
                    {
                        return arg;
                    }
                }
                else if(arg[1] == 'o')
                {
                    if(std::all_of(arg.begin() + 2,
                                   arg.end(),
                                   [](char x)
                                   {
                                       return (x >= '0' && x <= '7');
                                   }))
                    {
                        return arg;
                    }
                }
                else if(arg[1] == 'b')
                {
                    if(std::all_of(arg.begin() + 2,
                                   arg.end(),
                                   [](char x)
                                   {
                                       return (x == '0' || x == '1');
                                   }))
                    {
                        return arg;
                    }
                }
            }
            if(arg.find_first_of(stringQuote) == std::string::npos)
            {
                return std::string(1, stringQuote) + arg + stringQuote;
            }
            else
            {
                return characterQuote + arg + characterQuote;
            }
        }

        /// Comma separated join, adds quotes if needed
        inline std::string ini_join(const std::vector<std::string> &args,
                                    char sepChar        = ',',
                                    char arrayStart     = '[',
                                    char arrayEnd       = ']',
                                    char stringQuote    = '"',
                                    char characterQuote = '\'')
        {
            std::string joined;
            if(args.size() > 1 && arrayStart != '\0')
            {
                joined.push_back(arrayStart);
            }
            std::size_t start = 0;
            for(const auto &arg: args)
            {
                if(start++ > 0)
                {
                    joined.push_back(sepChar);
                    if(isspace(sepChar) == 0)
                    {
                        joined.push_back(' ');
                    }
                }
                joined.append(convert_arg_for_ini(arg, stringQuote, characterQuote));
            }
            if(args.size() > 1 && arrayEnd != '\0')
            {
                joined.push_back(arrayEnd);
            }
            return joined;
        }

        inline std::vector<std::string> generate_parents(const std::string &section, std::string &name)
        {
            std::vector<std::string> parents;
            if(detail::to_lower(section) != "default")
            {
                if(section.find('.') != std::string::npos)
                {
                    parents = detail::split(section, '.');
                }
                else
                {
                    parents = {section};
                }
            }
            if(name.find('.') != std::string::npos)
            {
                std::vector<std::string> plist = detail::split(name, '.');
                name                           = plist.back();
                detail::remove_quotes(name);
                plist.pop_back();
                parents.insert(parents.end(), plist.begin(), plist.end());
            }

            // clean up quotes on the parents
            for(auto &parent: parents)
            {
                detail::remove_quotes(parent);
            }
            return parents;
        }

        /// assuming non default segments do a check on the close and open of the segments in a configItem structure
        inline void checkParentSegments(std::vector<ConfigItem> &output, const std::string &currentSection)
        {
            std::string estring;
            auto parents = detail::generate_parents(currentSection, estring);
            if(!output.empty() && output.back().name == "--")
            {
                std::size_t msize = (parents.size() > 1U) ? parents.size() : 2;
                while(output.back().parents.size() >= msize)
                {
                    output.push_back(output.back());
                    output.back().parents.pop_back();
                }

                if(parents.size() > 1)
                {
                    std::size_t common = 0;
                    std::size_t mpair  = (std::min)(output.back().parents.size(), parents.size() - 1);
                    for(std::size_t ii = 0; ii < mpair; ++ii)
                    {
                        if(output.back().parents[ii] != parents[ii])
                        {
                            break;
                        }
                        ++common;
                    }
                    if(common == mpair)
                    {
                        output.pop_back();
                    }
                    else
                    {
                        while(output.back().parents.size() > common + 1)
                        {
                            output.push_back(output.back());
                            output.back().parents.pop_back();
                        }
                    }
                    for(std::size_t ii = common; ii < parents.size() - 1; ++ii)
                    {
                        output.emplace_back();
                        output.back().parents.assign(parents.begin(),
                                                     parents.begin() + static_cast<std::ptrdiff_t>(ii) + 1);
                        output.back().name = "++";
                    }
                }
            }
            else if(parents.size() > 1)
            {
                for(std::size_t ii = 0; ii < parents.size() - 1; ++ii)
                {
                    output.emplace_back();
                    output.back().parents.assign(parents.begin(),
                                                 parents.begin() + static_cast<std::ptrdiff_t>(ii) + 1);
                    output.back().name = "++";
                }
            }

            // insert a section end which is just an empty items_buffer
            output.emplace_back();
            output.back().parents = std::move(parents);
            output.back().name    = "++";
        }
    }  // namespace detail

    inline std::vector<ConfigItem> ConfigBase::from_config(std::istream &input) const
    {
        std::string line;
        std::string section = "default";

        std::vector<ConfigItem> output;
        bool isDefaultArray = (arrayStart == '[' && arrayEnd == ']' && arraySeparator == ',');
        bool isINIArray     = (arrayStart == '\0' || arrayStart == ' ') && arrayStart == arrayEnd;
        char aStart         = (isINIArray) ? '[' : arrayStart;
        char aEnd           = (isINIArray) ? ']' : arrayEnd;
        char aSep           = (isINIArray && arraySeparator == ' ') ? ',' : arraySeparator;

        while(getline(input, line))
        {
            std::vector<std::string> items_buffer;
            std::string name;

            detail::trim(line);
            std::size_t len = line.length();
            if(len > 1 && line.front() == '[' && line.back() == ']')
            {
                if(section != "default")
                {
                    // insert a section end which is just an empty items_buffer
                    output.emplace_back();
                    output.back().parents = detail::generate_parents(section, name);
                    output.back().name    = "--";
                }
                section = line.substr(1, len - 2);
                // deal with double brackets for TOML
                if(section.size() > 1 && section.front() == '[' && section.back() == ']')
                {
                    section = section.substr(1, section.size() - 2);
                }
                if(detail::to_lower(section) == "default")
                {
                    section = "default";
                }
                else
                {
                    detail::checkParentSegments(output, section);
                }
                continue;
            }
            if(len == 0)
            {
                continue;
            }
            // comment lines
            if(line.front() == ';' || line.front() == '#' || line.front() == commentChar)
            {
                continue;
            }

            // Find = in string, split and recombine
            auto pos = line.find(valueDelimiter);
            if(pos != std::string::npos)
            {
                name             = detail::trim_copy(line.substr(0, pos));
                std::string item = detail::trim_copy(line.substr(pos + 1));
                if(item.size() > 1 && item.front() == aStart)
                {
                    for(std::string multiline; item.back() != aEnd && std::getline(input, multiline);)
                    {
                        detail::trim(multiline);
                        item += multiline;
                    }
                    items_buffer = detail::split_up(item.substr(1, item.length() - 2), aSep);
                }
                else if((isDefaultArray || isINIArray) && item.find_first_of(aSep) != std::string::npos)
                {
                    items_buffer = detail::split_up(item, aSep);
                }
                else if((isDefaultArray || isINIArray) && item.find_first_of(' ') != std::string::npos)
                {
                    items_buffer = detail::split_up(item);
                }
                else
                {
                    items_buffer = {item};
                }
            }
            else
            {
                name         = detail::trim_copy(line);
                items_buffer = {"true"};
            }
            if(name.find('.') == std::string::npos)
            {
                detail::remove_quotes(name);
            }
            // clean up quotes on the items
            for(auto &it: items_buffer)
            {
                detail::remove_quotes(it);
            }

            std::vector<std::string> parents = detail::generate_parents(section, name);

            if(!output.empty() && name == output.back().name && parents == output.back().parents)
            {
                output.back().inputs.insert(output.back().inputs.end(), items_buffer.begin(), items_buffer.end());
            }
            else
            {
                output.emplace_back();
                output.back().parents = std::move(parents);
                output.back().name    = std::move(name);
                output.back().inputs  = std::move(items_buffer);
            }
        }
        if(section != "default")
        {
            // insert a section end which is just an empty items_buffer
            std::string ename;
            output.emplace_back();
            output.back().parents = detail::generate_parents(section, ename);
            output.back().name    = "--";
            while(output.back().parents.size() > 1)
            {
                output.push_back(output.back());
                output.back().parents.pop_back();
            }
        }
        return output;
    }

    inline std::string ConfigBase::to_config(const App *app,
                                             bool default_also,
                                             bool write_description,
                                             std::string prefix) const
    {
        std::stringstream out;
        std::string commentLead;
        commentLead.push_back(commentChar);
        commentLead.push_back(' ');

        std::vector<std::string> groups = app->get_groups();
        bool defaultUsed                = false;
        groups.insert(groups.begin(), std::string("Options"));
        if(write_description && (app->get_configurable() || app->get_parent() == nullptr || app->get_name().empty()))
        {
            out << commentLead << detail::fix_newlines(commentLead, app->get_description()) << '\n';
        }
        for(auto &group: groups)
        {
            if(group == "Options" || group.empty())
            {
                if(defaultUsed)
                {
                    continue;
                }
                defaultUsed = true;
            }
            if(write_description && group != "Options" && !group.empty())
            {
                out << '\n' << commentLead << group << " Options\n";
            }
            for(const Option *opt: app->get_options({}))
            {
                // Only process options that are configurable
                if(opt->get_configurable())
                {
                    if(opt->get_group() != group)
                    {
                        if(!(group == "Options" && opt->get_group().empty()))
                        {
                            continue;
                        }
                    }
                    std::string name  = prefix + opt->get_single_name();
                    std::string value = detail::ini_join(opt->reduced_results(),
                                                         arraySeparator,
                                                         arrayStart,
                                                         arrayEnd,
                                                         stringQuote,
                                                         characterQuote);

                    if(value.empty() && default_also)
                    {
                        if(!opt->get_default_str().empty())
                        {
                            value = detail::convert_arg_for_ini(opt->get_default_str(), stringQuote, characterQuote);
                        }
                        else if(opt->get_expected_min() == 0)
                        {
                            value = "false";
                        }
                        else if(opt->get_run_callback_for_default())
                        {
                            value = "\"\"";  // empty string default value
                        }
                    }

                    if(!value.empty())
                    {
                        if(write_description && opt->has_description())
                        {
                            out << '\n';
                            out << commentLead << detail::fix_newlines(commentLead, opt->get_description()) << '\n';
                        }
                        out << name << valueDelimiter << value << '\n';
                    }
                }
            }
        }
        auto subcommands = app->get_subcommands({});
        for(const App *subcom: subcommands)
        {
            if(subcom->get_name().empty())
            {
                if(write_description && !subcom->get_group().empty())
                {
                    out << '\n' << commentLead << subcom->get_group() << " Options\n";
                }
                out << to_config(subcom, default_also, write_description, prefix);
            }
        }

        for(const App *subcom: subcommands)
        {
            if(!subcom->get_name().empty())
            {
                if(subcom->get_configurable() && app->got_subcommand(subcom))
                {
                    if(!prefix.empty() || app->get_parent() == nullptr)
                    {
                        out << '[' << prefix << subcom->get_name() << "]\n";
                    }
                    else
                    {
                        std::string subname = app->get_name() + "." + subcom->get_name();
                        auto p              = app->get_parent();
                        while(p->get_parent() != nullptr)
                        {
                            subname = p->get_name() + "." + subname;
                            p       = p->get_parent();
                        }
                        out << '[' << subname << "]\n";
                    }
                    out << to_config(subcom, default_also, write_description, "");
                }
                else
                {
                    out << to_config(subcom, default_also, write_description, prefix + subcom->get_name() + ".");
                }
            }
        }

        return out.str();
    }

    inline std::string Formatter::make_group(std::string group,
                                             bool is_positional,
                                             std::vector<const Option *> opts) const
    {
        std::stringstream out;

        out << "\n" << group << ":\n";
        for(const Option *opt: opts)
        {
            out << make_option(opt, is_positional);
        }

        return out.str();
    }

    inline std::string Formatter::make_positionals(const App *app) const
    {
        std::vector<const Option *> opts = app->get_options(
            [](const Option *opt)
            {
                return !opt->get_group().empty() && opt->get_positional();
            });

        if(opts.empty())
            return std::string();

        return make_group(get_label("Positionals"), true, opts);
    }

    inline std::string Formatter::make_groups(const App *app, AppFormatMode mode) const
    {
        std::stringstream out;
        std::vector<std::string> groups = app->get_groups();

        // Options
        for(const std::string &group: groups)
        {
            std::vector<const Option *> opts = app->get_options(
                [app, mode, &group](const Option *opt)
                {
                    return opt->get_group() == group                     // Must be in the right group
                           && opt->nonpositional()                       // Must not be a positional
                           && (mode != AppFormatMode::Sub                // If mode is Sub, then
                               || (app->get_help_ptr() != opt            // Ignore help pointer
                                   && app->get_help_all_ptr() != opt));  // Ignore help all pointer
                });
            if(!group.empty() && !opts.empty())
            {
                out << make_group(group, false, opts);

                if(group != groups.back())
                    out << "\n";
            }
        }

        return out.str();
    }

    inline std::string Formatter::make_description(const App *app) const
    {
        std::string desc = app->get_description();
        auto min_options = app->get_require_option_min();
        auto max_options = app->get_require_option_max();
        if(app->get_required())
        {
            desc += " REQUIRED ";
        }
        if((max_options == min_options) && (min_options > 0))
        {
            if(min_options == 1)
            {
                desc += " \n[Exactly 1 of the following options is required]";
            }
            else
            {
                desc += " \n[Exactly " + std::to_string(min_options) + "options from the following list are required]";
            }
        }
        else if(max_options > 0)
        {
            if(min_options > 0)
            {
                desc += " \n[Between " + std::to_string(min_options) + " and " + std::to_string(max_options) +
                        " of the follow options are required]";
            }
            else
            {
                desc += " \n[At most " + std::to_string(max_options) + " of the following options are allowed]";
            }
        }
        else if(min_options > 0)
        {
            desc += " \n[At least " + std::to_string(min_options) + " of the following options are required]";
        }
        return (!desc.empty()) ? desc + "\n" : std::string{};
    }

    inline std::string Formatter::make_usage(const App *app, std::string name) const
    {
        std::stringstream out;

        out << get_label("Usage") << ":" << (name.empty() ? "" : " ") << name;

        std::vector<std::string> groups = app->get_groups();

        // Print an Options badge if any options exist
        std::vector<const Option *> non_pos_options = app->get_options(
            [](const Option *opt)
            {
                return opt->nonpositional();
            });
        if(!non_pos_options.empty())
            out << " [" << get_label("OPTIONS") << "]";

        // Positionals need to be listed here
        std::vector<const Option *> positionals = app->get_options(
            [](const Option *opt)
            {
                return opt->get_positional();
            });

        // Print out positionals if any are left
        if(!positionals.empty())
        {
            // Convert to help names
            std::vector<std::string> positional_names(positionals.size());
            std::transform(positionals.begin(),
                           positionals.end(),
                           positional_names.begin(),
                           [this](const Option *opt)
                           {
                               return make_option_usage(opt);
                           });

            out << " " << detail::join(positional_names, " ");
        }

        // Add a marker if subcommands are expected or optional
        if(!app->get_subcommands(
                   [](const CLI::App *subc)
                   {
                       return ((!subc->get_disabled()) && (!subc->get_name().empty()));
                   })
                .empty())
        {
            out << " " << (app->get_require_subcommand_min() == 0 ? "[" : "")
                << get_label(app->get_require_subcommand_max() < 2 || app->get_require_subcommand_min() > 1
                                 ? "SUBCOMMAND"
                                 : "SUBCOMMANDS")
                << (app->get_require_subcommand_min() == 0 ? "]" : "");
        }

        out << std::endl;

        return out.str();
    }

    inline std::string Formatter::make_footer(const App *app) const
    {
        std::string footer = app->get_footer();
        if(footer.empty())
        {
            return std::string{};
        }
        return footer + "\n";
    }

    inline std::string Formatter::make_help(const App *app, std::string name, AppFormatMode mode) const
    {
        // This immediately forwards to the make_expanded method. This is done this way so that subcommands can
        // have overridden formatters
        if(mode == AppFormatMode::Sub)
            return make_expanded(app);

        std::stringstream out;
        if((app->get_name().empty()) && (app->get_parent() != nullptr))
        {
            if(app->get_group() != "Subcommands")
            {
                out << app->get_group() << ':';
            }
        }

        out << make_description(app);
        out << make_usage(app, name);
        out << make_positionals(app);
        out << make_groups(app, mode);
        out << make_subcommands(app, mode);
        out << '\n' << make_footer(app);

        return out.str();
    }

    inline std::string Formatter::make_subcommands(const App *app, AppFormatMode mode) const
    {
        std::stringstream out;

        std::vector<const App *> subcommands = app->get_subcommands({});

        // Make a list in definition order of the groups seen
        std::vector<std::string> subcmd_groups_seen;
        for(const App *com: subcommands)
        {
            if(com->get_name().empty())
            {
                if(!com->get_group().empty())
                {
                    out << make_expanded(com);
                }
                continue;
            }
            std::string group_key = com->get_group();
            if(!group_key.empty() && std::find_if(subcmd_groups_seen.begin(),
                                                  subcmd_groups_seen.end(),
                                                  [&group_key](std::string a)
                                                  {
                                                      return detail::to_lower(a) == detail::to_lower(group_key);
                                                  }) == subcmd_groups_seen.end())
                subcmd_groups_seen.push_back(group_key);
        }

        // For each group, filter out and print subcommands
        for(const std::string &group: subcmd_groups_seen)
        {
            out << "\n" << group << ":\n";
            std::vector<const App *> subcommands_group = app->get_subcommands(
                [&group](const App *sub_app)
                {
                    return detail::to_lower(sub_app->get_group()) == detail::to_lower(group);
                });
            for(const App *new_com: subcommands_group)
            {
                if(new_com->get_name().empty())
                    continue;
                if(mode != AppFormatMode::All)
                {
                    out << make_subcommand(new_com);
                }
                else
                {
                    out << new_com->help(new_com->get_name(), AppFormatMode::Sub);
                    out << "\n";
                }
            }
        }

        return out.str();
    }

    inline std::string Formatter::make_subcommand(const App *sub) const
    {
        std::stringstream out;
        detail::format_help(out, sub->get_display_name(true), sub->get_description(), column_width_);
        return out.str();
    }

    inline std::string Formatter::make_expanded(const App *sub) const
    {
        std::stringstream out;
        out << sub->get_display_name(true) << "\n";

        out << make_description(sub);
        if(sub->get_name().empty() && !sub->get_aliases().empty())
        {
            detail::format_aliases(out, sub->get_aliases(), column_width_ + 2);
        }
        out << make_positionals(sub);
        out << make_groups(sub, AppFormatMode::Sub);
        out << make_subcommands(sub, AppFormatMode::Sub);

        // Drop blank spaces
        std::string tmp = detail::find_and_replace(out.str(), "\n\n", "\n");
        tmp             = tmp.substr(0, tmp.size() - 1);  // Remove the final '\n'

        // Indent all but the first line (the name)
        return detail::find_and_replace(tmp, "\n", "\n  ") + "\n";
    }

    inline std::string Formatter::make_option_name(const Option *opt, bool is_positional) const
    {
        if(is_positional)
            return opt->get_name(true, false);

        return opt->get_name(false, true);
    }

    inline std::string Formatter::make_option_opts(const Option *opt) const
    {
        std::stringstream out;

        if(!opt->get_option_text().empty())
        {
            out << " " << opt->get_option_text();
        }
        else
        {
            if(opt->get_type_size() != 0)
            {
                if(!opt->get_type_name().empty())
                    out << " " << get_label(opt->get_type_name());
                if(!opt->get_default_str().empty())
                    out << "=" << opt->get_default_str();
                if(opt->get_expected_max() == detail::expected_max_vector_size)
                    out << " ...";
                else if(opt->get_expected_min() > 1)
                    out << " x " << opt->get_expected();

                if(opt->get_required())
                    out << " " << get_label("REQUIRED");
            }
            if(!opt->get_envname().empty())
                out << " (" << get_label("Env") << ":" << opt->get_envname() << ")";
            if(!opt->get_needs().empty())
            {
                out << " " << get_label("Needs") << ":";
                for(const Option *op: opt->get_needs())
                    out << " " << op->get_name();
            }
            if(!opt->get_excludes().empty())
            {
                out << " " << get_label("Excludes") << ":";
                for(const Option *op: opt->get_excludes())
                    out << " " << op->get_name();
            }
        }
        return out.str();
    }

    inline std::string Formatter::make_option_desc(const Option *opt) const
    {
        return opt->get_description();
    }

    inline std::string Formatter::make_option_usage(const Option *opt) const
    {
        // Note that these are positionals usages
        std::stringstream out;
        out << make_option_name(opt, true);
        if(opt->get_expected_max() >= detail::expected_max_vector_size)
            out << "...";
        else if(opt->get_expected_max() > 1)
            out << "(" << opt->get_expected() << "x)";

        return opt->get_required() ? out.str() : "[" + out.str() + "]";
    }

}  // namespace CLI