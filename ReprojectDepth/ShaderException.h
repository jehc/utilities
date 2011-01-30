#ifndef SHADER_EXCEPTION_H
#define SHADER_EXCEPTION_h

#include <exception>

class ShaderException : public std::exception
{
public:
enum ExceptionType { CompileError, LinkError };

private:
ExceptionType type;
std::string message;
public:
ShaderException ( ExceptionType type, const std::string & message ) throw( )
  : type ( type ), message ( message )
{
  std::cerr << message << std::endl;
}

~ShaderException() throw( )
{
}

const char * what () throw( )
{
  std::stringstream ss;

  switch ( type )
  {
  case CompileError:
    ss << "CompileError: ";
    break;
  case LinkError:
    ss << "LinkError: ";
    break;
  default:
    throw std::exception ();
    break;
  }
  ss << message;
  return ss.str ().c_str ();
}
};


#endif
