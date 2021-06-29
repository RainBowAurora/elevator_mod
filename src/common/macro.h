#pragma once

// Put this in the declarations for a class to be uncopyable.
#define DISABLE_COPY(TypeName) TypeName(const TypeName &) = delete

// Put this in the declarations for a class to be unassignable.
#define DISABLE_ASSIGN(TypeName) void operator=(const TypeName &) = delete

// A macro to disallow the copy constructor and operator= functions.
// This should be used in the private: declarations for a class.
#define DISABLE_COPY_AND_ASSIGN(TypeName) \
  DISABLE_COPY(TypeName);                 \
  DISABLE_ASSIGN(TypeName)

#define DISABLE_IMPLICIT_CONSTRUCTORS(TypeName) \
  TypeName() = delete;                          \
  DISABLE_COPY_AND_ASSIGN(TypeName)
