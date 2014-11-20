#ifndef SINGLETON_H
#define SINGLETON_H

#define DEF_SINGLETON( NAME )    \
public:                        \
   static NAME& instance()      \
   {                            \
      static NAME _instance;    \
      return _instance;         \
   }                            \
private:                       \
   NAME();               \
   NAME( const NAME& );

#endif // SINGLETON_H
