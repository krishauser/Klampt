#if defined (__APPLE__) || defined (MACOSX)
  #include <Availability.h>
  #if __MAC_OS_X_VERSION_MIN_REQUIRED < 1070
    // earlier than 10.6, C++11 items are in the tr1 namespace
    #include <tr1/memory>
    using namespace std;
    #define shared_ptr tr1::shared_ptr

    //make_shared is not defined
    // zero arguments version
    template <typename T>
    inline shared_ptr<T> make_shared()
    {
      return shared_ptr<T>(new T());
    }

    // one argument version
    template <typename T, typename Arg1>
    inline shared_ptr<T> make_shared(Arg1&& arg1)
    {
      return shared_ptr<T>(new T(std::forward<Arg1>(arg1)));
    }

    // two arguments version
    template <typename T, typename Arg1, typename Arg2>
    inline shared_ptr<T> make_shared(Arg1&& arg1, Arg2&& arg2)
    {
      return shared_ptr<T>(new T(std::forward<Arg1>(arg1),
                                 std::forward<Arg2>(arg2)));
    }

    // three arguments version
    template <typename T, typename Arg1, typename Arg2, typename Arg3>
    inline shared_ptr<T> make_shared(Arg1&& arg1, Arg2&& arg2, Arg3&& arg3)
    {
      return shared_ptr<T>(new T(std::forward<Arg1>(arg1),
                                 std::forward<Arg2>(arg2),
                                 std::forward<Arg3>(arg3)));
    }

    // four arguments version
    template <typename T, typename Arg1, typename Arg2, typename Arg3, typename Arg3>
    inline shared_ptr<T> make_shared(Arg1&& arg1, Arg2&& arg2, Arg3&& arg3, Arg4&& arg4)
    {
      return shared_ptr<T>(new T(std::forward<Arg1>(arg1),
                                 std::forward<Arg2>(arg2),
                                 std::forward<Arg3>(arg3),
                                 std::forward<Arg4>(arg4)));
    }
  #endif 
#endif