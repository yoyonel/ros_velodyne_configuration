#ifndef SERVICELOCATOR_H
#define SERVICELOCATOR_H

// Service Locator.  See http://martinfowler.com/articles/injection.html
// Version 1
// Kenneth Kasajian

//
// Example usage:
//
// 1. Derive your interfaces (pure-virtual abstract classes) from sl::interface_t.
// Example:
//		struct MovieFinder : sl::interface_t
//		{
//			virtual vector<Movie> findAll() = 0;
//		};
//
//
// 2. Derive your template classes from sl::member_t, passing in the class name as the template parameter
// Example:
//
//		class ColonDelimitedMovieFinder : public sl::member_t<ColonDelimitedMovieFinder>
//		{ ...
//
//
// 3. Create an instance of sl::servicelocator_t.
// Example:
//		sl::servicelocator_t locator;
//
//
// 4. Register classes using register_class method.  The class to be registered is the template parameter.  The function parameter is the string id representing the class.
// Example:
//		locator.register_class<ColonDelimitedMovieFinder>( "finder" );
//
//
// 5. Now, the string id (such as "finder") above can be created an instance of ColonDelimitedMovieFinder, without referncing ColonDelimitedMovieFinder by name.
//    Assuming that ColonDelimitedMovieFinder is redefined to implement the MovieFinder interface like so:
//		class ColonDelimitedMovieFinder : public sl::member_t<ColonDelimitedMovieFinder>, public MovieFinder
//    then there are two methods of creating an instance of ColonDelimitedMovieFinder.
//	  One as a singleton (so that subsequent calls to create all return the same instance)
//    And one as a transient instance, where a new instance is created with each call.
//
//    The call to create a singleton instance is get_single_instance, and it looks like this:
//		MovieFinder* finder = locator.get_single_instance<MovieFinder>( "finder" );
//	  (again, note that the class ColonDelimitedMovieFinder is not referenced, but that's what's created because the string id "finder" was registered with ColonDelimitedMovieFinder above.)
//    Note that the object return from get_single_instance does not need to be freed.  Only a single instance is created and is owned by the service locator.
//	  When the 'locator' object goes out of scope, or is freed, all singletons are freed.
//
//    The second way to create an instance is via get_new_instance, and it looks like this:
//    	auto_ptr<MovieFinder> finder = locator.get_new_instance<MovieFinder>( "finder" );
//    For convenience, get_new_instance returns an auto_ptr, which is the safest thing to return.
//    If the client doesn'w an auto_ptr object, it's simple enough to call release() and get the raw object which
//    can then be managed by the client as desired.  However, in that case, the client is responsible for calling delete on it.
//

#include <memory>
#include <string>
#include <map>

using namespace std;

namespace sl
{
struct interface_t
{
    virtual ~interface_t() {}
};

template<class T>
class member_t
{
public:
    static interface_t* create()
    {
        return new T();
    }
};

class servicelocator_t
{
public:
    typedef map<string, interface_t*(*)()>	_classes_t;
    _classes_t								_classes;
    typedef map<string, interface_t*>		_singletons_t;
    _singletons_t							_singletons;

    ~servicelocator_t()
    {
        for ( _singletons_t::iterator it = _singletons.begin(); it != _singletons.end(); it++ )
            delete it->second;
    }

    template<class T>
    void register_class( const string& id )
    {
        _classes.insert( make_pair( id, T::create) );
    }

    // Gets a transient instance: each call creates and returns a reference to a new object
    template<class T>
    auto_ptr<T> get_new_instance( const string& id )
    {
        _classes_t::iterator found = _classes.find(id);
        if ( found != _classes.end() )
            return auto_ptr<T>( dynamic_cast<T*>( found->second() ) );
        throw runtime_error( "invalid id: " + id );
    }

    template<class T>
    T* get_single_instance( const string& id )
    {
        _singletons_t::iterator found_singleton = _singletons.find(id);
        if ( found_singleton != _singletons.end() )
            return dynamic_cast<T*>( found_singleton->second );

        _classes_t::iterator found_class = _classes.find(id);
        if ( found_class != _classes.end() )
        {
            T* obj( dynamic_cast<T*>( found_class->second() ) );
            _singletons.insert( make_pair( id, obj) );
            return obj;
        }
        throw runtime_error( "invalid id: " + id );
    }
};
}

#endif // SERVICELOCATOR_H
