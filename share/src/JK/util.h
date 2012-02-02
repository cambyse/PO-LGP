#define JK_VERBOSE

#if defined JK_VERBOSE
#define JK_DEBUG(var) cout << "@[" << __FILE__ << ":" << __LINE__ << "]: " << #var << " = " << var << endl
#else
#define JK_DEBUG(var) 
#endif

#define foreach_(container) for(typeof(container)::iterator it = container.begin(); it < container.end(); ++it) 
