template <typename T>
class Singleton {
public:
    // Public static method to access the single instance
    static T& getInstance() {
        // The static local variable is created only once, on the first call.
        // This creation is thread-safe in C++11 and later.
        static T instance; 
        return instance;
    }

    // Delete copy constructor and assignment operator to prevent copying
    Singleton(Singleton const&) = delete;
    void operator=(Singleton const&) = delete;

protected:
    // Protected constructor and destructor to prevent direct instantiation/deletion
    Singleton() = default;
    ~Singleton() = default;
};
