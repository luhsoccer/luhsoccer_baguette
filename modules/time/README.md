# Time Tool {#time}
The `time` tool implements timing functions for all other modules in this project. Whenever you want to use time you should use this package, as handles for fast-forward and slow motion will be implemented later. If you use this tool you do not have to worry about these features. The `time` tool bases on the [std::chrono](https://en.cppreference.com/w/cpp/chrono) module of c++. Furthermore, this tool is header-only. There is no content in the time.cpp. The smallest possible time difference is `1ns` and the maximum trackable difference in time is roughly `292 years`. The time is defined by the start of the program (here the time is 0). It increases over time and can be negative in general (even though that should not be useful). 

>  Note that durations and points in time of this tool do not have to correspond to real world seconds or dates. The program could run in slow motion or fast forward state. 
## Objects
This tool implements several time objects in the `luhsoccer::time` namespace in the [time.hpp](include/time/time.hpp)

## `TimePoint`
The `TimePoint` object represents a specific point in time. It is derived from `std::chrono::time_point<std::chrono::steady_clock>` ([see](https://en.cppreference.com/w/cpp/chrono/time_point)). However, the point in time can be defined by a double representing the seconds since the start of the program. Additionally, the point in time can be converted with `toSec()` and `toNSec()` to seconds and nanoseconds since start of the program, respectively. Basic arithmetics like subtractions of two ``TimePoints`` resulting in a ``Duration`` or adding a ``Duration`` to a ``TimePoint`` resulting in a ``TimePoint`` is implemented as well.

## `Duration`
The `Duration` objects represents a difference between two `TimePoints`. It is derived from `std::chrono::nanoseconds`
([see](https://en.cppreference.com/w/cpp/chrono/duration)). ``Durations`` can be defined by a double representing seconds, or two ints representing seconds and nanoseconds. They can be transformed to seconds or nanoseconds like `TimePoints`. Here also the basic arithmetics are implemented, as well as the multiplication of a ``Duration`` with a double resulting in a `Duration`.

## `Rate`
``Rate`` is a helper class to maintain a steady frequency in a while loop. It is constructed with a constant frequency in Hz. The `sleep()` method sleeps the thread so that the loop always runs with the defined frequency. For that the objects measures the execution time of the loop and sleeps for the difference. Additionally it prints a warning if the loop time could not be met.#

### Example implementation:
```cpp
#include "time/time.hpp"
namespace luhsoccer{
int main(){
    constexpr double frequency = 100;
    time::Rate rate(frequency);

    while (true) {
        // calculations...

        rate.sleep();
    }
    return 0;
}
    
}
```

## `LoopStopwatch`
The `LoopStopwatch` object measures the frequency of a loop or callback with the `tik()` function. The `LoopStopwatch` can be constructed with a name and a desired frequency and a window size. The window size defines how many measurements are used to calculate the frequency. If the window size is zero, all measurements are used. The `measuredFrequency()` method calculates the measured frequency and the `printResult()` method prints a log message.

### Example implementation:
```cpp
#include "time/time.hpp"
namespace luhsoccer{

class Foo{
   public:
    Foo(): watch("foo-stopwatch",100.0,20){
        //register callback...
    }

    time::LoopStopwatch watch;
    void callback(){
        watch.tik();
    }   
}

int main(){
    Foo f;
    while(true){
        f.watch.printResult();
    }
}
}
```

## Methods
The `time` tool also implements some static methods.

## ``timeSinceStart()``
This method is used to get the `Duration` since the start of the program. This method will also consider slow motion or fast forward states.

## `now()`
This method is quite similar to ``timeSinceStart()`` with the difference that this functions returns the current point in time. As the time in our project is defined since the start of the program it returns the same value (in a different format) as ``timeSinceStart()``.

